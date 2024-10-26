
#ifndef DIFFBOT_BASE_CONTROLLER_H
#define DIFFBOT_BASE_CONTROLLER_H


#include <ros.h>
#include <diffbot_msgs/EncodersStamped.h>
#include <diffbot_msgs/WheelsCmdStamped.h>
#include <diffbot_msgs/AngularVelocities.h>
#include <diffbot_msgs/PIDStamped.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <cstdio>  // Thêm dòng này để dùng sprintf

#include "diffbot_base_config_v3.h"
#include "encoder_diffbot_mod_stm32.h"
#include "l298_motor_driver.h"
#include "pid.h"
#include <string>


#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "tim.h"
#include "mainpp.h"
#ifdef __cplusplus
}
#endif

namespace diffbot {
    template <typename L298MotorController>
    class BaseController
    {
    public:

        BaseController(ros::NodeHandle &nh, L298MotorController* motor_controller_left, L298MotorController* motor_controller_right);

        struct UpdateRate
        {
            double imu_;
            double control_;
            double debug_;

            struct Period
            {
                double imu_;
                double control_;
                double debug_;

                inline Period(double imu_frequency, double control_frequency, double debug_frequency)
                    : imu_(1.0 / imu_frequency)
                    , control_(1.0 / control_frequency)
                    , debug_(1.0 / debug_frequency) {};
            } period_;

            inline Period& period() { return period_; };

            inline UpdateRate(double imu_frequency,
                               double control_frequency,
                               double debug_frequency)
                : imu_(imu_frequency)
                , control_(control_frequency)
                , debug_(debug_frequency)
                , period_(imu_frequency, control_frequency, debug_frequency) {};
        } update_rate_;

        inline int period(double frequency) { return 1 / frequency; };

        inline UpdateRate& publishRate() { return update_rate_; };

        struct LastUpdateTime
        {
            ros::Time command_received;
            ros::Time control;
            ros::Time imu;
            ros::Time debug;
            inline LastUpdateTime(ros::Time start)
                : command_received(start.toSec(), start.toNsec())
                , control(start.toSec(), start.toNsec())
                , imu(start.toSec(), start.toNsec())
                , debug(start.toSec(), start.toNsec()) {};
        } last_update_time_;
        inline LastUpdateTime& lastUpdateTime() { return last_update_time_; };
        inline bool debug() { return debug_; };
        void setup();
        void init();
        void eStop();
        void read();
        void write();
        void printDebug();
        void commandCallback(const diffbot_msgs::WheelsCmdStamped& cmd_msg);
        void resetEncodersCallback(const std_msgs::Empty& reset_msg);
        void pidLeftCallback(const diffbot_msgs::PIDStamped& pid_msg);
        void pidRightCallback(const diffbot_msgs::PIDStamped& pid_msg);

    private:
        ros::NodeHandle& nh_;

        // constants
        float wheel_radius_ = 0.0;
        float max_linear_velocity_ = 0.0;
        float max_angular_velocity_ = 0.0;
        diffbot::Encoder encoder_left_;
        diffbot::Encoder encoder_right_;
        long ticks_left_ = 0, ticks_right_ = 0;
        diffbot::JointState joint_state_left_, joint_state_right_;

        int encoder_resolution_;

        // ros::Subscriber<std_msgs::Empty, BaseController<L298MotorController, TMotorDriver>> sub_reset_encoders_;
        ros::Subscriber<std_msgs::Empty, BaseController<L298MotorController>> sub_reset_encoders_;

        // ROS Publisher setup to publish left and right encoder ticks
        // This uses the custom encoder ticks message that defines an array of two integers
        diffbot_msgs::EncodersStamped encoder_msg_;
        ros::Publisher pub_encoders_;

        sensor_msgs::JointState msg_measured_joint_states_;
        ros::Publisher pub_measured_joint_states_;

        // MotorControllerIntf<TMotorDriver>* p_motor_controller_right_;
        // MotorControllerIntf<TMotorDriver>* p_motor_controller_left_;
        // MotorControllerIntf* p_motor_controller_right_;
        // MotorControllerIntf* p_motor_controller_left_;
        L298MotorController* p_motor_controller_left_;
        L298MotorController* p_motor_controller_right_;

        ros::Subscriber<diffbot_msgs::WheelsCmdStamped, BaseController<L298MotorController>> sub_wheel_cmd_velocities_;

        float wheel_cmd_velocity_left_ = 0.0;
        float wheel_cmd_velocity_right_ = 0.0;

        int motor_cmd_left_ = 150;
        int motor_cmd_right_ = 750;

        ros::Subscriber<diffbot_msgs::PIDStamped, BaseController<L298MotorController>> sub_pid_left_;
        ros::Subscriber<diffbot_msgs::PIDStamped, BaseController<L298MotorController>> sub_pid_right_;
        PID motor_pid_left_;
        PID motor_pid_right_;

        // DEBUG
        bool debug_;
    };


}

template <typename L298MotorController>
using BC = diffbot::BaseController<L298MotorController>;

template <typename L298MotorController>
diffbot::BaseController<L298MotorController>
    ::BaseController(ros::NodeHandle &nh, L298MotorController* motor_controller_left, L298MotorController* motor_controller_right)
    : nh_(nh)
    , encoder_left_(nh, &htim2, ENCODER_RESOLUTION)
    , encoder_right_(nh, &htim5, ENCODER_RESOLUTION)
    , sub_reset_encoders_("reset", &BC<L298MotorController>::resetEncodersCallback, this)
    , pub_encoders_("encoder_ticks", &encoder_msg_)
    , pub_measured_joint_states_("measured_joint_states", &msg_measured_joint_states_)
    , sub_wheel_cmd_velocities_("wheel_cmd_velocities", &BC<L298MotorController>::commandCallback, this)
    , last_update_time_(nh.now())
    , update_rate_(UPDATE_RATE_IMU, UPDATE_RATE_CONTROL, UPDATE_RATE_DEBUG)
    , sub_pid_left_("pid_left", &BC<L298MotorController>::pidLeftCallback, this)
    , sub_pid_right_("pid_right", &BC<L298MotorController>::pidRightCallback, this)
    , motor_pid_left_(PWM_MIN, PWM_MAX, K_P, K_I, K_D)
    , motor_pid_right_(PWM_MIN, PWM_MAX, K_P, K_I, K_D)

{
    p_motor_controller_left_ = motor_controller_left;
    p_motor_controller_right_ = motor_controller_right;
}

template <typename L298MotorController>
void diffbot::BaseController<L298MotorController>::setup()
{
    nh_.initNode();
    nh_.advertise(pub_encoders_);
    // // msg_measured_joint_states_.position = (float*)malloc(sizeof(float) * 2);
    msg_measured_joint_states_.position = (double*)malloc(sizeof(double) * 2);
    msg_measured_joint_states_.position_length = 2;
    msg_measured_joint_states_.velocity = (double*)malloc(sizeof(double) * 2);
    msg_measured_joint_states_.velocity_length = 2;

    nh_.advertise(pub_measured_joint_states_);

    nh_.subscribe(sub_wheel_cmd_velocities_);
    nh_.subscribe(sub_reset_encoders_);

    nh_.subscribe(sub_pid_left_);
    nh_.subscribe(sub_pid_right_);

    while (!nh_.connected())
    {
        nh_.spinOnce();
    }
}

template <typename L298MotorController>
void diffbot::BaseController<L298MotorController>::init()
{
    nh_.loginfo("Get Parameters from Parameter Server");
    nh_.getParam("/diffbot/encoder_resolution", &this->encoder_resolution_);
    // std::string log_msg = std::string("/diffbot/encoder_resolution: ") + std::to_string(encoder_resolution_);
    // nh_.loginfo(log_msg.c_str());
    nh_.getParam("/diffbot/mobile_base_controller/wheel_radius", &wheel_radius_);
    // log_msg = std::string("/diffbot/mobile_base_controller/wheel_radius: ") + std::to_string(wheel_radius_);
    // nh_.loginfo(log_msg.c_str());
    nh_.getParam("/diffbot/mobile_base_controller/linear/x/max_velocity", &max_linear_velocity_);
    // log_msg = std::string("/diffbot/mobile_base_controller/linear/x/max_velocity: ") + std::to_string(max_linear_velocity_);
    // nh_.loginfo(log_msg.c_str());
    nh_.getParam("/diffbot/debug/base_controller", &debug_);
    // log_msg = std::string("/diffbot/debug/base_controller: ") + std::to_string(debug_);
    // nh_.loginfo(log_msg.c_str());

    // nh_.loginfo("Initialize DiffBot Wheel Encoders");
    encoder_left_.resolution(encoder_resolution_);
    encoder_right_.resolution(encoder_resolution_);

    std_msgs::Empty reset;
    this->resetEncodersCallback(reset);
    HAL_Delay(1);

    max_angular_velocity_ = max_linear_velocity_ / wheel_radius_;

    HAL_Delay(1000);
}
template <typename L298MotorController>
void diffbot::BaseController<L298MotorController>::commandCallback(const diffbot_msgs::WheelsCmdStamped& cmd_msg)
{
    wheel_cmd_velocity_left_ = cmd_msg.wheels_cmd.angular_velocities.joint[0];
    wheel_cmd_velocity_right_ = cmd_msg.wheels_cmd.angular_velocities.joint[1];

    lastUpdateTime().command_received = nh_.now();
    /////////////////////////////////              check wheel_cmd_velocities       /////////////////////////
    std::string log_msg = std::string("wheel_cmd_") + std::to_string(wheel_cmd_velocity_left_);
    nh_.logwarn(log_msg.c_str());
}

// ROS Subscriber setup to reset both encoders to zero
template <typename L298MotorController>
void diffbot::BaseController<L298MotorController>::resetEncodersCallback(const std_msgs::Empty& reset_msg)
{
    // reset both back to zero.
    this->encoder_left_.write(0);
    this->encoder_right_.write(0);
    this->nh_.loginfo("Reset both wheel encoders to zero");
}
template <typename L298MotorController>
void diffbot::BaseController<L298MotorController>::pidLeftCallback(const diffbot_msgs::PIDStamped& pid_msg)
{
    motor_pid_left_.updateConstants(pid_msg.pid.kp, pid_msg.pid.ki, pid_msg.pid.kd);
    motor_pid_left_.updateConstants(pid_msg.pid.kp, pid_msg.pid.ki, pid_msg.pid.kd);
}
template <typename L298MotorController>
void diffbot::BaseController<L298MotorController>::pidRightCallback(const diffbot_msgs::PIDStamped& pid_msg)
{
    motor_pid_left_.updateConstants(pid_msg.pid.kp, pid_msg.pid.ki, pid_msg.pid.kd);
    motor_pid_left_.updateConstants(pid_msg.pid.kp, pid_msg.pid.ki, pid_msg.pid.kd);
}

template <typename L298MotorController>
void diffbot::BaseController<L298MotorController>::read()
{
    joint_state_left_ = encoder_left_.jointState();
    joint_state_right_ = encoder_right_.jointState();

    msg_measured_joint_states_.position[0] = joint_state_left_.angular_position_;
    msg_measured_joint_states_.position[1] = joint_state_right_.angular_position_;

    msg_measured_joint_states_.velocity[0] = joint_state_left_.angular_velocity_;
    msg_measured_joint_states_.velocity[1] = joint_state_right_.angular_velocity_;
    pub_measured_joint_states_.publish(&msg_measured_joint_states_);

    // get the current tick count of each encoder
    ticks_left_ = encoder_left_.read();
    ticks_right_ = encoder_right_.read();

    encoder_msg_.encoders.ticks[0] = ticks_left_;
    encoder_msg_.encoders.ticks[1] = ticks_right_;
    // Avoid having too many publishers
    // Otherwise error like 'wrong checksum for topic id and msg'
    // and 'Write timeout: Write timeout' happen.
    // pub_encoders_.publish(&encoder_msg_);
}
template <typename L298MotorController>
void diffbot::BaseController<L298MotorController>::write()
{
    motor_cmd_left_ = motor_pid_left_.compute(wheel_cmd_velocity_left_, joint_state_left_.angular_velocity_);
    motor_cmd_right_ = motor_pid_right_.compute(wheel_cmd_velocity_right_, joint_state_right_.angular_velocity_);
    // motor_cmd_left_ = 250;
    // motor_cmd_right_ = 750;

    // std::string log_msg3 = std::string("joint_") + std::to_string(joint_state_left_.angular_velocity_);
    // nh_.logwarn(log_msg3.c_str());

    // std::string log_msg4 = std::string("motor_") + std::to_string(motor_cmd_left_);
    // nh_.logwarn(log_msg4.c_str());

    // // Set motor speeds using the motor controller
    p_motor_controller_left_->setSpeed(motor_cmd_left_);
    p_motor_controller_right_->setSpeed(motor_cmd_right_);

}
template <typename L298MotorController>
void diffbot::BaseController<L298MotorController>::eStop()
{
    wheel_cmd_velocity_left_ = 0;
    wheel_cmd_velocity_right_ = 0;
}
template <typename L298MotorController>
void diffbot::BaseController<L298MotorController>::printDebug()
{
    // std::string log_msg1 =
    //     std::string("\nRead:\n") +
    //     "ticks_left_: " + std::to_string(ticks_left_) +
    //     "\nticks_right_: " + std::to_string(ticks_right_) +
    //     "\nmeasured_ang_vel_left: " + std::to_string(joint_state_left_.angular_velocity_) +
    //     "\nmeasured_ang_vel_right: " + std::to_string(joint_state_right_.angular_velocity_) +
    //     "\nwheel_cmd_velocity_left_: " + std::to_string(wheel_cmd_velocity_left_) +
    //     "\nwheel_cmd_velocity_right_: " + std::to_string(wheel_cmd_velocity_right_);
    //     nh_.loginfo(log_msg1.c_str());

    // std::string log_msg2 =
    //     std::string("\nWrite:\n") +
    //     "motor_cmd_left_: " + std::to_string(motor_cmd_left_) +
    //     "\nmotor_cmd_right_: " + std::to_string(motor_cmd_right_) +
    //     "\npid_left_errors (p, i, d): " + std::to_string(motor_pid_left_.proportional()) + " " + std::to_string(motor_pid_left_.integral()) + " " + std::to_string(motor_pid_left_.derivative()) +
    //     "\npid_right_error (p, i, d): " + std::to_string(motor_pid_right_.proportional()) + " " + std::to_string(motor_pid_right_.integral()) + " " + std::to_string(motor_pid_right_.derivative());
    // // Chuyển đổi std::string thành const char* để dùng với nh_.loginfo
    // nh_.loginfo(log_msg2.c_str());
}


///////////////////////////////////////////////////////////////////////////////////////////////
#endif // DIFFBOT_BASE_CONTROLLE