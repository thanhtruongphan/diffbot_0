

#include <ros.h>

#include "diffbot_base_config_v3.h"
#include "base_controller_mod_v3.h"
// #include "adafruit_feather_wing/adafruit_feather_wing.h"
#include "l298_motor_driver.h"
#include "main.h"
#include "mainpp.h"

ros::NodeHandle nh;

using namespace diffbot;

L298MotorController motor_controller_right = L298MotorController(GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13, &htim3, TIM_CHANNEL_1);
L298MotorController motor_controller_left = L298MotorController(GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15, &htim4, TIM_CHANNEL_1);
BaseController<L298MotorController> base_controller(nh, &motor_controller_left, &motor_controller_right);

/////////////////////////////////            check encoders                   ///////////////////////////
extern TIM_HandleTypeDef htim2;
diffbot::Encoder encoder_l(nh, &htim2, 1980);
extern TIM_HandleTypeDef htim5;
diffbot::Encoder encoder_r(nh, &htim5, 1980);
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
    base_controller.setup();
    base_controller.init();
    nh.loginfo("Setup finished");
}


void loop()
{
    // static bool imu_is_initialized;

    // The main control loop for the base_conroller.
    // This block drives the robot based on a defined control rate
    ros::Duration command_dt = nh.now() - base_controller.lastUpdateTime().control;
    if (command_dt.toSec() >= ros::Duration(1.0 / base_controller.publishRate().control_, 0).toSec())
    {
        base_controller.read();
        base_controller.write();
        base_controller.lastUpdateTime().control = nh.now();
    }

    // This block stops the motor when no wheel command is received
    // from the high level hardware_interface::RobotHW
    command_dt = nh.now() - base_controller.lastUpdateTime().command_received;
    if (command_dt.toSec() >= ros::Duration(E_STOP_COMMAND_RECEIVED_DURATION, 0).toSec())
    {
        nh.logwarn("Emergency STOP");
        // base_controller.eStop();
    }

    // motor_controller_left.setSpeed(750);
    // motor_controller_right.setSpeed(250);
    // motor_controller_left.HAL_TIM_PeriodElapsedCallback(&htim3, 300);
    // motor_controller_right.HAL_TIM_PeriodElapsedCallback(&htim4, 750);
    // // This block publishes the IMU data based on a defined imu rate
    // ros::Duration imu_dt = nh.now() - base_controller.lastUpdateTime().imu;
    // if (imu_dt.toSec() >= base_controller.publishRate().period().imu_)
    // {
    //     // Sanity check if the IMU is connected
    //     if (!imu_is_initialized)
    //     {
    //         //imu_is_initialized = initIMU();
    //         if(imu_is_initialized)
    //             nh.loginfo("IMU Initialized");
    //         else
    //             nh.logfatal("IMU failed to initialize. Check your IMU connection.");
    //     }
    //     else
    //     {
    //         //publishIMU();
    //     }
    //     base_controller.lastUpdateTime().imu = nh.now();
    // }

    // This block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(base_controller.debug())
    {
        ros::Duration debug_dt = nh.now() - base_controller.lastUpdateTime().debug;
        if (debug_dt.toSec() >= base_controller.publishRate().period().debug_)
        {
            base_controller.printDebug();
            base_controller.lastUpdateTime().debug = nh.now();
        }
    }


    ///////////////////////                       check encoder           //////////////////////////////
    int rpm_l = encoder_l.getRPM();
    int rpm_r = encoder_r.getRPM();
    // std::string log_msg = std::string("encoder_ticks_getRPM_L:") + std::to_string(rpm_l) + std::string("encoder_ticks_getRPM_R:") + std::to_string(rpm_r);
    // nh.loginfo(log_msg.c_str());

    /////////////////////////////////////////////////////////////////////////////////////////////////////

    // Call all the callbacks waiting to be called
    nh.spinOnce();
    HAL_Delay(100);
}