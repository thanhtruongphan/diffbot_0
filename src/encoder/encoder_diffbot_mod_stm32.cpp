

#include "encoder_diffbot_mod_stm32.h"
#include <cstdio>
#include <string>

diffbot::Encoder::Encoder(ros::NodeHandle& nh, TIM_HandleTypeDef* htim, int encoder_resolution)
  : nh_(nh)
  , htim_(htim)
  , encoder_resolution_(encoder_resolution)
  , prev_update_time_(0, 0)
  , prev_encoder_ticks_(0)
{
    // Initialize the encoder (the hardware initialization is expected to be done elsewhere)
    // __HAL_TIM_SET_COUNTER(htim_, 0);  // Reset the encoder counter to zero
}

diffbot::JointState diffbot::Encoder::jointState()
{
    long encoder_ticks = read();
    ////////////////////////////////
    // char log_msg[10];
    // sprintf(log_msg, "encoder ticks... : %ld", encoder_ticks);
    // nh_.logwarn(log_msg);
    ///////////////////////////////
    ros::Time current_time = nh_.now();
    ros::Duration dt = current_time - prev_update_time_;

    double dts = dt.toSec();
    double delta_ticks = encoder_ticks - prev_encoder_ticks_;
    double delta_angle = ticksToAngle(delta_ticks);

    joint_state_.angular_position_ += delta_angle;
    joint_state_.angular_velocity_ = delta_angle / dts;

    prev_update_time_ = current_time;
    prev_encoder_ticks_ = encoder_ticks;

    return joint_state_;
}

double diffbot::Encoder::angularPosition()
{
    return joint_state_.angular_position_;
}

double diffbot::Encoder::angularVelocity()
{
    return joint_state_.angular_velocity_;
}

double diffbot::Encoder::ticksToAngle(const int &ticks) const
{
    // Convert encoder ticks to radians
    return (double)ticks * (2.0 * M_PI / encoder_resolution_);
}

int diffbot::Encoder::getRPM()
{
    long encoder_ticks = read();
    ros::Time current_time = nh_.now();
    ros::Duration dt = current_time - prev_update_time_;

    double dtm = dt.toSec() / 60.0;  // Convert time to minutes
    double delta_ticks = encoder_ticks - prev_encoder_ticks_;

    prev_update_time_ = current_time;
    prev_encoder_ticks_ = encoder_ticks;
    /////////////                    check encoder ticks             ////////////////
    std::string log_msg = std::string("encoder_ticks_:") + std::to_string(encoder_ticks)+ std::to_string(dtm)+ std::to_string(delta_ticks)+ std::to_string(encoder_resolution_);
    nh_.loginfo(log_msg.c_str());
    
    return (delta_ticks / encoder_resolution_) / dtm;
}
