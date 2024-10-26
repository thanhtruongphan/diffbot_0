


#ifndef DIFFBOT_ENCODER_H
#define DIFFBOT_ENCODER_H

#include <ros.h>
#include "stm32f4xx_hal.h" // Include STM32 HAL library
#include "main.h"
#include "mainpp.h"

namespace diffbot
{
    struct JointState
    {
        double angular_position_;
        double angular_velocity_;
    };

    class Encoder
    {
    public:
        /** \brief Construct a diffbot::Encoder providing access to quadrature encoder ticks and angular joint velocity.
         * 
         * \param nh reference to the main ros::NodeHandle to compute the velocity from time and ticks or angle (s = v * t)
         * \param htim Pointer to the STM32 timer handle used for encoder input (e.g. TIM1, TIM2)
         * \param encoder_resolution number of tick counts for one full revolution of the wheel (not the motor shaft).
         */
        Encoder(ros::NodeHandle& nh, TIM_HandleTypeDef* htim, int encoder_resolution);

        /** \brief Get revolutions per minute
         *
         * Calculates the wheels' revolutions per minute using encoder ticks.
         * 
         * \returns revolutions per minute
         */
        int getRPM();

        /** \brief Get the angular position in radians
         * 
         * \returns angular position (rad)
         */
        double angularPosition();

        /** \brief Get the angular velocity in rad/s
         *
         * \returns angular velocity (rad/s)
         */
        double angularVelocity();

        /** \brief Returns the current joint state (position, velocity)
         *
         * \returns JointState struct containing position and velocity
         */
        JointState jointState();

        /** \brief Convert number of encoder ticks to angle in radians 
         *
         * \param ticks tick count from an encoder which is converted to a corresponding absolute angle.
         * \returns angle corresponding to encoder ticks (rad)
         */
        double ticksToAngle(const int &ticks) const;

        /** \brief Read the current encoder tick count from the timer
         * 
         * \returns encoder ticks
         */
        inline int32_t read() { return __HAL_TIM_GET_COUNTER(htim_); };

        /** \brief Set the encoder tick count in the timer counter
         * 
         * \param p encoder ticks to set
         */
        inline void write(int32_t p) { __HAL_TIM_SET_COUNTER(htim_, p); };

        /** \brief Setter for encoder resolution
         * 
         * \param resolution value to which the encoder tick count should be set
         */
        inline void resolution(int resolution) { encoder_resolution_ = resolution; };

        /** \brief Getter for encoder resolution
         * 
         * \returns the currently set encoder resolution
         */
        inline int resolution() { return encoder_resolution_; };

        JointState joint_state_;

    private:
        // ROS node handle, used for getting the current time
        ros::NodeHandle& nh_;
        // Pointer to STM32 timer handle for encoder reading
        TIM_HandleTypeDef* htim_;
        // Encoder resolution for full wheel revolution
        int encoder_resolution_;
        // Previous time for velocity calculation
        ros::Time prev_update_time_;
        // Previous encoder tick count for velocity calculation
        long prev_encoder_ticks_;
    };
}

#endif // DIFFBOT_ENCODER_H
