
#include "l298_motor_driver.h"
#include "stm32f4xx_hal.h"
#include <ros.h>
#include <cstdio>

extern ros::NodeHandle nh;
int value;

diffbot::L298MotorController::L298MotorController(GPIO_TypeDef* gpioPort1, uint16_t in1, GPIO_TypeDef* gpioPort2, uint16_t in2, TIM_HandleTypeDef* htim, uint32_t channel) :
        in1Port(gpioPort1), in1Pin(in1), in2Port(gpioPort2), in2Pin(in2), pwmTimer(htim), pwmChannel(channel)
{
    // Initialize the motor control pins
    // HAL_TIM_PWM_Start(htim, channel); // Start PWM on the channel
}

void diffbot::L298MotorController::begin()
{
    // No specific initialization needed for L298
    // nh.loginfo("Initialize DiffBot Motor Controllers");

}

void diffbot::L298MotorController::setSpeed(int value)
{
    if (value > 0)
    {
        // Move forward
        HAL_GPIO_WritePin(in1Port, in1Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2Port, in2Pin, GPIO_PIN_RESET);
    }
    else if (value < 0)
    {
        // Move backward
        HAL_GPIO_WritePin(in1Port, in1Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2Port, in2Pin, GPIO_PIN_SET);
        value = -value; // Convert value to positive
    }
    else
    {
        // Stop the motor
        HAL_GPIO_WritePin(in1Port, in1Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2Port, in2Pin, GPIO_PIN_RESET);
    }
    // Set speed using PWM (value between 0-999)
    __HAL_TIM_SET_COMPARE(pwmTimer, TIM_CHANNEL_1, value);
    // pwmTimer->Instance->CCR1 = value;

    // char buffer[10]; // Make sure the buffer is large enough to hold the formatted string
    // sprintf(buffer, "val %d", value);
    // nh.logwarn(buffer);
}




// #include "stm32f4xx_hal.h"
// #include "l298_motor_driver.h"
// #include <ros.h>
// #include <cstdio>

// extern ros::NodeHandle nh;
// int targetSpeed, currentSpeed;
// // Class definition for the motor controller
// diffbot::L298MotorController::L298MotorController(GPIO_TypeDef* gpioPort1, uint16_t in1, GPIO_TypeDef* gpioPort2, uint16_t in2, TIM_HandleTypeDef* htim, uint32_t channel) :
//     in1Port(gpioPort1), in1Pin(in1), in2Port(gpioPort2), in2Pin(in2), pwmTimer(htim), pwmChannel(channel)
// {
//     // Initialize the motor control pins and start the PWM
//     // HAL_TIM_PWM_Start(htim, channel);  // Start PWM on the channel
//     // HAL_TIM_Base_Start_IT(htim);       // Start timer interrupt for dynamic control
// }
// void diffbot::L298MotorController::begin()
// {
//     // No specific initialization needed for L298
//     nh.loginfo("Initialize DiffBot Motor Controllers");

// }
// void diffbot::L298MotorController::HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim, int value)
// {

//     if (value > 0)
//     {
//         // Move forward
//         HAL_GPIO_WritePin(in1Port, in1Pin, GPIO_PIN_SET);
//         HAL_GPIO_WritePin(in2Port, in2Pin, GPIO_PIN_RESET);
//     }
//     else if (value < 0)
//     {
//         // Move backward
//         HAL_GPIO_WritePin(in1Port, in1Pin, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(in2Port, in2Pin, GPIO_PIN_SET);
//         targetSpeed = -value;  // Make targetSpeed positive
//     }
//     else
//     {
//         // Stop the motor
//         HAL_GPIO_WritePin(in1Port, in1Pin, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(in2Port, in2Pin, GPIO_PIN_RESET);
//     }
//     __HAL_TIM_SET_COMPARE(pwmTimer, TIM_CHANNEL_1, value);

//     char buffer[20];
//     sprintf(buffer, "Target speed %d", value);
//     nh.loginfo(buffer);
// }

