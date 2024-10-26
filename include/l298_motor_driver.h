

#ifndef L298_MOTOR_CONTROLLER_H
#define L298_MOTOR_CONTROLLER_H

#include "stm32f4xx_hal.h" 
#include "motor_controller_interface.h"
#include "tim.h"

namespace diffbot {

    /** \brief Implementation of the MotorControllerIntf for the L298 Motor Driver
     * 
     * This class implements the setSpeed method from MotorControllerIntf
     * to control a motor via the L298 motor driver.
     */
    class L298MotorController : public MotorControllerIntf<void>
    {
        public:
            /** \brief Constructor for L298MotorController
             * 
             * \param in1 First control pin for the motor's direction.
             * \param in2 Second control pin for the motor's direction.
             * \param en Enable pin for PWM speed control.
             */
            L298MotorController(GPIO_TypeDef* gpioPort1, uint16_t in1, GPIO_TypeDef* gpioPort2, uint16_t in2, TIM_HandleTypeDef* htim, uint32_t channel);

            /** \brief Initializes the motor controller
             * 
             * This should be called in the setup function. 
             * No additional configuration is needed for L298.
             */
            void begin();

            /** \brief Set the speed of the motor
             * 
             * Implements the setSpeed method to control the motor via L298.
             * The value ranges from -255 to 255. Positive values move forward,
             * negative values move backward, and zero stops the motor.
             * 
             * \param value Motor speed (-255 to 255).
             */
            void setSpeed(int value) override;
            // void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim, int value);

        private:
            GPIO_TypeDef* in1Port; // Port for in1
            uint16_t in1Pin; // Pin for in1
            GPIO_TypeDef* in2Port; // Port for in2
            uint16_t in2Pin; // Pin for in2
            TIM_HandleTypeDef* pwmTimer; // Timer for PWM
            uint32_t pwmChannel; // PWM channel
    };

}

#endif // L298_MOTOR_CONTROLLER_H
