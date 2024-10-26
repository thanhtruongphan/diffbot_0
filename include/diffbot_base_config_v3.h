// // / Encoder pins
// #define ENCODER_LEFT_H1 GPIO_PIN_12
// #define ENCODER_LEFT_H2 GPIO_PIN_13
// // Encoder resolution used for initialization 
// // will be read from parameter server
// #define ENCODER_RESOLUTION 1980

// #define ENCODER_RIGHT_H1 GPIO_PIN_14
// #define ENCODER_RIGHT_H2 GPIO_PIN_15

// /// Motor i2c address
// // #define MOTOR_DRIVER_ADDR 0x60
// // #define MOTOR_LEFT 4
// // #define MOTOR_RIGHT 3

// #define K_P 0.6 // P constant
// #define K_I 0.0 // I constant
// #define K_D 0.0 // D constant

// // #define PWM_BITS 8  // PWM Resolution of the microcontroller

// #define UPDATE_RATE_CONTROL 20
// #define UPDATE_RATE_IMU 1
// #define UPDATE_RATE_DEBUG 5

// #define E_STOP_COMMAND_RECEIVED_DURATION 5 // Stop motors if no command was received after this amount of seconds

// // #define PWM_MAX pow(2, PWM_BITS) - 1
// #define PWM_MAX 1000
// #define PWM_MIN -(PWM_MAX)



// // Encoder pins for STM32
// #define ENCODER_LEFT_H1 GPIO_PIN_12  // Left encoder, pin 1
// #define ENCODER_LEFT_H2 GPIO_PIN_13  // Left encoder, pin 2

// #define ENCODER_RIGHT_H1 GPIO_PIN_14  // Right encoder, pin 1
// #define ENCODER_RIGHT_H2 GPIO_PIN_15  // Right encoder, pin 2

// Encoder resolution










#define ENCODER_RESOLUTION 1980      //1980   Modify this if the actual resolution differs

// PID constants for motor control
#define K_P 5.0 // P constant
#define K_I 0.0 // I constant
#define K_D 0.0 // D constant

// Motor driver PWM range and limits for STM32
#define PWM_MAX 999                 // Adjusted for a 1 kHz frequency PWM (Timer-based control on STM32)
#define PWM_MIN -(PWM_MAX)

// Update rates (control loop frequency in Hz)
#define UPDATE_RATE_CONTROL 200     // Control update rate (e.g., PID) :20
#define UPDATE_RATE_IMU 3           // IMU update rate (if used) :1
#define UPDATE_RATE_DEBUG 100       // Debug output rate :5

// Emergency stop command timeout
#define E_STOP_COMMAND_RECEIVED_DURATION 5  // Stop motors if no command received within 5 seconds
