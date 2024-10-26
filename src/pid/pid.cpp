// #include "pid.h"

// namespace diffbot {

// PID::PID(float min_val, float max_val, float kp, float ki, float kd):
//     min_val_(min_val),
//     max_val_(max_val),
//     kp_(kp),
//     ki_(ki),
//     kd_(kd),
//     // integral_(0),        // Khởi tạo integral
//     // prev_error_(0)       // Khởi tạo prev_error
// {
// }

// // Định nghĩa hàm constrain
// inline double constrain(double value, double min_val, double max_val) {
//     if (value < min_val) return min_val;
//     else if (value > max_val) return max_val;   
//     else return value;
// }

// double PID::compute(float setpoint, float measured_value)
// {
//     double error;
//     double pid;

//     // setpoint is constrained between min and max to prevent pid from having too much error
//     error = setpoint - measured_value;
//     proportional_ = error;
//     integral_ += error;
//     derivative_ = error - prev_error_;

//     if(setpoint == 0 && error == 0)
//     {
//         integral_ = 0;
//     }

//     pid = (kp_ * proportional_) + (ki_ * integral_) + (kd_ * derivative_);
//     prev_error_ = error;

//     return constrain(pid, min_val_, max_val_);
// }

// void PID::updateConstants(float kp, float ki, float kd)
// {
//     kp_ = kp;
//     ki_ = ki;
//     kd_ = kd;
// }


// } // namespace diffbot


#include "pid.h"

diffbot::PID::PID(float min_val, float max_val, float kp, float ki, float kd):
    min_val_(min_val),
    max_val_(max_val),
    kp_(kp),
    ki_(ki),
    kd_(kd)
{
    // Constructor body left empty
}

// Định nghĩa hàm constrain
inline double constrain(double value, double min_val, double max_val) {
    if (value < min_val) return min_val;
    else if (value > max_val) return max_val;   
    else return value;
}

double diffbot::PID::compute(float setpoint, float measured_value)
{
    double error;
    double pid;

    // setpoint is constrained between min and max to prevent pid from having too much error
    error = setpoint - measured_value;
    proportional_ = error;
    integral_ += error;
    derivative_ = error - prev_error_;

    if(setpoint == 0 && error == 0)
    {
        integral_ = 0;
    }

    pid = (kp_ * proportional_) + (ki_ * integral_) + (kd_ * derivative_);
    prev_error_ = error;

    return constrain(pid, min_val_, max_val_);
}

void diffbot::PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

