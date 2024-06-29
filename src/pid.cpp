#include "pid.h"

// PID 클래스 생성자 정의
PID::PID(float kp, float ki, float kd, float integral_limit = 0.5f, float output_limit = 1.0f) 
: _kp(kp), _ki(ki), _kd(kd), _prev_error(0), _integral(0), _integral_limit(integral_limit), _output_limit(output_limit) 
{
}

// Calculate output with PID compute.
float PID::calculate(float setpoint, float measured_value, float deltaT) {
    float error = setpoint - measured_value;
    
    // Integral calculation with windup guard
    _integral += error * deltaT;
    if (_integral > _integral_limit) _integral = _integral_limit;
    else if (_integral < -_integral_limit) _integral = -_integral_limit;
    
    // Derivative calculation
    float derivative = (error - _prev_error) / deltaT;
    _prev_error = error;
    
    // PID output
    float output = _kp * error + _ki * _integral + _kd * derivative;
    
    // Output clamping
    if (output > _output_limit) output = _output_limit;
    else if (output < -_output_limit) output = -_output_limit;
    
    return output;
}
