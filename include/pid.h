#ifndef ROBOT_PWM_PID_H
#define ROBOT_PWM_PID_H

class PID {
public:
    PID(float kp, float ki, float kd, float integral_limit = 0.5f, float output_limit = 1.0f) 
        : _kp(kp), _ki(ki), _kd(kd), _prev_error(0), _integral(0), _integral_limit(integral_limit), _output_limit(output_limit) {}

    float calculate(float setpoint, float measured_value, float deltaT) {
        // // error
        // float error = setpoint - measured_value;
        // // integral
        // _integral += error * deltaT;
        // // derivative
        // float derivative = (error - _prev_error) / deltaT;
        // _prev_error = error;
        // // control signal
        // return _kp * error + _ki * _integral + _kd * derivative;
        // Error calculation
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

    float get_rpm() const { return _rpm; }
    float get_speed() const { return _speed; }
    float get_target() const { return _target; }
    void set_rpm(float rpm) { _rpm = rpm; }
    void set_speed(float speed) { _speed = speed; }
    void set_target(float target) {_target = target;}

private:
    float _kp, _ki, _kd;
    float _prev_error;
    float _integral;
    float _integral_limit;
    float _output_limit;

    float _rpm;
    float _speed;
    float _target;
};

#endif // ROBOT_PWM_PID_H
