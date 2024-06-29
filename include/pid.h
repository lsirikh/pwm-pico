#ifndef ROBOT_PWM_PID_H
#define ROBOT_PWM_PID_H

class PID {
public:
    PID(float kp, float ki, float kd, float integral_limit, float output_limit);
    //: _kp(kp), _ki(ki), _kd(kd), _prev_error(0), _integral(0), _integral_limit(integral_limit), _output_limit(output_limit) {}

    float calculate(float setpoint, float measured_value, float deltaT);

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
