#ifndef ROBOT_PWM_ROBOT_H
#define ROBOT_PWM_ROBOT_H

#include "pico/stdlib.h"
#include <cstdio>
#include "config.h"
#include "pins.h"
#include "pid.h"
#include "motor_control.h"


//모터 핀 정의 정보 구조체
struct MotorPins
{
    uint pwm;
    uint en_a;
    uint en_b;
};

//로봇핀 구조체
//하위 구조체 MotorPins
struct RobotPins
{
    MotorPins left;
    MotorPins right;
};

//로봇 상태 정보 구조체
struct RobotState
{
    int32_t l_ticks;//left encoder ticks
    int32_t r_ticks;//right encoder ticks
    float l_position;//left motor position
    float r_position;//right motor position
    float l_speed;//left motor speed
    float r_speed;//right motor speed
    float l_effort;//
    float r_effort;//
    float l_ref_speed;//left motor requested speed?
    float r_ref_speed;//right motor requested speed?
};
//로봇 Odometry
//각각 변수가 의미하는 바를 확인
struct RobotOdometry
{
    float x_pos;
    float y_pos;
    float theta;
    float v;
    float w;
};


class Robot{
public:
    Robot(
            float kp,
            float kd,
            float ki,
            uint status_led_pin,
            RobotPins pins
            );
    void start();
    void setTargetSpeed(float left_speed, float right_speed);
    void setUnicycle(float v, float w);
    RobotState getState();
    RobotOdometry getOdometry();
    void setPidTunings(float kp, float kd, float ki);
    void updatePid(uint32_t l_encoder_ticks, uint32_t r_encoder_ticks);
    float calculate_rpm(int32_t ticks, float deltaT);
    float calculate_speed(float rpm);
    void adjust_motor_speed_based_on_ticks(int32_t tick_base, int32_t left_ticks, int32_t right_ticks, float* left_speed, float* right_speed);

private:
    float _kp;
    float _kd;
    float _ki;
    float _pid_rate;
    uint32_t _sample_time_ms;
    float _l_input;
    float _l_output;
    float _l_setpoint;
    float _r_input;
    float _r_output;
    float _r_setpoint;
    float _linear;
    float _angular;
    absolute_time_t _prev_time;

    DCMotor _l_motor;
    DCMotor _r_motor;
    PID _l_pid;
    PID _r_pid;
    uint _status_led_pin;
    RobotState _state;
    RobotOdometry _odom;

    void controlLoop();
    void updateOdometry(int32_t dl_ticks, int32_t dr_ticks);
    void initPins();
};

#endif //ROBOT_PWM_ROBOT_H
