#ifndef ROBOT_PWM_ROBOT_H
#define ROBOT_PWM_ROBOT_H

#include "pico/stdlib.h"
#include <cstdio>
#include "config.h"
#include "pins.h"
#include "pid.h"
#include "motor_control.h"
//#include "pid_controller.h"


//모터 핀 정의 정보 구조체
struct MotorPins
{
    uint dir_pin;
    uint pwm_pin;
    bool is_reversed;
    float min_duty;
    float max_duty;
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
            float kp_l, float kd_l, float ki_l,
            float kp_r, float kd_r, float ki_r,
            uint32_t sample_time_ms,
            uint status_led_pin,
            RobotPins pins
            );
    void start();
    void setTargetSpeed(float left_speed, float right_speed);
    void setUnicycle(float v, float w);
    RobotState getState();
    RobotOdometry getOdometry();
    void clearPosition();
    void setPidTunings(float kp, float kd, float ki);
    void updatePid(uint32_t l_encoder_ticks, uint32_t r_encoder_ticks);
    float calculate_rpm(int32_t ticks, float deltaT);
    float calculate_speed(float rpm);
    void adjust_motor_speed_based_on_ticks(int32_t tick_base, int32_t left_ticks, int32_t right_ticks, float* left_speed, float* right_speed);

    // Update PID constants
    void updatePID(float kp_l, float kd_l, float ki_l, float kp_r, float kd_r, float ki_r);

private:
    float _kp_l;
    float _kd_l;
    float _ki_l;
    float _kp_r;
    float _kd_r;
    float _ki_r;
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
    int print_interval_ms = 500; // 0.5초마다 출력
    absolute_time_t last_print_time;

    // Robot 클래스 내 멤버 변수로 이전 값 저장 변수 추가
    float _previous_linear = 0.0f;
    float _previous_angular = 0.0f;

    DCMotor _l_motor;
    DCMotor _r_motor;
    PID _l_pid;
    PID _r_pid;
    // PID_Controller _l_pid;
    // PID_Controller _r_pid;
    uint _status_led_pin;
    RobotState _state;
    RobotOdometry _odom;

    void controlLoop();
    void updateOdometry(int32_t dl_ticks, int32_t dr_ticks);
    void initPins();

};

#endif //ROBOT_PWM_ROBOT_H
