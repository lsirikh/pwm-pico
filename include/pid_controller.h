
#ifndef ROBOT_PWM_PID_CONTROLLER_H
#define ROBOT_PWM_PID_CONTROLLER_H
#include "pico/stdlib.h"
#include "cstdio"

class PID_Controller {
public:
    // 생성자: 입력, 출력, 목표 값 포인터와 PID 상수, 샘플링 시간을 인자로 받음
    PID_Controller(float* input, float* output, float* setpoint, float kp, float ki, float kd, uint32_t sample_time_ms);
    // 출력 변화율 제한 함수 추가
    void set_output_ramp_rate(float rate); 
    // PID 계산 함수
    void compute();
    // 출력 한계 설정 함수
    void set_output_limits(float min, float max);
    // PID 상수 설정 함수
    void set_gains(float kp, float ki, float kd);

private:
    uint32_t _sample_time_ms; // 샘플링 시간 (밀리초)
    float _kp; // 비례 상수
    float _kd; // 미분 상수
    float _ki; // 적분 상수

    float* _my_input; // 입력 값 포인터
    float* _my_output; // 출력 값 포인터
    float* _my_setpoint; // 목표 값 포인터
    float _out_min; // 출력 최소 값
    float _out_max; // 출력 최대 값
    float _out_sum; // 출력 합 (적분 항)
    float _err_sum; // 에러 합
    float _last_err; // 마지막 에러 값
    float _i_term; // 적분 항

    uint32_t _last_time; // 마지막 계산 시간
    float _last_input; // 마지막 입력 값
    float _last_output; // 이전 출력 값 저장 변수 추가
    
    float _output_ramp_rate; // 출력 변화율 제한 값 저장 변수 추가
};


#endif //ROBOT_PWM_PID_CONTROLLER_H
