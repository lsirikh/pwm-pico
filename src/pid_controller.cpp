#include "pid_controller.h"

// PID 클래스 생성자 정의
PID_Controller::PID_Controller(float *input, float *output, float *setpoint, float kp, float ki, float kd, uint32_t sample_time_ms)
: _my_input(input), _my_output(output), _my_setpoint(setpoint), _sample_time_ms(sample_time_ms), _out_sum(0), _last_input(0), _last_output(0.0f), _output_ramp_rate(0.0f)
{
    set_output_limits(-1.0f, 1.0f); // 출력 한계를 -1.0에서 1.0으로 설정
    set_gains(kp, ki, kd); // PID 상수를 설정
}

// 출력 변화율 제한 함수 구현
void PID_Controller::set_output_ramp_rate(float rate)
{
    _output_ramp_rate = rate; // 출력 변화율 제한 값 설정
}

// PID 상수를 설정하는 함수 정의
void PID_Controller::set_gains(float kp, float ki, float kd)
{
    if(kp < 0 || ki < 0 || kd < 0) return; // PID 상수가 음수인 경우 설정하지 않음

    float sample_time_s = (float)_sample_time_ms / 1000.0f; // 샘플링 시간을 초 단위로 변환
    printf("pid sample time: %f [s]", sample_time_s); // 샘플링 시간을 출력

    _kp = kp; // 비례 상수 설정
    _ki = ki * sample_time_s; // 적분 상수를 샘플링 시간에 맞게 설정
    _kd = kd / sample_time_s; // 미분 상수를 샘플링 시간에 맞게 설정
}

// 출력 한계를 설정하는 함수 정의
void PID_Controller::set_output_limits(float min, float max)
{
    if(min >= max) return; // 최소 값이 최대 값보다 크거나 같은 경우 설정하지 않음
    _out_min = min; // 출력 최소 값 설정
    _out_max = max; // 출력 최대 값 설정

    if(*_my_output > _out_max) *_my_output = _out_max; // 출력 값이 최대 값을 초과하면 최대 값으로 설정
    else if(*_my_output < _out_min) *_my_output = _out_min; // 출력 값이 최소 값보다 작으면 최소 값으로 설정

    if(_out_sum > _out_max) _out_sum = _out_max; // 출력 합이 최대 값을 초과하면 최대 값으로 설정
    else if(_out_sum < _out_min) _out_sum = _out_min; // 출력 합이 최소 값보다 작으면 최소 값으로 설정
}

// PID 제어를 계산하는 함수 정의
void PID_Controller::compute()
{
    float input = *_my_input; // 현재 입력 값을 저장
    float error = *_my_setpoint - input; // 목표 값과 입력 값의 차이(오차) 계산

    // float delta_input = input - _last_input; // 입력 값의 변화량 계산
    // _out_sum += _ki * error; // 적분 항 업데이트

    // if(_out_sum > _out_max) _out_sum = _out_max; // 적분 항이 최대 값을 초과하면 최대 값으로 설정
    // else if(_out_sum < _out_min) _out_sum = _out_min; // 적분 항이 최소 값보다 작으면 최소 값으로 설정

    // float output = _kp * error + _out_sum - _kd * delta_input; // PID 출력 계산

    // if(output > _out_max) output = _out_max; // 출력 값이 최대 값을 초과하면 최대 값으로 설정
    // else if(output < _out_min) output = _out_min; // 출력 값이 최소 값보다 작으면 최소 값으로 설정

    // 비례 항 계산
    float p_term = _kp * error;

    // 적분 항 업데이트
    _out_sum += _ki * error;
    // 적분 항 제한
    if (_out_sum > _out_max) _out_sum = _out_max;
    else if (_out_sum < _out_min) _out_sum = _out_min;

    // 미분 항 계산
    float delta_input = input - _last_input;
    float d_term = -_kd * delta_input;

    // 출력 값 계산
    float output = p_term + _out_sum + d_term;


    // 출력 변화율 제한 적용
    if (_output_ramp_rate > 0.0f)
    {
        float max_delta = _output_ramp_rate * (_sample_time_ms / 1000.0f);
        float delta_output = output - _last_output;

        if (delta_output > max_delta)
            output = _last_output + max_delta;
        else if (delta_output < -max_delta)
            output = _last_output - max_delta;
    }

    *_my_output = output; // 출력 값을 설정
    _last_input = input; // 마지막 입력 값을 현재 입력 값으로 업데이트
    _last_output = output; // 마지막 출력 값을 현재 출력 값으로 업데이트

}


