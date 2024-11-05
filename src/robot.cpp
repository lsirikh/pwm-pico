#include "robot.h"

Robot::Robot(
        float kp_l, float kd_l, float ki_l,
        float kp_r, float kd_r, float ki_r,
        uint32_t sample_time_ms,
        uint status_led_pin,
        RobotPins pins
        )
:
_kp_l(kp_l), _kd_l(kd_l), _ki_l(ki_l), _kp_r(kp_r), _kd_r(kd_r), _ki_r(ki_r), // PID 상수 초기화
  _sample_time_ms(sample_time_ms), // PID 상수 초기화
  _l_input(0.0f), _l_output(0.0f), _l_setpoint(0.0f), // 왼쪽 모터 PID 변수 초기화
  _r_input(0.0f), _r_output(0.0f), _r_setpoint(0.0f), // 오른쪽 모터 PID 변수 초기화
  _l_motor(pins.left.dir_pin, pins.left.pwm_pin, pins.left.is_reversed, pins.left.min_duty, pins.left.max_duty), // 왼쪽 모터 제어 객체 초기화
  _r_motor(pins.right.dir_pin, pins.right.pwm_pin, pins.right.is_reversed, pins.right.min_duty, pins.right.max_duty), // 오른쪽 모터 제어 객체 초기화
  _l_pid(kp_l, kd_l, ki_l, 0.5f, L_MOTOR_MIN_SPEED, L_MOTOR_MAX_SPEED), // 왼쪽 모터 PID 제어 객체 초기화
  _r_pid(kp_r, kd_r, ki_r, 0.5f, R_MOTOR_MIN_SPEED, R_MOTOR_MAX_SPEED), // 오른쪽 모터 PID 제어 객체 초기화
  //_l_pid(&_l_input, &_l_output, &_l_setpoint, kp_l, kd_l, ki_l, sample_time_ms), // 왼쪽 모터 PID 제어 객체 초기화
  //_r_pid(&_r_input, &_r_output, &_r_setpoint, kp_r, kd_r, ki_r, sample_time_ms), // 오른쪽 모터 PID 제어 객체 초기화
  _status_led_pin(status_led_pin) // 상태 LED 핀 초기화
{
    _l_motor.write(0.0f); // 왼쪽 모터 초기 속도 설정
    _r_motor.write(0.0f); // 오른쪽 모터 초기 속도 설정
    _l_setpoint = 0; // 왼쪽 모터 목표 속도 초기화
    _r_setpoint = 0; // 오른쪽 모터 목표 속도 초기화
    initPins(); // 핀 초기화 함수 호출
    last_print_time = get_absolute_time();

    // 출력 변화율 제한 설정 (필요에 따라 조정)
    //_l_pid.set_output_ramp_rate(2.0f); // 왼쪽 모터
    //_r_pid.set_output_ramp_rate(2.0f); // 오른쪽 모터
    //_pid_rate = float(sample_time_ms) / 1000.0f; // PID 샘플링 시간을 초 단위로 변환
}

void Robot::adjust_motor_speed_based_on_ticks(int32_t tick_base, int32_t left_ticks, int32_t right_ticks, float* left_speed, float* right_speed) {
    int32_t tick_diff = left_ticks - right_ticks;
    float adjustment = 0.0f;

    if (tick_diff > tick_base) {
        adjustment = tick_diff * 0.0001f; // Adjust this factor as needed
        *left_speed -= adjustment;
        *right_speed += adjustment;
    } else if (tick_diff < -tick_base) {
        adjustment = -tick_diff * 0.0001f; // Adjust this factor as needed
        *left_speed += adjustment;
        *right_speed -= adjustment;
    }
}

//로봇의 PID 제어를 업데이트하고, 오도메트리와 속도를 계산
void Robot::updatePid(uint32_t l_encoder_ticks, uint32_t r_encoder_ticks)
{
    // Calculate deltaT
    absolute_time_t current_time = get_absolute_time();
    float deltaT = absolute_time_diff_us(_prev_time, current_time) / 1e6;
    _prev_time = current_time;

    int32_t l_ticks = l_encoder_ticks; // 왼쪽 엔코더 틱 값 저장
    int32_t r_ticks = r_encoder_ticks; // 오른쪽 엔코더 틱 값 저장

    // 왼쪽 모터와 오른쪽 모터의 현재 위치 계산
    _state.l_position = (2.0 * M_PI) * l_ticks / ROBOT_MOTOR_PPR;
    _state.r_position = (2.0 * M_PI) * r_ticks / ROBOT_MOTOR_PPR;

    // 왼쪽 모터와 오른쪽 모터의 틱 변화량 계산
    int32_t dl_ticks = l_ticks - _state.l_ticks;
    int32_t dr_ticks = r_ticks - _state.r_ticks;

    // update odometry
    updateOdometry(dl_ticks, dr_ticks);

    // 왼쪽 모터와 오른쪽 모터의 목표 속도 설정
    _state.l_ref_speed = _l_setpoint;
    _state.r_ref_speed = _r_setpoint;
    
    // 왼쪽 모터와 오른쪽 모터의 현재 속도 계산
    _state.l_speed = (2.0 * M_PI) * dl_ticks / (ROBOT_MOTOR_PPR * deltaT);
    _state.r_speed = (2.0 * M_PI) * dr_ticks / (ROBOT_MOTOR_PPR * deltaT);


    //////////////////// 속도 필터 적용 (예: 1차 저역통과 필터) //////////////////
    // float l_speed = (2.0 * M_PI) * dl_ticks / (ROBOT_MOTOR_PPR * deltaT);
    // float r_speed = (2.0 * M_PI) * dr_ticks / (ROBOT_MOTOR_PPR * deltaT);

    // float alpha = 0.5f; // 필터 계수 (0 < alpha < 1)
    // _state.l_speed = alpha * l_speed + (1 - alpha) * _state.l_speed;
    // _state.r_speed = alpha * r_speed + (1 - alpha) * _state.r_speed;
    ///////////////////////////////////////////////////////////////////////

    // 로봇의 직선 속도와 각속도 계산
    _odom.v = (ROBOT_WHEEL_RADIUS / 2.0f) * (_state.l_speed + _state.r_speed);
    _odom.w = (ROBOT_WHEEL_RADIUS / ROBOT_WHEEL_SEPARATION) * (_state.r_speed - _state.l_speed);

    // PID 제어 입력 값 설정
    _l_input = _state.l_speed;
    _r_input = _state.r_speed;

    // PID 제어 출력 값 설정
    _state.l_effort = _l_pid.calculate(_state.l_ref_speed, _state.l_speed, deltaT);
    _state.r_effort = _r_pid.calculate(_state.r_ref_speed, _state.r_speed, deltaT);

    // // PID 제어 계산 수행
    // _l_pid.compute();
    // _r_pid.compute();
    // // PID 제어 출력 값 설정
    // _state.l_effort = _l_output;
    // _state.r_effort = _r_output;

    //printf("dl_ticks: %d, dr_ticks: %d, L_speed: %f, R_speed: %f, L_effort: %f, R_effort: %f\n\r", dl_ticks, dr_ticks, _state.l_speed, _state.r_speed, _state.l_effort, _state.r_effort);
    // Adjust right motor speed based on tick difference
    //adjust_motor_speed_based_on_ticks(50, l_encoder_ticks, r_encoder_ticks, &_state.l_effort, &_state.r_effort);

    _l_motor.write(_state.l_effort);
    _r_motor.write(_state.r_effort);

    // 엔코더 틱 값 업데이트
    _state.l_ticks = l_ticks;
    _state.r_ticks = r_ticks;
}

void Robot::updateOdometry(int32_t dl_ticks, int32_t dr_ticks) {
    // 왼쪽 바퀴와 오른쪽 바퀴의 이동 거리 계산
    float delta_l = (2 * M_PI * ROBOT_WHEEL_RADIUS * dl_ticks) / ROBOT_MOTOR_PPR;
    float delta_r = (2 * M_PI * ROBOT_WHEEL_RADIUS * dr_ticks) / ROBOT_MOTOR_PPR;
    
    // 중앙 이동 거리 계산
    float delta_center = (delta_l + delta_r) / 2;

    // x, y 좌표와 방향(theta) 업데이트
    _odom.x_pos += delta_center * cosf(_odom.theta);
    _odom.y_pos += delta_center * sinf(_odom.theta);
    _odom.theta += (delta_r - delta_l) / ROBOT_WHEEL_SEPARATION;
    
    // 로봇의 직선 속도와 각속도 설정
    _odom.v = _linear;
    _odom.w = _angular;
}

void Robot::setTargetSpeed(float left_speed, float right_speed)
{
    _l_setpoint = left_speed; // 왼쪽 바퀴 목표 속도 설정
    _r_setpoint = right_speed; // 오른쪽 바퀴 목표 속도 설정
}

void Robot::setUnicycle(float v, float w)
{
    // 속도 제한
    if(v > ROBOT_MAX_LINEAR_M_S) v = ROBOT_MAX_LINEAR_M_S; // 최대 직선 속도 제한
    if(v < ROBOT_MIN_LINEAR_M_S) v = ROBOT_MIN_LINEAR_M_S; // 최소 직선 속도 제한
    if(w > ROBOT_MAX_ANGULAR_R_S) w = ROBOT_MAX_ANGULAR_R_S; // 최대 각속도 제한
    if(w < ROBOT_MIN_ANGULAR_R_S) w = ROBOT_MIN_ANGULAR_R_S; // 최소 각속도 제한

    // 왼쪽 및 오른쪽 바퀴 속도 계산
    float v_l = (2 * v - w * ROBOT_WHEEL_SEPARATION) / (2 * ROBOT_WHEEL_RADIUS);
    float v_r = (2 * v + w * ROBOT_WHEEL_SEPARATION) / (2 * ROBOT_WHEEL_RADIUS);

    _linear = v; // 로봇의 직선 속도 설정
    _angular = w; // 로봇의 각속도 설정
    //printf("v: %f, w: %f, v_l: %f, v_r: %f\n\r", v, w, v_l, v_r); // 디버깅을 위해 출력
    setTargetSpeed(v_l, v_r); // 바퀴 속도 설정 함수 호출
}

void Robot::initPins()
{
    gpio_init(_status_led_pin);
    gpio_set_dir(_status_led_pin, GPIO_OUT);
}

RobotState Robot::getState() {
    return _state;
}

RobotOdometry Robot::getOdometry() {
    return _odom;
}

void Robot::clearPosition(){
    _odom.x_pos = 0;
    _odom.y_pos = 0;
}

// Update PID constants for left and right motors
void Robot::updatePID(float kp_l, float kd_l, float ki_l, float kp_r, float kd_r, float ki_r) {
    _l_pid.setPIDConstants(kp_l, ki_l, kd_l);  // Update left motor PID constant
    _r_pid.setPIDConstants(kp_r, ki_r, kd_r);  // Update right motor PID constant
}
