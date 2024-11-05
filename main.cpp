#include "pico/stdlib.h"
#include "motor_control.h"
#include "pins.h"
#include "config.h"
#include "pid.h" 
#include "robot.h"

#include <cstdio>
#include <stdint.h>
#include <cstring>

#define UART_ID uart0

#define PREV_MASK 0x1
#define CURR_MASK 0x2
#define INVALID_MASK 0x3

#define BUFFER_SIZE 64  // 필요에 따라 버퍼 크기를 조정하세요

//encoder1_mask stands for M1_ENC_A/B_PIN
const uint32_t encoder1_mask = (0x01 << M1_ENC_A_PIN) | (0x01 << M1_ENC_B_PIN);
//encoder2_mask stands for M2_ENC_A/B_PIN
const uint32_t encoder2_mask = (0x01 << M2_ENC_A_PIN) | (0x01 << M2_ENC_B_PIN);

volatile int32_t encoder1_ticks = 0;
volatile uint32_t encoder1_state;

volatile int32_t encoder2_ticks = 0;
volatile uint32_t encoder2_state;


RobotPins robot_pins = 
{
    {M1_DIR_PIN, M1_PWM_PIN, false, L_MOTOR_MIN_SPEED, L_MOTOR_MAX_SPEED},  //left motor
    {M2_DIR_PIN, M2_PWM_PIN, true, R_MOTOR_MIN_SPEED, R_MOTOR_MAX_SPEED}    //right motor
};

// // Define the motors
// DCMotor left_motor(M1_ENA_PIN, M1_ENB_PIN, M1_PWM_PIN, L_MOTOR_MIN_SPEED, L_MOTOR_MAX_SPEED);
// DCMotor right_motor(M2_ENA_PIN, M2_ENB_PIN, M2_PWM_PIN, R_MOTOR_MIN_SPEED, R_MOTOR_MAX_SPEED);

// // Define PID controllers
// // PID left_pid(1.0, 0.01, 0.00, 0.5f, L_MOTOR_MAX_SPEED);  // Relative Slow
// // PID right_pid(1.085, 0.01, 0.00, 0.5f, R_MOTOR_MAX_SPEED); // More Fast
// PID left_pid(1.0, 0.01, 0.00, 0.5f, L_MOTOR_MIN_SPEED, L_MOTOR_MAX_SPEED);  // Relative Slow
// PID right_pid(1.0, 0.01, 0.00, 0.5f, L_MOTOR_MIN_SPEED, L_MOTOR_MAX_SPEED); // More Fast

// uart stuff
char in_buffer[100];
uint16_t char_idx = 0;

// 0.17, 0.0001, 0.0001,  // left motor PID constants
// 0.1712, 0.0001, 0.0001,  // right motor PID constants

// 0.1, 0.0, 0.0,  // left motor PID constants
// 0.082, 0.0, 0.0,  // right motor PID constants

const int sample_time_ms = 20;

#pragma pack(push, 1)  // 구조체 패딩을 제거하여 정확한 바이트 배열을 유지
struct Message {
    uint16_t ctx;             // 2바이트, 시작 표시자 (예: 0x1234)
    float l_speed;            // 4바이트, 왼쪽 바퀴 속도
    float r_speed;            // 4바이트, 오른쪽 바퀴 속도
    float x_pos;              // 4바이트, x 좌표
    float y_pos;              // 4바이트, y 좌표
    float theta;              // 4바이트, 방향 각도
    float v;                  // 4바이트, 직선 속도
    float w;                  // 4바이트, 각속도
    uint16_t crc;             // 2바이트, CRC-16-CCITT
    uint16_t ext;             // 2바이트, 종료 표시자 (예: 0x5678)
};
#pragma pack(pop)

#pragma pack(push, 1)
struct ControlMessage {
    uint16_t ctx;      // 2바이트, 시작 표시자 (예: 0x1234)
    float linear;      // 4바이트, 직선 속도 명령
    float angular;     // 4바이트, 각속도 명령
    uint16_t crc;      // 2바이트, CRC-16-CCITT
    uint16_t ext;      // 2바이트, 종료 표시자 (예: 0x5678)
};
#pragma pack(pop)

//Robot class 생성
Robot robot(
    0.109, 0.0014, 0.0001,  // left motor PID constants
    0.1, 0.0014, 0.0001,  // right motor PID constants
    sample_time_ms,
    LED_PIN,            // status LED pin
    robot_pins          // robot pins structure
);


enum RobotOperation {
    MOVE_FORWARD,
    ROTATE,
    STOP
};

RobotOperation robot_state = MOVE_FORWARD;
float target_distance = 0.40;
float target_angle = M_PI / 2; // 90 degrees in radians
float start_x = 0.0;
float start_y = 0.0;
float start_theta = 0.0;
const float slow_zone_ratio = 0.1; // 10%

float linear = 0;
float angular = 0;

absolute_time_t prev_time;
int32_t prev_encoder1_ticks = 0;
int32_t prev_encoder2_ticks = 0;
volatile bool timer_flag = false;


const int print_interval_ms = 500;  // ms based time
absolute_time_t last_print_time = get_absolute_time();

// Global variable for boot time
absolute_time_t boot_time;


// encoder interrupt callback
void gpio_callback(uint gpio, uint32_t events)
{

    // GPIO 11, Event 4 : 00
    // GPIO 10, Event 8 : 01
    // GPIO 11, Event 8 : 11
    // GPIO 10, Event 4 : 10

    // GPIO 12, Event 4 : 00
    // GPIO 13, Event 8 : 10
    // GPIO 12, Event 8 : 11
    // GPIO 13, Event 4 : 01


    int32_t change1 = 0;
    int32_t change2 = 0;
    uint32_t new_state1 = ((gpio_get_all() & encoder1_mask) >> (M1_ENC_B_PIN > M1_ENC_A_PIN ? M1_ENC_A_PIN : M1_ENC_B_PIN)) & 0x3;
    uint32_t new_state2 = ((gpio_get_all() & encoder2_mask) >> (M2_ENC_B_PIN > M2_ENC_A_PIN ? M2_ENC_A_PIN : M2_ENC_B_PIN)) & 0x3;
    // Debugging new_state and encoder state
    //printf("gpio : %d, events : %d, new_state1: %u, encoder1_state: %u, new_state2: %u, encoder2_state: %u\r\n", gpio, events, new_state1, encoder1_state, new_state2, encoder2_state);

    if(((new_state1 ^ encoder1_state) != INVALID_MASK) && (new_state1 != encoder1_state))
    {
        change1 = (encoder1_state & PREV_MASK) ^ ((new_state1 & CURR_MASK) >> 1);
        if(change1 == 0)
        {
            change1 = -1;
        }
        else
        {
            change1 = 1;
        }
        if(M1_ENC_INVERTED) change1 = -1 * change1;
        encoder1_ticks -= change1;
        //printf("GPIO: %d, Events: %d, Encoder1 Change: %d, New State: %u, Encoder1 Ticks: %d\r\n", gpio, events, change1, new_state1, encoder1_ticks);
    }

    if(((new_state2 ^ encoder2_state) != INVALID_MASK) && (new_state2 != encoder2_state))
    {
        change2 = (encoder2_state & PREV_MASK) ^ ((new_state2 & CURR_MASK) >> 1);
        if(change2 == 0)
        {
            change2 = -1;
        }
        else
        {
            change2 = 1;
        }
        if(M2_ENC_INVERTED) change2 = -1 * change2;
        encoder2_ticks -= change2;
        //printf("GPIO: %d, Events: %d, Encoder2 Change: %d, New State: %u, Encoder2 Ticks: %d\r\n", gpio, events, change2, new_state2, encoder2_ticks);
    }
    encoder1_state = new_state1;
    encoder2_state = new_state2;
    
    // absolute_time_t now = get_absolute_time();
    // if (absolute_time_diff_us(last_print_time, now) >= print_interval_ms * 1000) {
    //     last_print_time = now;
    //     printf("encoder1_ticks : %d, encoder2_ticks : %d\n\r", encoder1_ticks, encoder2_ticks); // 디버깅을 위해 출력
    // }

    //printf("encoder1_ticks : %d, encoder2_ticks : %d\n\r", encoder1_ticks, encoder2_ticks);
}

void setup() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize encoder pins
    gpio_init(M1_ENC_A_PIN);
    gpio_set_dir(M1_ENC_A_PIN, GPIO_IN);
    gpio_pull_up(M1_ENC_A_PIN);

    gpio_init(M1_ENC_B_PIN);
    gpio_set_dir(M1_ENC_B_PIN, GPIO_IN);
    gpio_pull_up(M1_ENC_B_PIN);

    gpio_init(M2_ENC_A_PIN);
    gpio_set_dir(M2_ENC_A_PIN, GPIO_IN);
    gpio_pull_up(M2_ENC_A_PIN);

    gpio_init(M2_ENC_B_PIN);
    gpio_set_dir(M2_ENC_B_PIN, GPIO_IN);
    gpio_pull_up(M2_ENC_B_PIN);

    gpio_set_irq_enabled_with_callback(M1_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(M1_ENC_B_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(M2_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(M2_ENC_B_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    encoder1_state = ((gpio_get_all() & encoder1_mask) >> (M1_ENC_A_PIN < M1_ENC_B_PIN ? M1_ENC_A_PIN : M1_ENC_B_PIN)) & 0x3;
    encoder2_state = ((gpio_get_all() & encoder2_mask) >> (M2_ENC_A_PIN < M2_ENC_B_PIN ? M2_ENC_A_PIN : M2_ENC_B_PIN)) & 0x3;


    boot_time = get_absolute_time();
    
}



void printState(float v, float w, RobotState state, RobotOdometry odometry)
{
    printf(
            // diff setpoint,wheel setpoint, speed
            "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d\n\r",
            state.l_ref_speed, state.r_ref_speed,
            state.l_speed, state.r_speed, 
            state.l_effort, state.r_effort,
            odometry.x_pos, odometry.y_pos, 
            odometry.theta, odometry.v, odometry.w,
            encoder1_ticks, encoder2_ticks
            );
}

// CRC 계산 함수 추가
uint16_t calculate_crc(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;  // CRC-16-CCITT
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// void sendRos(float v, float w, RobotState state, RobotOdometry odometry)
// {
//     char buffer[80];
//     int length = snprintf(buffer, sizeof(buffer),
//         "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
//         state.l_speed, state.r_speed, 
//         odometry.x_pos, odometry.y_pos, 
//         odometry.theta, odometry.v, odometry.w
//     );

//     // CRC 계산 및 추가
//     uint16_t crc = calculate_crc((const uint8_t *)buffer, length);
//     printf("%s,%04X", buffer, crc);  // 데이터 끝에 CRC 추가하여 전송
// }

void sendRos(const RobotState& state, const RobotOdometry& odometry) {
    Message msg;
    msg.ctx = 0x1234;  // 시작 표시자 설정

    // 데이터 필드 설정
    msg.l_speed = state.l_speed;
    msg.r_speed = state.r_speed;
    msg.x_pos = odometry.x_pos;
    msg.y_pos = odometry.y_pos;
    msg.theta = odometry.theta;
    msg.v = odometry.v;
    msg.w = odometry.w;

    // CRC 계산 (데이터 필드만 사용)
    uint8_t* dataPtr = reinterpret_cast<uint8_t*>(&msg.l_speed);
    size_t dataLength = sizeof(msg.l_speed) + sizeof(msg.r_speed) + sizeof(msg.x_pos) +
                        sizeof(msg.y_pos) + sizeof(msg.theta) + sizeof(msg.v) + sizeof(msg.w);

    msg.crc = calculate_crc(dataPtr, dataLength);

    msg.ext = 0x5678;  // 종료 표시자 설정

    // 바이트 배열로 전송
    uint8_t* msgPtr = reinterpret_cast<uint8_t*>(&msg);
    size_t msgSize = sizeof(Message);

    for (size_t i = 0; i < msgSize; i++) {
        putchar_raw(msgPtr[i]);  // putchar_raw를 사용하여 바이트 단위로 전송
    }
}

// bool receiveMessage(ControlMessage* msg) {
//     uint8_t buffer[BUFFER_SIZE];
//     size_t buffer_index = 0;
//     bool start_found = false;
//     absolute_time_t start_time = get_absolute_time();

//     while (true) {
//         // 타임아웃 처리 (예: 1초)
//         if (absolute_time_diff_us(start_time, get_absolute_time()) > 1000000) {
//             return false;  // 타임아웃 발생 시 실패로 반환
//         }

//         int ch = getchar_timeout_us(10000);  // 10ms 타임아웃
//         if (ch != PICO_ERROR_TIMEOUT) {
//             uint8_t byte = (uint8_t)ch;

//             // 버퍼가 가득 찼을 경우 초기화
//             if (buffer_index >= BUFFER_SIZE) {
//                 buffer_index = 0;
//                 start_found = false;
//             }

//             // 시작 표시자를 찾지 못한 상태에서 시작 표시자 확인
//             if (!start_found) {
//                 // 시작 표시자 확인 (0x1234)
//                 if (byte == 0x34) {
//                     buffer[buffer_index++] = byte;
//                     int next_ch = getchar_timeout_us(10000); // 10ms 타임아웃
//                     if (next_ch != PICO_ERROR_TIMEOUT && (uint8_t)next_ch == 0x12) {
//                         buffer[buffer_index++] = (uint8_t)next_ch;
//                         start_found = true;
//                     } else {
//                         buffer_index = 0;  // 시작 표시자 불일치 시 버퍼 초기화
//                     }
//                 }
//             } else {
//                 // 시작 표시자를 찾은 후 데이터 수신
//                 buffer[buffer_index++] = byte;

//                 // 최소 메시지 크기 이상 수신된 경우 종료 표시자 확인
//                 if (buffer_index >= sizeof(ControlMessage)) {
//                     // 종료 표시자 확인 (0x5678)
//                     if (buffer[buffer_index - 2] == 0x78 && buffer[buffer_index - 1] == 0x56) {
//                         // 전체 메시지를 수신했으므로 파싱 시작
//                         memcpy(msg, buffer, sizeof(ControlMessage));

//                         // CRC 검증
//                         uint8_t* dataPtr = reinterpret_cast<uint8_t*>(&msg->linear);
//                         size_t dataLength = sizeof(msg->linear) + sizeof(msg->angular);
//                         uint16_t calculated_crc = calculate_crc(dataPtr, dataLength);

//                         if (msg->crc == calculated_crc) {
//                             // 수신 성공
//                             return true;
//                         } else {
//                             // CRC 오류
//                             return false;
//                         }
//                     }
//                 }
//             }
//         }
//     }
// }

bool receiveMessage(ControlMessage* msg) {
    const size_t MESSAGE_SIZE = sizeof(ControlMessage);
    uint8_t buffer[MESSAGE_SIZE];
    size_t index = 0;
    absolute_time_t start_time = get_absolute_time();

    // 필요한 바이트 수만큼 데이터를 읽습니다.
    while (index < MESSAGE_SIZE) {
        int ch = getchar_timeout_us(10000);  // 타임아웃 설정 (예: 10ms)
        if (ch != PICO_ERROR_TIMEOUT) {
            buffer[index++] = (uint8_t)ch;
        } else {
            // 일정 시간 이상 데이터가 수신되지 않으면 실패로 간주
            if (absolute_time_diff_us(start_time, get_absolute_time()) > 500000) {  // 500ms
                return false;
            }
        }
    }

    // 구조체로 변환
    memcpy(msg, buffer, MESSAGE_SIZE);

    // 시작 및 종료 표시자 검증
    if (msg->ctx != 0x1234 || msg->ext != 0x5678) {
        return false;
    }

    // CRC 검증
    uint8_t* dataPtr = reinterpret_cast<uint8_t*>(&msg->linear);
    size_t dataLength = sizeof(msg->linear) + sizeof(msg->angular);
    uint16_t calculated_crc = calculate_crc(dataPtr, dataLength);

    if (msg->crc != calculated_crc) {
        return false;
    }

    return true;
}


bool timerCallback(repeating_timer_t *rt) {
    robot.setUnicycle(linear, angular);
    robot.updatePid(encoder1_ticks, encoder2_ticks);

    timer_flag = true;
    return true;
}

bool timerStatusCallback(repeating_timer_t *rt) {
    //printf("encoder1_ticks: %d, encoder2_ticks: %d\n\r", encoder1_ticks, encoder2_ticks);
    //printState(linear, angular, robot.getState(), robot.getOdometry());
    //sendRos(linear, angular, robot.getState(), robot.getOdometry());
    return true;
}

int main() {
    setup();

    ///////////////////////////////////////////////////////////////////////////////
    // When ROS2 let pico control with linear and angular as interoperative mode //
    ///////////////////////////////////////////////////////////////////////////////

    // 1. timer setup
    repeating_timer_t timer;
    if (!add_repeating_timer_ms(sample_time_ms, timerCallback, NULL, &timer)) {
        while (true) {
            gpio_put(LED_PIN, true);
            sleep_ms(50);
            gpio_put(LED_PIN, false);
            sleep_ms(1500);
        }
    }
    
    // repeating_timer_t status_timer;
    // if (!add_repeating_timer_ms(print_interval_ms, timerStatusCallback, NULL, &status_timer))
    // {
    //     while (true) {
    //         gpio_put(LED_PIN, true);
    //         sleep_ms(50);
    //         gpio_put(LED_PIN, false);
    //         sleep_ms(1500);
    //     }
    // }

    // 2. Received a message from uart serial
    int ch;
    int ch_idx = 0;  // ch_idx 초기화 추가
    int value1, value2;
    float value3;
    char* ch_ptr;
    char* ch_ptr2;
    char* ch_ptr3;

    ControlMessage control_msg;
    while (true)
    {
        // 명령 수신
        if (receiveMessage(&control_msg)) {
            gpio_put(LED_PIN, true);  // 유효한 데이터 수신 표시
            linear = control_msg.linear;
            angular = control_msg.angular;

            // 로봇 상태 업데이트
            robot.setUnicycle(linear, angular);

            // 로봇 상태 전송
            sendRos(robot.getState(), robot.getOdometry());

            gpio_put(LED_PIN, false);
        }
    }

   
}
