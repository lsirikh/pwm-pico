#include "pico/stdlib.h"
#include "motor_control.h"
#include "pins.h"
#include "config.h"
#include "pid.h" 
#include "robot.h"

#include <cstdio>

#define UART_ID uart0

#define PREV_MASK 0x1
#define CURR_MASK 0x2
#define INVALID_MASK 0x3

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
    {M1_DIR_PIN, M1_PWM_PIN, M1_ENC_INVERTED, L_MOTOR_MIN_SPEED, L_MOTOR_MAX_SPEED},
    {M2_DIR_PIN, M2_PWM_PIN, M2_ENC_INVERTED, R_MOTOR_MIN_SPEED, R_MOTOR_MAX_SPEED}
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

//Robot class 생성
Robot robot(
    0.17, 0.01, 0.0001,  // left motor PID constants
    0.1712, 0.01, 0.0001,  // right motor PID constants
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

const int sample_time_ms = 20;
const int print_interval_ms = 500;  // ms based time
absolute_time_t last_print_time = get_absolute_time();

// Global variable for boot time
absolute_time_t boot_time;

// // Low-pass filter variables
// float v1Filt = 0;
// float v1Prev = 0;
// float v2Filt = 0;
// float v2Prev = 0;

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
        // printf("GPIO: %d, Events: %d, Encoder1 Change: %d, New State: %u, Encoder1 Ticks: %d\r\n", gpio, events, change1, new_state1, encoder1_ticks);
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
        // printf("GPIO: %d, Events: %d, Encoder2 Change: %d, New State: %u, Encoder2 Ticks: %d\r\n", gpio, events, change2, new_state2, encoder2_ticks);
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

//print_relative_time("Status Update", duty_cycle, left_pid.get_rpm(), right_pid.get_rpm(), left_pid.get_speed(), right_pid.get_speed(), encoder1_ticks, encoder2_ticks);
// Function to print the current relative time in HH:mm:ss.ff format
void print_relative_time(const char* message, float duty_cycle, float target_left, float target_right, float rpm_left, float rpm_right, float speed_left, float speed_right, int32_t ticks_left, int32_t ticks_right) {
    absolute_time_t now = get_absolute_time();
    int64_t us_since_boot = absolute_time_diff_us(boot_time, now);
    
    int hours = us_since_boot / 1000000 / 3600;
    int minutes = (us_since_boot / 1000000 / 60) % 60;
    int seconds = (us_since_boot / 1000000) % 60;
    int hundredths = (us_since_boot / 10000) % 100;

    printf("[%02d:%02d:%02d.%02d] %s PWM: %.2f, TARGET_L: %.2f, TARGET_R: %.2f, RPM_L: %.2f, RPM_R: %.2f, Speed_L: %.2f m/s, Speed_R: %.2f m/s, Ticks_L: %d, Ticks_R: %d, Tick_diff: %d\r\n", 
           hours, minutes, seconds, hundredths, message,
           duty_cycle,
           target_left, target_right,
           rpm_left, rpm_right, 
           speed_left, speed_right, 
           ticks_left, ticks_right, ticks_right-ticks_left);
}

void print_relative_time(const char* message, float duty_cycle, float target, float rpm, float speed, int32_t ticks) {
    absolute_time_t now = get_absolute_time();
    int64_t us_since_boot = absolute_time_diff_us(boot_time, now);
    
    int hours = us_since_boot / 1000000 / 3600;
    int minutes = (us_since_boot / 1000000 / 60) % 60;
    int seconds = (us_since_boot / 1000000) % 60;
    int hundredths = (us_since_boot / 10000) % 100;

    printf("[%02d:%02d:%02d.%02d] %s PWM: %.2f, TARGET: %.2f, RPM: %.2f, Speed: %.2f m/s, Ticks: %d\r\n", 
           hours, minutes, seconds, hundredths, message,
           duty_cycle,
           target,
           rpm, 
           speed, 
           ticks);
}

void printState(float v, float w, RobotState state, RobotOdometry odometry, int32_t ticks_left, int32_t ticks_right)
{
    absolute_time_t now = get_absolute_time();
    int64_t us_since_boot = absolute_time_diff_us(boot_time, now);
    
    int hours = us_since_boot / 1000000 / 3600;
    int minutes = (us_since_boot / 1000000 / 60) % 60;
    int seconds = (us_since_boot / 1000000) % 60;
    int hundredths = (us_since_boot / 10000) % 100;

    printf(
            // diff setpoint,wheel setpoint, speed
            //"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n\r",
            "[%02d:%02d:%02d.%02d] V: %f, W: %f, Req_S_L: %.3f, Req_S_R: %.3f, Curr_S_L: %.3f, Curr_S_R: %.3f, Eff_S_L: %.3f, Eff_S_R: %.3f, X: %.3f, Y: %.3f, θ: %.3f, V: %.3f, W: %.3f, Ticks_L: %d, Ticks_R: %d, Tick_diff: %d\n\r",
            hours, minutes, seconds, hundredths,
            v,w,
            state.l_ref_speed, state.r_ref_speed,
            state.l_speed, state.r_speed, 
            state.l_effort, state.r_effort,
            odometry.x_pos, odometry.y_pos, 
            odometry.theta, odometry.v, odometry.w,
            ticks_left, ticks_right, ticks_right-ticks_left
            );
}

void printState(float v, float w, RobotState state, RobotOdometry odometry)
{
    printf(
            // diff setpoint,wheel setpoint, speed
            "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n\r",
            state.l_ref_speed, state.r_ref_speed,
            state.l_speed, state.r_speed, 
            state.l_effort, state.r_effort,
            odometry.x_pos, odometry.y_pos, 
            odometry.theta, odometry.v, odometry.w
            );
}


bool timerCallback(repeating_timer_t *rt) {
    robot.setUnicycle(linear, angular);
    robot.updatePid(encoder1_ticks, encoder2_ticks);

    timer_flag = true;
    return true;
}

bool timerStatusCallback(repeating_timer_t *rt) {
    printf("encoder1_ticks: %d, encoder2_ticks: %d\n\r", encoder1_ticks, encoder2_ticks);
    return true;
}

void controlLoop() {

    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(last_print_time, now) >= print_interval_ms * 1000) {
        last_print_time = now;
        switch (robot_state) {
        case MOVE_FORWARD:
            printf("[MOVE FORWARD]\t");
            break;
        case ROTATE:
            printf("[ROTATE 90 Degree]\t");
            break;
        case STOP:
            printf("[STOP]\t");
            break;
    }
        printState(linear, angular, robot.getState(), robot.getOdometry(),encoder1_ticks, encoder2_ticks);
    }

    RobotOdometry odometry = robot.getOdometry();
    float remaining_distance = sqrt(pow(odometry.x_pos - start_x, 2) + pow(odometry.y_pos - start_y, 2));
    float remaining_angle = fabs(odometry.theta - start_theta);

    switch (robot_state) {
        case MOVE_FORWARD:
            if (
                //Slow move After Target Distance remain 10% off
                (target_distance - target_distance * slow_zone_ratio) < remaining_distance && remaining_distance <= target_distance 
                //Slow move Before remain Distance up to target distance multiply slow_zone ratio 
                || remaining_distance <= target_distance * slow_zone_ratio) {
                linear = 0.05; // Reduce speed to half (0.1 m/s / 2)
            } else {
                linear = 0.1; // 0.1 m/s
            }
            angular = 0.0; // no rotation

            if (remaining_distance >= target_distance) {
                linear = 0.0; // stop moving forward
                robot_state = ROTATE;
                start_x = odometry.x_pos;
                start_y = odometry.y_pos;
            }
            break;

        case ROTATE:
            linear = 0.0; // no forward movement
            if (
                //Slow move After Target angle remain 10% off
                (target_angle - target_angle * slow_zone_ratio) < remaining_angle && remaining_angle<= target_angle 
                //Slow move Before remain angle up to target angle multiply slow_zone ratio 
                || remaining_angle <= target_angle * slow_zone_ratio) {
                angular = 0.5; // Reduce speed to half (1.0 rad/s / 2)
            } else {
                angular = 1.0; // start rotating
            }
            if (remaining_angle >= target_angle) {
                angular = 0.0; // stop rotating
                start_theta = odometry.theta;
                robot_state = MOVE_FORWARD;
            }
            break;

        case STOP:
            linear = 0.0;
            angular = 0.0;
            break;
    }
}

int main() {
    setup();

    repeating_timer_t timer;
    if (!add_repeating_timer_ms(sample_time_ms, timerCallback, NULL, &timer)) {
        while (true) {
            gpio_put(LED_PIN, true);
            sleep_ms(50);
            gpio_put(LED_PIN, false);
            sleep_ms(1500);
        }
    }

    // 상태 확인 및 디버깅 타이머 추가
    // repeating_timer_t status_timer;
    // if (!add_repeating_timer_ms(3000, timerStatusCallback, NULL, &status_timer))
    // {
    //     while (true) {
    //         gpio_put(LED_PIN, true);
    //         sleep_ms(50);
    //         gpio_put(LED_PIN, false);
    //         sleep_ms(1500);
    //     }
    // }
        
    
    // while (true) {
    //     if (timer_flag) {
    //         timer_flag = false;
    //         controlLoop();
    //         // Print the status every 1 second
    //         // absolute_time_t now = get_absolute_time();
    //         // if (absolute_time_diff_us(last_print_time, now) >= print_interval_ms * 1000) {
    //         //     last_print_time = now;
    //         //     printState(linear, angular, robot.getState(), robot.getOdometry(),encoder1_ticks, encoder2_ticks);
    //         // }
    //     }
    // }

    int ch;
    int ch_idx = 0;  // ch_idx 초기화 추가
    int value1, value2;
    float value3;
    char* ch_ptr;
    char* ch_ptr2;
    char* ch_ptr3;

    while (true)
    {
        ch = getchar_timeout_us(0);
        while(ch != PICO_ERROR_TIMEOUT)
        {
            gpio_put(LED_PIN, true);
            // printf(" %c ", ch);
            putchar(ch);
            in_buffer[ch_idx++] = ch;
            if(ch == '/')
            {
                in_buffer[ch_idx] = 0;      // end of string
                linear = strtof(in_buffer, &ch_ptr);
                angular = strtof(ch_ptr+1, &ch_ptr2);
                printState(linear, angular, robot.getState(), robot.getOdometry());
                ch_idx = 0;
                break;
            }

            ch = getchar_timeout_us(0);
        }
        gpio_put(LED_PIN, false);
    }

   
}
