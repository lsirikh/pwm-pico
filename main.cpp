#include "pico/stdlib.h"
#include "motor_control.h"
#include "pins.h"
#include "config.h"
#include "pid.h" 
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

// Define the motors
DCMotor left_motor(M1_ENA_PIN, M1_ENB_PIN, M1_PWM_PIN, L_MOTOR_MIN_SPEED, L_MOTOR_MAX_SPEED);
DCMotor right_motor(M2_ENA_PIN, M2_ENB_PIN, M2_PWM_PIN, R_MOTOR_MIN_SPEED, R_MOTOR_MAX_SPEED);

// Define PID controllers
PID left_pid(1.0, 0.01, 0.00, 0.5f, L_MOTOR_MAX_SPEED);  // Relative Slow
PID right_pid(1.085, 0.01, 0.00, 0.5f, R_MOTOR_MAX_SPEED); // More Fast

absolute_time_t prev_time;
int32_t prev_encoder1_ticks = 0;
int32_t prev_encoder2_ticks = 0;
volatile bool timer_flag = false;

const int sample_time_ms = 20;
const int print_interval_ms = 200;  // ms based time
float duty_cycle = 0.8f;
int step = 0;

// Global variable for boot time
absolute_time_t boot_time;

// Low-pass filter variables
float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

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

    if((new_state1 != encoder1_state) 
    && ((new_state1 ^ encoder1_state) != INVALID_MASK)
    )
    {

        change1 = (encoder1_state & PREV_MASK) ^ ((new_state1 & CURR_MASK) >> 1);
        if(change1 == 0)
        {
            change1 = 1;
        }else{
            change1 = -1;
        }
        if(M1_ENC_INVERTED) change1 = -change1;
        encoder1_ticks += change1;
        //printf("xor: %u, change1 : %d\r\n", (new_state1 ^ encoder1_state), change1);
    }

    if((new_state2 != encoder2_state) 
    && ((new_state2 ^ encoder2_state) != INVALID_MASK)
    )
    {
        change2 = (encoder2_state & PREV_MASK) ^ ((new_state2 & CURR_MASK) >> 1);
        if(change2 == 0)
        {
            change2 = 1;
        }else{
            change2 = -1;
        }
        if(M2_ENC_INVERTED) change2 = -change2;
        encoder2_ticks += change2;
        //printf("xor: %u, change2 : %d\r\n", (new_state2 ^ encoder2_state), change2);

    }
    encoder1_state = new_state1;
    encoder2_state = new_state2;
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

// Function to calculate RPM from encoder ticks
float calculate_rpm(int32_t ticks, float deltaT) {
    return (ticks / ROBOT_MOTOR_PPR) * (60.0f / deltaT);
}

// Function to calculate linear speed (m/s) from RPM
float calculate_speed(float rpm) {
    float wheel_circumference = 2 * M_PI * ROBOT_WHEEL_RADIUS;
    return (rpm / 60.0f) * wheel_circumference;
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

void adjust_motor_speed_based_on_ticks(int32_t tick_base, int32_t left_ticks, int32_t right_ticks, float* left_speed, float* right_speed) {
    int32_t tick_diff = left_ticks - right_ticks;
    float adjustment = 0.0f;

    if (tick_diff > tick_base) {
        adjustment = tick_diff * 0.001f; // Adjust this factor as needed
        *left_speed -= adjustment;
        *right_speed += adjustment;
    } else if (tick_diff < -tick_base) {
        adjustment = -tick_diff * 0.001f; // Adjust this factor as needed
        *left_speed += adjustment;
        *right_speed -= adjustment;
    }
}


bool timerCallback(repeating_timer_t *rt) {
    // Calculate deltaT
    absolute_time_t current_time = get_absolute_time();
    float deltaT = absolute_time_diff_us(prev_time, current_time) / 1e6;
    prev_time = current_time;

    // Calculate RPM and speed
    int32_t current_encoder1_ticks = encoder1_ticks;
    int32_t current_encoder2_ticks = encoder2_ticks;

    float rpm_left = calculate_rpm(current_encoder1_ticks - prev_encoder1_ticks, deltaT);
    float rpm_right = calculate_rpm(current_encoder2_ticks - prev_encoder2_ticks, deltaT);

    float speed_left = calculate_speed(rpm_left);
    float speed_right = calculate_speed(rpm_right);

    // // Low-pass filter (25 Hz cutoff)
    // v1Filt = 0.854 * v1Filt + 0.0728 * rpm_left + 0.0728 * v1Prev;
    // v1Prev = rpm_left;
    // v2Filt = 0.854 * v2Filt + 0.0728 * rpm_right + 0.0728 * v2Prev;
    // v2Prev = rpm_right;

    // // Low-pass filter (50 Hz cutoff)
    // v1Filt = 0.933 * v1Filt + 0.0335 * rpm_left + 0.0335 * v1Prev;
    // v1Prev = rpm_left;
    // v2Filt = 0.933 * v2Filt + 0.0335 * rpm_right + 0.0335 * v2Prev;
    // v2Prev = rpm_right;

    // Update previous tick counts
    prev_encoder1_ticks = current_encoder1_ticks;
    prev_encoder2_ticks = current_encoder2_ticks;

    // Calculate control effort using PID
    float control_left = left_pid.calculate(duty_cycle, speed_left, deltaT);
    float control_right = right_pid.calculate(duty_cycle, speed_right, deltaT);

    //float control_left = left_pid.calculate(duty_cycle, v1Filt, deltaT);
    //float control_right = right_pid.calculate(duty_cycle, v2Filt, deltaT);

    // Adjust right motor speed based on tick difference
    adjust_motor_speed_based_on_ticks(50, encoder1_ticks, encoder2_ticks, &control_left, &control_right);



    // Apply control effort
    left_motor.write(control_left);
    right_motor.write(control_right);

    // Update RPM and speed in PID controllers for logging
    left_pid.set_target(control_left);
    right_pid.set_target(control_right);
    left_pid.set_rpm(rpm_left);
    right_pid.set_rpm(rpm_right);
    left_pid.set_speed(speed_left);
    right_pid.set_speed(speed_right);

    timer_flag = true;
    return true;
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
    bool isIncrease = false;
    absolute_time_t last_print_time = get_absolute_time();
    while (true) {
        if (timer_flag) {
            timer_flag = false;

            // Print the status every 1 second
            absolute_time_t now = get_absolute_time();
            if (absolute_time_diff_us(last_print_time, now) >= print_interval_ms * 1000) {
                print_relative_time("Status Update", duty_cycle, 
                left_pid.get_target(), right_pid.get_target(), 
                left_pid.get_rpm(), right_pid.get_rpm(), 
                left_pid.get_speed(), right_pid.get_speed(), 
                encoder1_ticks, encoder2_ticks);

                // print_relative_time("Status Update", duty_cycle, 
                // left_pid.get_target(), 
                // left_pid.get_rpm(), 
                // left_pid.get_speed(), 
                // encoder1_ticks);

                last_print_time = now;

                // if (++step == 10) {
                //     step = 0;

                //     if(!isIncrease) {
                //         duty_cycle -= 0.1f;
                //         // duty_cycle이 감소하는 구간
                //         if(duty_cycle <= 0.3f && duty_cycle > 0.2f) {
                //             duty_cycle = -0.3f; // 0.3 구간을 통과하려고 0.4에서 0.3이 되면 -0.3으로 변경
                //         }
                //     } else {
                //         duty_cycle += 0.1f;
                //         // duty_cycle이 증가하는 구간
                //         if(duty_cycle >= -0.3f && duty_cycle < -0.2f) {
                //             duty_cycle = 0.3f; // -0.3 구간을 통과하려고 -0.3인 경우 0.3으로 변경
                //         }
                //     }

                //     if (duty_cycle >= 1.0f) {
                //         isIncrease = false;
                //     } else if (duty_cycle <= -1.0f) {
                //         isIncrease = true;
                //     }
                // }


            }
        }
    }
}
