#include "motor_control.h"

DCMotor::DCMotor(uint enA_pin, uint enB_pin, uint pwm_pin, float min_duty, float max_duty)
    : _enA_pin(enA_pin), _enB_pin(enB_pin), _pwm_pin(pwm_pin), _min_duty(min_duty), _max_duty(max_duty)
{
    // Initialize PWM
    _slice_num = pwm_gpio_to_slice_num(_pwm_pin);
    _channel = pwm_gpio_to_channel(_pwm_pin);
    gpio_set_function(_pwm_pin, GPIO_FUNC_PWM);

    // pwm_set_wrap(_slice_num, TOP);

    // Set PWM frequency
    set_pwm_frequency(PWM_FREQUENCY);

    pwm_set_chan_level(_slice_num, _channel, 0);

    // Initialize GPIO
    gpio_init(_enA_pin);
    gpio_init(_enB_pin);
    gpio_set_dir(_enA_pin, GPIO_OUT);
    gpio_set_dir(_enB_pin, GPIO_OUT);
    gpio_put(_enA_pin, false);
    gpio_put(_enB_pin, false);

    // Enable PWM
    pwm_set_enabled(_slice_num, true);
}

void DCMotor::set_pwm_frequency(float frequency)
{
    float divider = SYSTEM_CLOCK / (frequency * TOP);
    pwm_set_clkdiv(_slice_num, divider);

    uint32_t wrap = SYSTEM_CLOCK / (divider * frequency) - 1;
    pwm_set_wrap(_slice_num, wrap);
}

void DCMotor::write_int16(int16_t pwm)
{
    if(pwm < 0)
    {
        gpio_put(_enA_pin, true);
        gpio_put(_enB_pin, false);
        pwm_set_chan_level(_slice_num, _channel, abs(pwm));
    } else
    {
        gpio_put(_enA_pin, false);
        gpio_put(_enB_pin, true);
        pwm_set_chan_level(_slice_num, _channel, pwm);
    }
    pwm_set_enabled(_slice_num, true);
}

void DCMotor::write(float duty_cycle)
{
    if(duty_cycle >= _max_duty) duty_cycle = _max_duty;
    if(duty_cycle <= _min_duty) duty_cycle = _min_duty;
    write_int16((int16_t)(duty_cycle * TOP));
}
