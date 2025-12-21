#include "motor_control_hal_rp2040_internal.h"

#if defined(ARDUINO_ARCH_RP2040)

static uint16_t pwm_set_frequency(pwm_config *pwm_conf, uint32_t pwm_frequency) {
    uint32_t total_div      = RP2040_SYSTEM_CLOCK_HZ / pwm_frequency / 2;
    float    clk_divider    =                      1.0f;
    uint16_t pwm_wrap_value =                         0;

    if (total_div > PWM_MAX_TOP) {
        clk_divider = (float)total_div / PWM_MAX_TOP;
        if (clk_divider > PWM_MAX_DIVIDER) clk_divider = PWM_MAX_DIVIDER;
        pwm_wrap_value = PWM_MAX_TOP;
    } else if (total_div < 1) {
        clk_divider    = 1.0f;
        pwm_wrap_value = 1;
    } else {
        clk_divider    = 1.0f;
        pwm_wrap_value = (uint16_t)total_div;
    }

    pwm_config_set_clkdiv( pwm_conf, clk_divider);
    pwm_config_set_wrap(   pwm_conf, pwm_wrap_value);

    return pwm_wrap_value;
}

static uint pwm_init_pin(uint8_t pwm_pin) {
    gpio_set_function(  pwm_pin, GPIO_FUNC_PWM);
    gpio_set_outover(   pwm_pin, GPIO_OVERRIDE_INVERT);
    pwm_set_gpio_level( pwm_pin, PWM_MAX_TOP);

    uint motor_pwm_slice = pwm_gpio_to_slice_num(pwm_pin);
    pwm_set_counter(motor_pwm_slice, 0);
    return motor_pwm_slice;
}

static void pwm_set_duty_level(MotorContext* ctx) {
    if (ctx->motor_pwm_slice_a == ctx->motor_pwm_slice_b) {
        pwm_set_both_levels(ctx->motor_pwm_slice_a, ctx->next_duty_level_a, ctx->next_duty_level_b);
    } else {
        pwm_set_gpio_level(ctx->pwm_a_pin, ctx->next_duty_level_a);
        pwm_set_gpio_level(ctx->pwm_b_pin, ctx->next_duty_level_b);
    }
}

static void pwm_init_common(MotorContext* ctx) {
    ctx->motor_pwm_slice_a = pwm_init_pin(ctx->pwm_a_pin);
    ctx->motor_pwm_slice_b = pwm_init_pin(ctx->pwm_b_pin);

    // Default to OFF state, which is MAX_TOP due to inverted output
    ctx->next_duty_level_a = PWM_MAX_TOP;
    ctx->next_duty_level_b = PWM_MAX_TOP;

    pwm_config motor_pwm_conf = pwm_get_default_config();
    ctx->pwm_wrap_value = pwm_set_frequency(&motor_pwm_conf, ctx->pwm_frequency);
    pwm_config_set_phase_correct(&motor_pwm_conf, true);

    pwm_init(ctx->motor_pwm_slice_a, &motor_pwm_conf, false);
    pwm_init(ctx->motor_pwm_slice_b, &motor_pwm_conf, false);

    irq_init_pwm(ctx);

    uint32_t mask = (1u << ctx->motor_pwm_slice_a) | (1u << ctx->motor_pwm_slice_b);
    hw_set_bits(&pwm_hw->en, mask);

    ctx->is_initialized = true;
}

#endif // ARDUINO_ARCH_RP2040
