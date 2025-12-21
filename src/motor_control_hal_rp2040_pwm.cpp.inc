#include "motor_control_hal_rp2040_internal.h"

#if defined(ARDUINO_ARCH_RP2040)

static void pwm_set_frequency(pwm_config *pwm_conf, uint32_t pwm_frequency, uint16_t* pwm_wrap_value_out) {
    uint32_t total_div      = RP2040_SYSTEM_CLOCK_HZ / pwm_frequency / 2;
    float    clk_divider    = 1.0f;
    uint16_t pwm_wrap_value = 0;

    if (total_div > PWM_MAX_TOP) {
        clk_divider = (float)total_div / PWM_MAX_TOP;
        if (clk_divider > PWM_MAX_DIVIDER) clk_divider = PWM_MAX_DIVIDER;
        pwm_wrap_value = PWM_MAX_TOP;
    } else if (total_div < 1) {
        clk_divider = 1.0f;
        pwm_wrap_value = 1;
    } else {
        clk_divider = 1.0f;
        pwm_wrap_value = (uint16_t)total_div;
    }

    pwm_config_set_clkdiv(pwm_conf, clk_divider);
    pwm_config_set_wrap(pwm_conf, pwm_wrap_value);
    *pwm_wrap_value_out = pwm_wrap_value;
}

static uint pwm_init_pin(uint8_t pwm_pin) {
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    gpio_set_outover(pwm_pin, GPIO_OVERRIDE_INVERT);
    pwm_set_gpio_level(pwm_pin, PWM_MAX_TOP);
    uint motor_pwm_slice = pwm_gpio_to_slice_num(pwm_pin);
    pwm_set_counter(motor_pwm_slice, 0);
    return motor_pwm_slice;
}

static void on_pwm_wrap() {
    for (int id = 0; id < MAX_MOTORS; id++) {
        MotorContext* ctx = &g_motors[id];
        if (!ctx->is_initialized) continue;

        if (pwm_get_irq_status_mask() & (1u << ctx->motor_pwm_slice_a)) {
            pwm_clear_irq(ctx->motor_pwm_slice_a);

            #if defined(MOTOR_CURRENT_PIN)
                if (!g_adc_busy) {
                    adc_select_input(MOTOR_CURRENT_PIN - MOTOR_ADC_BASE_PIN);
                    uint16_t current_val = adc_read();
                    const float V_limit = MAX_CURRENT_AMPS * SHUNT_RESISTOR_OHMS;
                    const uint16_t adc_threshold = (uint16_t)((V_limit / ADC_REF_VOLTAGE) * ADC_MAX_VALUE);

                    if (current_val > adc_threshold) {
                        pwm_set_gpio_level(ctx->pwm_a_pin, PWM_MAX_TOP); pwm_set_gpio_level(ctx->pwm_b_pin, PWM_MAX_TOP);
                        return;
                    }
                }
            #endif

            if (ctx->shunt_callback) {
                trigger_adc_measurement(ctx, SHUNT);
            }

            if (!ctx->skip_measurement) {
                ctx->alarm_user_data.ctx = ctx;
                ctx->alarm_user_data.type = BEMF;
                add_alarm_in_us(ctx->adc_trigger_delay_us, delayed_adc_trigger_callback, &ctx->alarm_user_data, true);
            }
        }
    }
}

static void pwm_set_motor_levels(MotorContext* ctx, uint16_t level_a, uint16_t level_b) {
    if (ctx->motor_pwm_slice_a == ctx->motor_pwm_slice_b) {
        pwm_set_both_levels(ctx->motor_pwm_slice_a, level_a, level_b);
    } else {
        pwm_set_gpio_level(ctx->pwm_a_pin, level_a);
        pwm_set_gpio_level(ctx->pwm_b_pin, level_b);
    }
}

static void pwm_init_common(MotorContext* ctx) {
    ctx->motor_pwm_slice_a = pwm_init_pin(ctx->pwm_a_pin);
    ctx->motor_pwm_slice_b = pwm_init_pin(ctx->pwm_b_pin);

    pwm_config motor_pwm_conf = pwm_get_default_config();
    pwm_set_frequency(&motor_pwm_conf, ctx->pwm_frequency, &ctx->pwm_wrap_value);
    pwm_config_set_phase_correct(&motor_pwm_conf, true);

    pwm_init(ctx->motor_pwm_slice_a, &motor_pwm_conf, false);
    pwm_init(ctx->motor_pwm_slice_b, &motor_pwm_conf, false);

    uint32_t mask = (1u << ctx->motor_pwm_slice_a) | (1u << ctx->motor_pwm_slice_b);
    hw_set_bits(&pwm_hw->en, mask);

    ctx->is_initialized = true;
}

#endif // ARDUINO_ARCH_RP2040
