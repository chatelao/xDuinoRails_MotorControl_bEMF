#include "motor_control_hal_rp2040_internal.h"

#if defined(ARDUINO_ARCH_RP2040)

static void on_pwm_wrap() {
    for (int id = 0; id < MAX_MOTORS; id++) {
        MotorContext* ctx = &g_motors[id];
        if (!ctx->is_initialized) continue;

        // Check if the interrupt was triggered by this motor's PWM slice.
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

static void pwm_init_common(MotorContext* ctx) {
    gpio_set_function(ctx->pwm_a_pin, GPIO_FUNC_PWM);
    gpio_set_function(ctx->pwm_b_pin, GPIO_FUNC_PWM);

    gpio_set_outover(ctx->pwm_b_pin, GPIO_OVERRIDE_INVERT);
    gpio_set_outover(ctx->pwm_a_pin, GPIO_OVERRIDE_INVERT);
    pwm_set_gpio_level(ctx->pwm_a_pin, PWM_MAX_TOP);
    pwm_set_gpio_level(ctx->pwm_b_pin, PWM_MAX_TOP);

    uint32_t system_clock = RP2040_SYSTEM_CLOCK_HZ;
    float divider = 1.0f;
    uint32_t top = system_clock / ctx->pwm_frequency;
    if (top > PWM_MAX_TOP) {
        divider = (float)top / (float)PWM_MAX_TOP;
        if (divider > PWM_MAX_DIVIDER) divider = PWM_MAX_DIVIDER;
    }
    ctx->pwm_wrap_value = (uint16_t)(system_clock / (ctx->pwm_frequency * divider)) - 1;

    pwm_config motor_pwm_conf = pwm_get_default_config();
    pwm_config_set_clkdiv(&motor_pwm_conf, divider);
    pwm_config_set_wrap(&motor_pwm_conf, ctx->pwm_wrap_value);
    pwm_config_set_phase_correct(&motor_pwm_conf, true);

    ctx->motor_pwm_slice_a = pwm_gpio_to_slice_num(ctx->pwm_a_pin);
    ctx->motor_pwm_slice_b = pwm_gpio_to_slice_num(ctx->pwm_b_pin);

    pwm_set_counter(ctx->motor_pwm_slice_a, 0);
    pwm_set_counter(ctx->motor_pwm_slice_b, 0);
    pwm_init(ctx->motor_pwm_slice_a, &motor_pwm_conf, false);
    pwm_init(ctx->motor_pwm_slice_b, &motor_pwm_conf, false);

    uint32_t mask = (1u << ctx->motor_pwm_slice_a) | (1u << ctx->motor_pwm_slice_b);
    hw_set_bits(&pwm_hw->en, mask);

    #ifdef DEBUG_PWM
        static bool debug_pwm_initialized = false;
        if (!debug_pwm_initialized) {
            gpio_set_function(DEBUG_PWM_PIN, GPIO_FUNC_PWM);
            uint debug_pwm_slice = pwm_gpio_to_slice_num(DEBUG_PWM_PIN);
            pwm_init(debug_pwm_slice, &motor_pwm_conf, true);
            const float pulse_width_us = 1.0f;
            uint16_t pulse_counts = (uint16_t)((pulse_width_us * 1e-6f * RP2040_SYSTEM_CLOCK_HZ) / divider);
            pwm_set_gpio_level(DEBUG_PWM_PIN, pulse_counts);
            debug_pwm_initialized = true;
        }
    #endif

    ctx->is_initialized = true;
}

#endif // ARDUINO_ARCH_RP2040
