#include "motor_control_hal_rp2040_internal.h"

#if defined(ARDUINO_ARCH_RP2040)

void __not_in_flash_func(on_pwm_wrap)() {
    // This ISR is shared by all PWM slices. It iterates through all motors
    // and checks if the interrupt was triggered for their specific slice.
    uint32_t irq_status = pwm_get_irq_status_mask();

    for (int id = 0; id < MAX_MOTORS; id++) {
        MotorContext* ctx = &g_motors[id];
        if (!ctx->is_initialized) continue;

        // Check if the interrupt was triggered for the current motor's slice.
        if (irq_status & (1u << ctx->motor_pwm_slice_a)) {
            pwm_clear_irq(ctx->motor_pwm_slice_a);

            // Atomically update PWM duty cycles for motors on different slices.
            // Motors on the same slice are updated directly in `hal_motor_set_duty`.
            if (ctx->motor_pwm_slice_a != ctx->motor_pwm_slice_b) {
                pwm_set_gpio_level(ctx->pwm_a_pin, ctx->next_duty_level_a);
                pwm_set_gpio_level(ctx->pwm_b_pin, ctx->next_duty_level_b);
            }

            if (!ctx->skip_measurement) {
                trigger_adc_measurement(ctx, BEMF);
            }
        }
    }
}

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

    #ifdef USE_IRQ_TRIGGER
        // The IRQ handler is shared across all PWM slices. It's set once.
        static bool pwm_irq_initialized = false;
        if (!pwm_irq_initialized) {
            irq_set_exclusive_handler( PWM_IRQ_WRAP, on_pwm_wrap );
            irq_set_enabled(           PWM_IRQ_WRAP, true );
            pwm_irq_initialized = true;
        }
        // Enable the interrupt for the specific slice of the current motor.
        pwm_clear_irq(      ctx->motor_pwm_slice_a);
        pwm_set_irq_enabled(ctx->motor_pwm_slice_a, true);
    #endif

    uint32_t mask = (1u << ctx->motor_pwm_slice_a) | (1u << ctx->motor_pwm_slice_b);
    hw_set_bits(&pwm_hw->en, mask);

    ctx->is_initialized = true;
}

#endif // ARDUINO_ARCH_RP2040
