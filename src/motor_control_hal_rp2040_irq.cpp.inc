#include "motor_control_hal_rp2040_internal.h"

#if defined(ARDUINO_ARCH_RP2040)

#ifdef USE_IRQ_TRIGGER

void __not_in_flash_func(on_pwm_wrap)() {
    // This ISR is shared by all PWM slices. It iterates through all motors
    // and checks if the interrupt was triggered for their specific slice.
    uint32_t irq_pwm_slices = pwm_get_irq_status_mask();

    for (int id = 0; id < MAX_MOTORS; id++) {
        MotorContext* ctx = &g_motors[id];

        if (  !ctx->is_initialized                                // Current motor is not active
           || !(irq_pwm_slices & (1u << ctx->motor_pwm_slice_a))  // Current slice did not trigger the interrupt
           )
          continue;

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

static void pwm_init_irq(MotorContext* ctx) {
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
}

#endif // USE_IRQ_TRIGGER

#endif // ARDUINO_ARCH_RP2040
