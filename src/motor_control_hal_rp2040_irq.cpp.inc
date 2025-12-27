#include "motor_control_hal_rp2040_types_constants.h"

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

        if (ctx->bemf_mode == BEMF_MODE_DMA || ctx->bemf_mode == BEMF_MODE_IRQ) {
            trigger_adc_measurement(ctx);
        } else if (ctx->bemf_mode == BEMF_MODE_TIMER_ADC) {
            // If no alarm is currently active, schedule one.
            if (ctx->alarm_id == -1) {
                // The callback will fire once. It returns 0 to prevent re-arming.
                ctx->alarm_id = add_alarm_in_us(ctx->bemf_measurement_delay_us, delayed_adc_trigger_callback, ctx, true);
            }
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

static void adc_irq_handler() {
    if (g_current_motor_context == nullptr) {
        return;
    }

    MotorContext* ctx = g_current_motor_context;
    uint32_t sum_A = 0, sum_B = 0;
    int count = 0;

    // Drain the FIFO
    while (adc_fifo_get_level() >= 2) {
        sum_B += adc_fifo_get();
        sum_A += adc_fifo_get();
        count++;
    }

    if (ctx->bemf_mode == BEMF_MODE_IRQ) {
        if (count > 0) {
            int measured_bemf = abs((int)(sum_A / count) - (int)(sum_B / count));
            if (ctx->bemf_callback) {
                ctx->bemf_callback(measured_bemf);
            }
        }
        adc_run(false);
        g_adc_busy = false;
        g_current_motor_context = nullptr;
    } else if (ctx->bemf_mode == BEMF_MODE_TIMER_ADC) {
        if (count > 0) {
            // Store the reading in the ring buffer temporarily
            ctx->bemf_ring_buffer[ctx->bemf_samples_taken] = abs((int)(sum_A / count) - (int)(sum_B / count));
            ctx->bemf_samples_taken++;
        }

        if (ctx->bemf_samples_taken >= ctx->bemf_samples_to_take) {
            // All samples taken, calculate average and call back
            uint32_t total_sum = 0;
            for(int i = 0; i < ctx->bemf_samples_to_take; i++) {
                total_sum += ctx->bemf_ring_buffer[i];
            }
            if (ctx->bemf_callback) {
                ctx->bemf_callback(total_sum / ctx->bemf_samples_to_take);
            }
            adc_run(false);
            g_adc_busy = false;
            g_current_motor_context = nullptr;
        } else {
            // More samples to take, trigger next measurement
            trigger_adc_measurement(ctx);
        }
    }
}

#endif // ARDUINO_ARCH_RP2040
