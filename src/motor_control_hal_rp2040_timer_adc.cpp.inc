#pragma once

// Forward declaration for the timer callback
static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data);

// Implementation of the public API function
void hal_motor_init_bemf_timer_adc(hal_bemf_update_callback_t callback, uint32_t measurement_delay_us, uint8_t samples_to_take, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) {
        return;
    }
    MotorContext* ctx = &g_motors[motor_id];

    ctx->bemf_mode = BEMF_MODE_TIMER_ADC;
    ctx->bemf_callback = callback;
    ctx->bemf_measurement_delay_us = measurement_delay_us;
    ctx->bemf_samples_to_take = samples_to_take;
    ctx->bemf_samples_taken = 0;
    ctx->alarm_id = -1;

    adc_init_common();
}

// Timer callback function
static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data) {
    MotorContext* ctx = (MotorContext*)user_data;

    // Mark alarm as inactive
    ctx->alarm_id = -1;

    // It's time to start the ADC measurement sequence
    if (!g_adc_busy) {
        g_adc_busy = true;
        g_current_motor_context = ctx;
        ctx->bemf_samples_taken = 0; // Reset for the new sequence
        trigger_adc_measurement(ctx);
    }

    // A return value of 0 means the alarm will not repeat.
    return 0;
}
