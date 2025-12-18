#include "motor_control_hal.h"
#include <Arduino.h>

/**
 * @brief Checks the solenoid/motor position by pinging both directions and measuring the response.
 *
 * This function performs a diagnostic test:
 * 1. Pings the "forward" direction (PWM A).
 * 2. Measures the BEMF/Response immediately after switch-off.
 * 3. Pings the "reverse" direction (PWM B).
 * 4. Measures the BEMF/Response immediately after switch-off.
 *
 * @param ping_pwm_value The PWM duty cycle (0-255) for the ping pulse.
 * @param ping_duration_ms The duration of the ping pulse in milliseconds.
 * @param measurement_delay_us The delay after switching off before measuring the response.
 * @param[out] response_a The measured response for the forward/A direction.
 * @param[out] response_b The measured response for the reverse/B direction.
 * @param motor_id The index of the motor to control (0 to MAX_MOTORS-1). Defaults to 0.
 */
void hal_motor_check_solenoid_position(int ping_pwm_value, int ping_duration_ms, int measurement_delay_us, int* response_a, int* response_b, uint8_t motor_id) {
    // We need to capture the BEMF value. The HAL usually provides it via callback or buffer.
    // Since this is a synchronous diagnostic function, we can't easily rely on the asynchronous callback
    // unless we hook it or poll the buffer.
    // Polling the buffer is safer as it doesn't require modifying the global callback state which might be in use.

    volatile uint16_t* buffer;
    int last_write_pos;
    int buffer_size;

    // --- Ping A (Forward) ---
    hal_motor_set_pwm(ping_pwm_value, true, motor_id);
    delay(ping_duration_ms);
    hal_motor_set_pwm(0, true, motor_id); // Switch off

    // Wait for the measurement delay.
    delayMicroseconds(measurement_delay_us);

    // Read the latest value from the buffer
    buffer_size = hal_motor_get_bemf_buffer(&buffer, &last_write_pos, motor_id);
    if (buffer_size > 0 && buffer != nullptr) {
        // Get the last written sample.
        int index = (last_write_pos - 1 + buffer_size) % buffer_size;

        int idx_a, idx_b;
        if (index % 2 != 0) { // index is Odd (A)
            idx_a = index;
            idx_b = (index - 1 + buffer_size) % buffer_size;
        } else { // index is Even (B)
             idx_b = index;
             idx_a = (index - 1 + buffer_size) % buffer_size;
        }

        // Calculate differential
        int val_a = buffer[idx_a];
        int val_b = buffer[idx_b];
        *response_a = abs(val_a - val_b);

    } else {
        *response_a = -1; // Error or not supported
    }

    // Short pause between pings to let things settle
    delay(50);

    // --- Ping B (Reverse) ---
    hal_motor_set_pwm(ping_pwm_value, false, motor_id);
    delay(ping_duration_ms);
    hal_motor_set_pwm(0, false, motor_id); // Switch off

    delayMicroseconds(measurement_delay_us);

    buffer_size = hal_motor_get_bemf_buffer(&buffer, &last_write_pos, motor_id);
    if (buffer_size > 0 && buffer != nullptr) {
        int index = (last_write_pos - 1 + buffer_size) % buffer_size;
        int idx_a, idx_b;
        if (index % 2 != 0) { // index is Odd (A)
            idx_a = index;
            idx_b = (index - 1 + buffer_size) % buffer_size;
        } else { // index is Even (B)
             idx_b = index;
             idx_a = (index - 1 + buffer_size) % buffer_size;
        }

        int val_a = buffer[idx_a];
        int val_b = buffer[idx_b];
        *response_b = abs(val_a - val_b);
    } else {
        *response_b = -1;
    }

    // Ensure motor is off
    hal_motor_set_pwm(0, true, motor_id);
}
