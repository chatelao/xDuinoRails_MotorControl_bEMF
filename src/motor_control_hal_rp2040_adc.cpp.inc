#include "motor_control_hal_rp2040_internal.h" // Include the internal header for RP2040 HAL.

#if defined(ARDUINO_ARCH_RP2040) // Compiles this file only for the RP2040 architecture.

static void trigger_adc_measurement(MotorContext* ctx, MeasurementType type) { // Function to trigger an ADC measurement.
    if (!ctx) return; // If the motor context is null, return.

    if (g_adc_busy) return; // If the ADC is busy, return.

    g_adc_busy = true; // Set the ADC busy flag.

    if (type == BEMF) { // If the measurement type is BEMF.
        uint8_t adc_ch_a = ctx->bemf_a_pin - MOTOR_ADC_BASE_PIN; // Calculate the ADC channel for BEMF pin A.
        uint8_t adc_ch_b = ctx->bemf_b_pin - MOTOR_ADC_BASE_PIN; // Calculate the ADC channel for BEMF pin B.
        adc_select_input(adc_ch_b); // Select the ADC input for BEMF pin B.
        adc_set_round_robin((1u << adc_ch_a) | (1u << adc_ch_b)); // Set the ADC to round-robin between channels A and B.
        adc_fifo_drain(); // Drain the ADC FIFO.

        dma_channel_set_trans_count(ctx->dma_channel_bemf, BEMF_RING_BUFFER_SIZE, false); // Set the DMA transfer count for BEMF.
        dma_channel_set_write_addr(ctx->dma_channel_bemf, ctx->bemf_ring_buffer, true); // Set the DMA write address for the BEMF ring buffer.
    }

    adc_run(true); // Start the ADC.
}

static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data) { // Callback function for the delayed ADC trigger.
    AlarmUserData* data = (AlarmUserData*)user_data; // Cast the user data to AlarmUserData.
    trigger_adc_measurement(data->ctx, data->type); // Trigger the ADC measurement.
    return 0; // Return 0.
}

static void dma_irq_handler() { // DMA interrupt handler.
    for (int id = 0; id < MAX_MOTORS; id++) { // Loop through all motors.
        MotorContext* ctx = &g_motors[id]; // Get the motor context.
        if (!ctx->is_initialized) continue; // If the motor is not initialized, continue.

        if (ctx->dma_channel_bemf >= 0 && (dma_hw->ints0 & (1u << ctx->dma_channel_bemf))) { // If the BEMF DMA channel interrupt is pending.

            adc_run(false); // Stop the ADC.
            g_adc_busy = false; // Clear the ADC busy flag.

            dma_hw->ints0 = 1u << ctx->dma_channel_bemf; // Clear the DMA interrupt.

            uint32_t sum_A = 0, sum_B = 0; // Initialize sums for channels A and B.

            for (uint i = 0; i < BEMF_RING_BUFFER_SIZE; i += 2) { // Loop through the BEMF ring buffer.
                sum_B += ctx->bemf_ring_buffer[i]; // Add the value from channel B to sum_B.
                sum_A += ctx->bemf_ring_buffer[i + 1]; // Add the value from channel A to sum_A.
            }

            int count = BEMF_RING_BUFFER_SIZE / 2; // Calculate the number of samples per channel.
            int measured_bemf = abs((int)(sum_A / count) - (int)(sum_B / count)); // Calculate the measured BEMF.

            if (ctx->bemf_callback) { // If a BEMF callback is registered.
                ctx->bemf_callback(measured_bemf); // Call the BEMF callback.
            }
        }
    }
}

#endif // ARDUINO_ARCH_RP2040
