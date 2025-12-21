#include "motor_control_hal_rp2040_internal.h" // Include the internal header for RP2040 HAL.

#if defined(ARDUINO_ARCH_RP2040) // Compiles this file only for the RP2040 architecture.

static void trigger_adc_measurement(MotorContext* ctx) { // Function to trigger an ADC measurement.
    if (!ctx) return; // If the motor context is null, return.

    if (g_adc_busy) return; // If the ADC is busy, return.

    g_adc_busy = true; // Set the ADC busy flag.

    uint8_t adc_ch_a = ctx->bemf_a_pin - MOTOR_ADC_BASE_PIN; // Calculate the ADC channel for BEMF pin A.
    uint8_t adc_ch_b = ctx->bemf_b_pin - MOTOR_ADC_BASE_PIN; // Calculate the ADC channel for BEMF pin B.
    adc_select_input(adc_ch_b); // Select the ADC input for BEMF pin B.
    adc_set_round_robin((1u << adc_ch_a) | (1u << adc_ch_b)); // Set the ADC to round-robin between channels A and B.
    adc_fifo_drain(); // Drain the ADC FIFO.

    dma_channel_set_trans_count(ctx->dma_channel_bemf, BEMF_RING_BUFFER_SIZE, false); // Set the DMA transfer count for BEMF.
    dma_channel_set_write_addr(ctx->dma_channel_bemf, ctx->bemf_ring_buffer, true); // Set the DMA write address for the BEMF ring buffer.

    adc_run(true); // Start the ADC.
}

#endif // ARDUINO_ARCH_RP2040
