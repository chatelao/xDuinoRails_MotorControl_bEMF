#include "motor_control_hal_rp2040_internal.h"

#if defined(ARDUINO_ARCH_RP2040)

/**
 * @brief Triggers a BEMF ADC measurement sequence.
 * @param ctx Pointer to the motor's context.
 * @note This function sets up the ADC for a round-robin capture of the two BEMF
 * phases, drains the FIFO, starts the DMA capture, and then enables the ADC. The
 * actual data transfer is handled by the DMA controller, and the processing is
 * done in the DMA IRQ handler.
 */
static void trigger_adc_measurement(MotorContext* ctx) {
    if (!ctx || ctx->skip_measurement) return;
    if (g_adc_busy) return;
    g_adc_busy = true;

    // Configure ADC for round-robin sampling of the two BEMF pins
    uint8_t adc_ch_a = ctx->bemf_a_pin - MOTOR_ADC_BASE_PIN;
    uint8_t adc_ch_b = ctx->bemf_b_pin - MOTOR_ADC_BASE_PIN;
    adc_select_input(adc_ch_b);
    adc_set_round_robin((1u << adc_ch_a) | (1u << adc_ch_b));
    adc_fifo_drain(); // Ensure FIFO is clear before starting a new capture

    // Start the DMA transfer which will be paced by the ADC's DREQ
    dma_bemf_start_capture(ctx);

    adc_run(true); // Enable the ADC to start the measurement
}

int hal_motor_get_bemf_buffer(volatile uint16_t** buffer, int* last_write_pos, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return 0;
    MotorContext* ctx = &g_motors[motor_id];
    if (!ctx->is_initialized || ctx->dma_channel_bemf < 0) return 0;

    *buffer = ctx->bemf_ring_buffer;

    // Directly read the current write address from the DMA hardware registers.
    // This provides the most up-to-date position in the ring buffer.
    uintptr_t current_write_addr = (uintptr_t)dma_hw->ch[ctx->dma_channel_bemf].write_addr;
    uintptr_t buffer_start_addr = (uintptr_t)ctx->bemf_ring_buffer;
    int byte_offset = current_write_addr - buffer_start_addr;

    // Convert byte offset to array index (since buffer is uint16_t)
    *last_write_pos = byte_offset / sizeof(uint16_t);

    return BEMF_RING_BUFFER_SIZE;
}

#endif // ARDUINO_ARCH_RP2040
