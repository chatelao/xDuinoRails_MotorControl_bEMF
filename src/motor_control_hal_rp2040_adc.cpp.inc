#include "motor_control_hal_rp2040_types_constants.h"

#if defined(ARDUINO_ARCH_RP2040)

static void adc_init_common() {
    static bool adc_initialized = false;
    if (adc_initialized) return;

    adc_init();
    adc_fifo_setup(true, true, ADC_FIFO_THRESHOLD, false, false);
    adc_initialized = true;
}

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

    // This setup uses DMA for efficient, CPU-free ADC data capture.
    // The ADC is configured for round-robin sampling of the two BEMF pins.
    // As the ADC converts each sample, its hardware DREQ signal triggers the
    // DMA controller to transfer the result directly into the ring buffer.
    uint8_t adc_ch_a = ctx->bemf_a_pin - MOTOR_ADC_BASE_PIN;
    uint8_t adc_ch_b = ctx->bemf_b_pin - MOTOR_ADC_BASE_PIN;
    adc_select_input(adc_ch_b);
    adc_set_round_robin((1u << adc_ch_a) | (1u << adc_ch_b));
    adc_fifo_drain(); // Ensure FIFO is clear before starting a new capture

    // Start the DMA transfer which will be paced by the ADC's DREQ
    dma_bemf_start_capture(ctx);

    adc_run(true); // Enable the ADC to start the measurement
}

#endif // ARDUINO_ARCH_RP2040
