#include "motor_control_hal_rp2040_types_constants.h"

#if defined(ARDUINO_ARCH_RP2040)

// Forward declaration for the IRQ handler, which is used to set up the interrupt
// before the function itself is defined.
static void dma_bemf_irq_handler();

/**
 * @brief Starts the DMA transfer for BEMF ADC samples.
 * @param ctx Pointer to the motor's context.
 * @note This function is designed to be called from the ADC trigger logic. It resets
 * the DMA transfer count and write address to begin a new capture sequence into the
 * ring buffer.
 */
static void dma_bemf_start_capture(MotorContext* ctx) {
    if (!ctx || ctx->dma_channel_bemf < 0) return;

    // Reset the DMA transfer count and set the write address to the start of the ring buffer.
    // This effectively restarts the DMA capture for each BEMF measurement cycle.
    dma_channel_set_trans_count(ctx->dma_channel_bemf, BEMF_RING_BUFFER_SIZE, false);
    dma_channel_set_write_addr(ctx->dma_channel_bemf, ctx->bemf_ring_buffer, true);
}

/**
 * @brief DMA interrupt handler for BEMF measurement.
 * @note This function is triggered when the DMA controller has finished transferring the
 * configured number of ADC samples (BEMF_RING_BUFFER_SIZE) into the ring buffer. It
 * stops the ADC, processes the collected samples to calculate the average BEMF voltage,
 * and invokes the user-provided callback with the result.
 */
static void dma_bemf_irq_handler() {
    for (int id = 0; id < MAX_MOTORS; id++) {
        MotorContext* ctx = &g_motors[id];
        if(!ctx->is_initialized || ctx->dma_channel_bemf < 0) continue;

        // Skip if the interrupt is not for this motor's BEMF DMA channel
        if (!(dma_hw->ints0 & (1u << ctx->dma_channel_bemf))) continue;

        adc_run(false);    // Stop the ADC as the capture is complete
        g_adc_busy = false; // Clear the global ADC busy flag

        dma_hw->ints0 = 1u << ctx->dma_channel_bemf; // Clear the DMA interrupt flag

        // Process the captured data by averaging the samples for each phase
        uint32_t sum_A = 0, sum_B = 0;
        for (uint i = 0; i < BEMF_RING_BUFFER_SIZE; i += 2) {
            sum_B += ctx->bemf_ring_buffer[i];
            sum_A += ctx->bemf_ring_buffer[i + 1];
        }

        int count = BEMF_RING_BUFFER_SIZE / 2;
        if (count > 0) {
            int measured_bemf = abs((int)(sum_A / count) - (int)(sum_B / count));
            if (ctx->bemf_callback) {
                ctx->bemf_callback(measured_bemf);
            }
        }
    }
}

// =============================================================================
// Public HAL Function Implementations
// =============================================================================

/**
 * @brief Initializes the DMA for BEMF sensing.
 *
 * Configures a DMA channel to continuously transfer ADC conversion results
 * into a ring buffer. An interrupt is configured to fire when the buffer is
 * full, which triggers the processing of the collected BEMF data.
 *
 * @param callback The function to call with new BEMF data.
 * @param motor_id The index of the motor to initialize.
 */
void hal_motor_init_bemf_dma(hal_bemf_update_callback_t callback, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return;
    MotorContext* ctx = &g_motors[motor_id];
    if (!ctx->is_initialized) return;

    ctx->bemf_callback = callback;
    ctx->bemf_mode = BEMF_MODE_DMA;
    bool bemf_enabled = (ctx->bemf_a_pin != MOTOR_PIN_UNDEFINED) && (ctx->bemf_b_pin != MOTOR_PIN_UNDEFINED);
    if (!bemf_enabled) return;

    ctx->dma_channel_bemf = dma_claim_unused_channel(true);
    dma_channel_config dma_config = dma_channel_get_default_config(ctx->dma_channel_bemf);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_config, false);
    channel_config_set_write_increment(&dma_config, true);
    channel_config_set_dreq(&dma_config, DREQ_ADC);

    // Configure the DMA to use a ring buffer. This makes the write address wrap around
    // to the beginning when it reaches the end, which is ideal for continuous sampling.
    // The ring size is set to the total buffer size in bytes, calculated as the number
    // of samples multiplied by the size of each sample (uint16_t). The `__builtin_ctz`
    // intrinsic is a highly efficient way to compute the required log2 of the buffer size.
    channel_config_set_ring(&dma_config, true, __builtin_ctz(BEMF_RING_BUFFER_SIZE * sizeof(uint16_t)));

    dma_channel_configure(
        ctx->dma_channel_bemf, &dma_config,
        ctx->bemf_ring_buffer, // Destination: our ring buffer
        &adc_hw->fifo,         // Source: the ADC FIFO
        BEMF_RING_BUFFER_SIZE, // Number of transfers
        false                  // Don't trigger immediately
    );

    dma_channel_set_irq0_enabled(ctx->dma_channel_bemf, true);
    static bool dma_irq_initialized = false;
    if (!dma_irq_initialized) {
        irq_set_exclusive_handler(DMA_IRQ_0, dma_bemf_irq_handler);
        irq_set_enabled(DMA_IRQ_0, true);
        dma_irq_initialized = true;
    }
}

/**
 * @brief Retrieves the BEMF ring buffer for diagnostics.
 *
 * Provides direct, low-level access to the raw ADC sample buffer for a given
 * motor. This is primarily intended for debugging, visualization, or advanced
 * control algorithms that need to inspect the BEMF waveform directly.
 *
 * @param[out] buffer Pointer to a volatile uint16_t pointer, which will be
 *                    set to the address of the internal ring buffer.
 * @param[out] last_write_pos Pointer to an integer that will be updated with
 *                            the most recent write position in the buffer.
 * @param motor_id The index of the motor whose buffer should be retrieved.
 * @return The total size of the ring buffer in number of samples.
 */
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
