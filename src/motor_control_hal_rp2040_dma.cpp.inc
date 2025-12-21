#if defined(ARDUINO_ARCH_RP2040)

static void dma_irq_handler();

void hal_motor_init_bemf_dma(hal_bemf_update_callback_t callback, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return;
    MotorContext* ctx = &g_motors[motor_id];
    if (!ctx->is_initialized) return;

    ctx->bemf_callback = callback;
    bool bemf_enabled = (ctx->bemf_a_pin != MOTOR_PIN_UNDEFINED) && (ctx->bemf_b_pin != MOTOR_PIN_UNDEFINED);
    if (!bemf_enabled) return;

    ctx->dma_channel_bemf = dma_claim_unused_channel(true);
    dma_channel_config dma_config = dma_channel_get_default_config(ctx->dma_channel_bemf);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_config, false);
    channel_config_set_write_increment(&dma_config, true);
    channel_config_set_dreq(&dma_config, DREQ_ADC);
    channel_config_set_ring(&dma_config, true, __builtin_ctz(BEMF_RING_BUFFER_SIZE * sizeof(uint16_t)));

    dma_channel_configure(
        ctx->dma_channel_bemf, &dma_config,
        ctx->bemf_ring_buffer, &adc_hw->fifo,
        BEMF_RING_BUFFER_SIZE, false
    );

    dma_channel_set_irq0_enabled(ctx->dma_channel_bemf, true);
    static bool dma_irq_initialized = false;
    if (!dma_irq_initialized) {
        irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
        irq_set_enabled(DMA_IRQ_0, true);
        dma_irq_initialized = true;
    }
}

int hal_motor_get_bemf_buffer(volatile uint16_t** buffer, int* last_write_pos, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return 0;
    MotorContext* ctx = &g_motors[motor_id];
    if (!ctx->is_initialized) return 0;

    *buffer = ctx->bemf_ring_buffer;

    uintptr_t current_write_addr = (uintptr_t)dma_hw->ch[ctx->dma_channel_bemf].write_addr;
    uintptr_t buffer_start_addr = (uintptr_t)ctx->bemf_ring_buffer;
    int byte_offset = current_write_addr - buffer_start_addr;

    *last_write_pos = byte_offset / sizeof(uint16_t);

    return BEMF_RING_BUFFER_SIZE;
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
