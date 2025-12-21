#if defined(ARDUINO_ARCH_STM32)

void hal_motor_init_bemf_adc_dma(uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback, uint8_t motor_id) {
    hal_motor_init_bemf_adc(bemf_a_pin, bemf_b_pin, motor_id);
    hal_motor_init_bemf_dma(callback, motor_id);
}

int hal_motor_get_bemf_buffer(volatile uint16_t** buffer, int* last_write_pos, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) {
        return 0;
    }

    MotorContext* context = &motor_contexts[motor_id];
    *buffer = context->bemf_buffer;

    // The STM32 DMA controller provides the number of remaining data units to be transferred.
    // We can use this to calculate the last write position.
    *last_write_pos = (BEMF_RING_BUFFER_SIZE * 2) - __HAL_DMA_GET_COUNTER(&context->hdma_adc);

    return BEMF_RING_BUFFER_SIZE * 2;
}

// --- Placeholder for Shunt functionality ---
void hal_motor_init_shunt_adc(uint8_t shunt_a_pin, uint8_t shunt_b_pin, uint8_t motor_id) {
    // Not yet implemented
}

void hal_motor_init_shunt_dma(hal_shunt_update_callback_t callback, uint8_t motor_id) {
    // Not yet implemented
}

void hal_motor_init_shunt_adc_dma(uint8_t shunt_a_pin, uint8_t shunt_b_pin, hal_shunt_update_callback_t callback, uint8_t motor_id) {
    // Not yet implemented
}

int hal_motor_get_shunt_buffer(volatile uint16_t** buffer, int* last_write_pos, uint8_t motor_id) {
    // Not yet implemented
    return 0;
}


#endif // ARDUINO_ARCH_STM32
