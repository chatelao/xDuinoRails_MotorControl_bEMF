#include "motor_control_hal_rp2040_internal.h"

#if defined(ARDUINO_ARCH_RP2040)

// =============================================================================
// Global Variables
// =============================================================================
MotorContext g_motors[MAX_MOTORS];
volatile bool g_adc_busy = false;

// =============================================================================
// Public HAL Function Implementations
// =============================================================================

void hal_motor_init_pwm(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint32_t pwm_frequency, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return;
    MotorContext* ctx = &g_motors[motor_id];

    ctx->pwm_frequency = pwm_frequency;
    ctx->pwm_a_pin = pwm_a_pin;
    ctx->pwm_b_pin = pwm_b_pin;
    ctx->dma_channel_bemf = -1;
    ctx->dma_channel_shunt = -1;

    pwm_init_common(ctx);
}

void hal_motor_init_bemf_adc_dma(uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return;
    MotorContext* ctx = &g_motors[motor_id];
    if (!ctx->is_initialized) return;

    ctx->bemf_a_pin = bemf_a_pin;
    ctx->bemf_b_pin = bemf_b_pin;
    ctx->bemf_callback = callback;
    ctx->adc_trigger_delay_us = BEMF_MEASUREMENT_DELAY_US;

    bool bemf_enabled = (bemf_a_pin != MOTOR_PIN_UNDEFINED) && (bemf_b_pin != MOTOR_PIN_UNDEFINED);
    ctx->skip_measurement = !bemf_enabled;
    if (!bemf_enabled) return;

    static bool adc_initialized = false;
    if (!adc_initialized) {
        adc_init();
        adc_fifo_setup(true, true, ADC_FIFO_THRESHOLD, false, false);
        adc_initialized = true;
    }

    adc_gpio_init(ctx->bemf_a_pin);
    adc_gpio_init(ctx->bemf_b_pin);

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

void hal_motor_init_shunt_adc_dma(uint8_t shunt_a_pin, uint8_t shunt_b_pin, hal_shunt_update_callback_t callback, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return;
    MotorContext* ctx = &g_motors[motor_id];
    if (!ctx->is_initialized) return;

    ctx->shunt_a_pin = shunt_a_pin;
    ctx->shunt_b_pin = shunt_b_pin;
    ctx->shunt_callback = callback;

    bool shunt_enabled = (shunt_a_pin != MOTOR_PIN_UNDEFINED) && (shunt_b_pin != MOTOR_PIN_UNDEFINED);
    if (!shunt_enabled) return;

    static bool adc_initialized = false;
    if (!adc_initialized) {
        adc_init();
        adc_fifo_setup(true, true, ADC_FIFO_THRESHOLD, false, false);
        adc_initialized = true;
    }

    adc_gpio_init(ctx->shunt_a_pin);
    adc_gpio_init(ctx->shunt_b_pin);

    ctx->dma_channel_shunt = dma_claim_unused_channel(true);
    dma_channel_config dma_config = dma_channel_get_default_config(ctx->dma_channel_shunt);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_config, false);
    channel_config_set_write_increment(&dma_config, true);
    channel_config_set_dreq(&dma_config, DREQ_ADC);
    channel_config_set_ring(&dma_config, true, __builtin_ctz(SHUNT_RING_BUFFER_SIZE * sizeof(uint16_t)));

    dma_channel_configure(
        ctx->dma_channel_shunt, &dma_config,
        ctx->shunt_ring_buffer, &adc_hw->fifo,
        SHUNT_RING_BUFFER_SIZE, false
    );

    dma_channel_set_irq0_enabled(ctx->dma_channel_shunt, true);
    static bool dma_irq_initialized = false;
    if (!dma_irq_initialized) {
        irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
        irq_set_enabled(DMA_IRQ_0, true);
        dma_irq_initialized = true;
    }
}

void hal_motor_set_pwm(int duty_cycle, bool forward, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return;
    MotorContext* ctx = &g_motors[motor_id];
    if (!ctx->is_initialized) return;

    uint16_t on_level  = map(duty_cycle, PWM_DUTY_MIN, PWM_DUTY_MAX, ctx->pwm_wrap_value, 0);
    uint16_t off_level = PWM_MAX_TOP;

    ctx->next_level_a = forward ? on_level : off_level;
    ctx->next_level_b = forward ? off_level : on_level;

    // For same-slice motors, the PWM levels must be updated immediately
    // because the ISR can't handle the 'both levels' case atomically.
    if (ctx->motor_pwm_slice_a == ctx->motor_pwm_slice_b) {
        pwm_set_motor_levels(ctx, ctx->next_level_a, ctx->next_level_b);
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

int hal_motor_get_shunt_buffer(volatile uint16_t** buffer, int* last_write_pos, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return 0;
    MotorContext* ctx = &g_motors[motor_id];
    if (!ctx->is_initialized || ctx->dma_channel_shunt < 0) return 0;

    *buffer = ctx->shunt_ring_buffer;

    uintptr_t current_write_addr = (uintptr_t)dma_hw->ch[ctx->dma_channel_shunt].write_addr;
    uintptr_t buffer_start_addr = (uintptr_t)ctx->shunt_ring_buffer;
    int byte_offset = current_write_addr - buffer_start_addr;
    *last_write_pos = byte_offset / sizeof(uint16_t);

    return SHUNT_RING_BUFFER_SIZE;
}

#endif // ARDUINO_ARCH_RP2040
