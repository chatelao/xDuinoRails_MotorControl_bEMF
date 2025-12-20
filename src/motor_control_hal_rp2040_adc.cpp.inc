#include "motor_control_hal_rp2040_internal.h"

#if defined(ARDUINO_ARCH_RP2040)

// =============================================================================
// Internal Functions
// =============================================================================
static void adc_init_common() {
    static bool adc_initialized = false;
    if (!adc_initialized) {
        adc_init();
        adc_fifo_setup(true, true, ADC_FIFO_THRESHOLD, false, false);
        adc_initialized = true;
    }
}

static void trigger_adc_measurement(MotorContext* ctx, MeasurementType type) {
    if (!ctx) return;

    if (g_adc_busy) return;

    g_adc_busy = true;

    if (type == BEMF) {
        uint8_t adc_ch_a = ctx->bemf_a_pin - MOTOR_ADC_BASE_PIN;
        uint8_t adc_ch_b = ctx->bemf_b_pin - MOTOR_ADC_BASE_PIN;
        adc_select_input(adc_ch_b);
        adc_set_round_robin((1u << adc_ch_a) | (1u << adc_ch_b));
        adc_fifo_drain();

        dma_channel_set_trans_count(ctx->dma_channel_bemf, BEMF_RING_BUFFER_SIZE, false);
        dma_channel_set_write_addr(ctx->dma_channel_bemf, ctx->bemf_ring_buffer, true);

    } else if (type == SHUNT) {
        uint8_t adc_ch_a = ctx->shunt_a_pin - MOTOR_ADC_BASE_PIN;
        uint8_t adc_ch_b = ctx->shunt_b_pin - MOTOR_ADC_BASE_PIN;
        adc_select_input(adc_ch_b);
        adc_set_round_robin((1u << adc_ch_a) | (1u << adc_ch_b));
        adc_fifo_drain();

        dma_channel_set_trans_count(ctx->dma_channel_shunt, SHUNT_RING_BUFFER_SIZE, false);
        dma_channel_set_write_addr(ctx->dma_channel_shunt, ctx->shunt_ring_buffer, true);
    }

    adc_run(true);
}

static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data) {
    AlarmUserData* data = (AlarmUserData*)user_data;
    trigger_adc_measurement(data->ctx, data->type);
    return 0;
}

static void dma_irq_handler() {
    for (int id = 0; id < MAX_MOTORS; id++) {
        MotorContext* ctx = &g_motors[id];
        if (!ctx->is_initialized) continue;

        if (ctx->dma_channel_bemf >= 0 && (dma_hw->ints0 & (1u << ctx->dma_channel_bemf))) {

            adc_run(false);
            g_adc_busy = false;

            dma_hw->ints0 = 1u << ctx->dma_channel_bemf;

            uint32_t sum_A = 0, sum_B = 0;

            for (uint i = 0; i < BEMF_RING_BUFFER_SIZE; i += 2) {
                sum_B += ctx->bemf_ring_buffer[i];
                sum_A += ctx->bemf_ring_buffer[i + 1];
            }

            int count = BEMF_RING_BUFFER_SIZE / 2;
            int measured_bemf = abs((int)(sum_A / count) - (int)(sum_B / count));

            if (ctx->bemf_callback) {
                ctx->bemf_callback(measured_bemf);
            }
        }

        if (ctx->dma_channel_shunt >= 0 && (dma_hw->ints0 & (1u << ctx->dma_channel_shunt))) {
            adc_run(false);
            g_adc_busy = false;

            dma_hw->ints0 = 1u << ctx->dma_channel_shunt;

            uint32_t sum_A = 0, sum_B = 0;
            for (uint i = 0; i < SHUNT_RING_BUFFER_SIZE; i += 2) {
                sum_A += ctx->shunt_ring_buffer[i];
                sum_B += ctx->shunt_ring_buffer[i+1];
            }

            int count = SHUNT_RING_BUFFER_SIZE / 2;
            int measured_shunt = abs((int)(sum_A / count) - (int)(sum_B / count));

            if (ctx->shunt_callback) {
                ctx->shunt_callback(measured_shunt);
            }
        }
    }
}

#endif // ARDUINO_ARCH_RP2040
