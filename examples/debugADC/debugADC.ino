#include <Arduino.h>
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

// =============================================================================
// Constants
// =============================================================================
#define ADC_RING_BUFFER_SIZE 4
#if defined(ARDUINO_SEEED_XIAO_RP2040)
  const uint ADC_PIN_A = D0; // Corresponds to GPIO26 for XIAO RP2040
  const uint ADC_PIN_B = D1; // Corresponds to GPIO27 for XIAO RP2040
#else
  const uint ADC_PIN_A = D7; // Corresponds to GPIO28
  const uint ADC_PIN_B = D8; // Corresponds to GPIO27
#endif
const uint MOTOR_ADC_BASE_PIN = 26;

// =============================================================================
// Global Variables
// =============================================================================
volatile uint16_t adc_ring_buffer[ADC_RING_BUFFER_SIZE];
int dma_channel;
volatile bool g_adc_data_ready = false;

// =============================================================================
// DMA Interrupt Handler
// =============================================================================
void dma_irq_handler() {
    if (dma_hw->ints0 & (1u << dma_channel)) {
        dma_hw->ints0 = 1u << dma_channel; // Clear the interrupt
        g_adc_data_ready = true;
        adc_run(false); // Stop ADC
    }
}

// =============================================================================
// ADC & DMA Setup
// =============================================================================
void setup_adc_dma() {
    // 1. ADC Initialization
    adc_init();
    adc_gpio_init(ADC_PIN_A);
    adc_gpio_init(ADC_PIN_B);

    adc_set_temp_sensor_enabled(false);
    adc_fifo_setup(
        true,  // Write ADC readings to FIFO
        true,  // Enable DMA data request (DREQ)
        1,     // DREQ asserted when FIFO contains at least 1 sample
        false, // Disable error bit
        false  // Don't right-shift samples to 8 bits
    );

    // 2. DMA Channel Configuration
    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_channel);

    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_ADC);

    dma_channel_configure(
        dma_channel,
        &cfg,
        adc_ring_buffer,    // Write address
        &adc_hw->fifo,      // Read address
        ADC_RING_BUFFER_SIZE, // Number of transfers
        false               // Don't start immediately
    );

    // 3. DMA Interrupt Setup
    dma_channel_set_irq0_enabled(dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}

// =============================================================================
// Arduino Sketch
// =============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    Serial.println("ADC Debug Example Started");

    setup_adc_dma();

    // Select initial ADC channel
    uint8_t adc_ch_a = ADC_PIN_A - MOTOR_ADC_BASE_PIN;
    uint8_t adc_ch_b = ADC_PIN_B - MOTOR_ADC_BASE_PIN;
    adc_select_input(adc_ch_b);
    adc_set_round_robin((1u << adc_ch_a) | (1u << adc_ch_b));
}

void loop() {
    if (g_adc_data_ready) {
        g_adc_data_ready = false;

        uint32_t sum_A = 0, sum_B = 0;
        int count = ADC_RING_BUFFER_SIZE / 2;

        // The ADC samples channels in a round-robin fashion.
        // Even indices will be channel B, odd indices will be channel A.
        for (uint i = 0; i < ADC_RING_BUFFER_SIZE; i += 2) {
            sum_B += adc_ring_buffer[i];
            sum_A += adc_ring_buffer[i + 1];
        }

        float avg_A = (float)sum_A / count;
        float avg_B = (float)sum_B / count;

        Serial.print("Avg A: ");
        Serial.print(avg_A);
        Serial.print(", Avg B: ");
        Serial.println(avg_B);

        delay(500); // Wait before starting the next measurement

    } else {
        // If no data is ready, trigger a new measurement
        adc_fifo_drain();
        dma_channel_set_write_addr(dma_channel, adc_ring_buffer, true);
        dma_channel_set_trans_count(dma_channel, ADC_RING_BUFFER_SIZE, false);
        adc_run(true);
    }
}
