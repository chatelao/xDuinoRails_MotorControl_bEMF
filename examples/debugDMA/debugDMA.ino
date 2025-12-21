/**
 * @file debugDMA.ino
 * @brief Minimalistic example for RP2040 DMA operations.
 *
 * This sketch demonstrates the core concept of using a DMA channel to transfer
 * data from a source variable into a destination ring buffer.
 *
 * It showcases:
 * a) How to configure a DMA channel for repeated transfers.
 * b) How to set up a ring buffer on the destination side.
 * c) How to manually trigger a single DMA transfer, simulating an
 *    external trigger (like one from a PWM or ADC peripheral) in a simple way.
 *
 * Instead of complex hardware triggers, this example uses a manual,
 * software-based trigger in the main loop to keep the focus purely on the
 * DMA data movement. The contents of the ring buffer are printed to the
 * Serial monitor to visualize the DMA's operation.
 */

#include <Arduino.h>
#include <hardware/dma.h>
#include <hardware/pwm.h>
#include <hardware/adc.h>

#define DEBUG_SERIAL Serial
#define RING_BUFFER_SIZE 16
#define ADC_PIN 26 // ADC0

// --- Manual Trigger DMA ---
volatile uint16_t manual_source_value = 0;
volatile uint16_t manual_ring_buffer[RING_BUFFER_SIZE];
int manual_dma_channel;

// --- PWM Trigger DMA ---
volatile uint16_t pwm_source_value = 0xAAAA;
volatile uint16_t pwm_ring_buffer[RING_BUFFER_SIZE];
int pwm_dma_channel;
uint pwm_slice_num;

// --- ADC Trigger DMA ---
volatile uint16_t adc_ring_buffer[RING_BUFFER_SIZE];
int adc_dma_channel;


void setup() {
    DEBUG_SERIAL.begin(115200);
    delay(2000);
    DEBUG_SERIAL.println("\n--- DMA Hardware Trigger Example ---");

    // Initialize all buffers with a known value to see changes.
    for (int i = 0; i < RING_BUFFER_SIZE; i++) {
        manual_ring_buffer[i] = 0xFFFF;
        pwm_ring_buffer[i]    = 0xFFFF;
        adc_ring_buffer[i]    = 0xFFFF;
    }

    // --- Manual DMA Configuration (Original) ---
    manual_dma_channel = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(manual_dma_channel);

    channel_config_set_transfer_data_size( &cfg, DMA_SIZE_16); // SIZE  - Configure 16-bit transfers.
    channel_config_set_read_increment(     &cfg, false);       // READ  - Keep the address fix.
    channel_config_set_write_increment(    &cfg, true);        // WRITE - Auto-Increment the address.
    channel_config_set_ring(               &cfg, true, 5);     // WRITE - true = "ring for WRITE ", ring size = 2^5 = 32 bytes

    // Configure the channel, but don't start it yet.
    // It will be triggered manually in the loop.
    // We set the transfer count to a large number, but we will re-trigger
    // manually in the loop.
    dma_channel_configure(
        manual_dma_channel,
        &cfg,
        manual_ring_buffer, // Dst:   Start of our ring buffer
        &manual_source_value, // Src:   A single value
        0xFFFFFFFF,        // Time:  Run for a long time
        false              // Start: Don't start yet
    );

    // --- PWM-Triggered DMA Configuration ---
    pwm_dma_channel = dma_claim_unused_channel(true);
    dma_channel_config pwm_dma_cfg = dma_channel_get_default_config(pwm_dma_channel);
    channel_config_set_transfer_data_size(&pwm_dma_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&pwm_dma_cfg, false);
    channel_config_set_write_increment(&pwm_dma_cfg, true);
    channel_config_set_ring(&pwm_dma_cfg, true, 5); // 2^5 = 32 bytes

    // --- ADC-Triggered DMA Configuration ---
    adc_dma_channel = dma_claim_unused_channel(true);
    dma_channel_config adc_dma_cfg = dma_channel_get_default_config(adc_dma_channel);
    channel_config_set_transfer_data_size(&adc_dma_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&adc_dma_cfg, false); // Read from the same ADC FIFO address
    channel_config_set_write_increment(&adc_dma_cfg, true);
    channel_config_set_ring(&adc_dma_cfg, true, 5); // 2^5 = 32 bytes

    // --- PWM Peripheral Configuration ---
    // We need a PWM running to generate DREQ signals.
    const uint pwm_pin = PICO_DEFAULT_LED_PIN; // Or any other GPIO
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    pwm_slice_num = pwm_gpio_to_slice_num(pwm_pin);

    // Configure and enable the PWM slice.
    // It will run freely, and its DREQ signal will pace the PWM-triggered DMA.
    pwm_config pwm_cfg = pwm_get_default_config();
    pwm_config_set_wrap(&pwm_cfg, 255);
    pwm_init(pwm_slice_num, &pwm_cfg, true); // `true` starts the PWM.
    DEBUG_SERIAL.println("PWM configured and running.");

    // --- ADC Peripheral Configuration ---
    // We will use the ADC in free-running mode to generate DREQ signals.
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0); // ADC0 is on GPIO 26

    // Enable ADC FIFO and set DREQ threshold.
    // A DREQ is generated when the FIFO has at least 1 sample.
    adc_fifo_setup(
        true,    // Write ADC results to FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ asserted when FIFO contains at least 1 sample
        false,   // Don't set error bit on overflow
        false    // Don't simplify data to 8 bits
    );
    
    // Set the ADC to free-running mode. The ADC will continuously sample
    // and fill the FIFO, triggering the DMA.
    adc_run(true);
    DEBUG_SERIAL.println("ADC configured and running in free-run mode.");

    // --- Finalize DMA Channel Setups & Start ---

    // Link PWM DREQ to the PWM DMA channel and start it.
    channel_config_set_dreq(&pwm_dma_cfg, pwm_get_dreq(pwm_slice_num));
    dma_channel_configure(
        pwm_dma_channel,
        &pwm_dma_cfg,
        pwm_ring_buffer,      // Dst
        &pwm_source_value,    // Src
        0xFFFFFFFF,           // Count
        true                  // Start immediately
    );

    // Link ADC DREQ to the ADC DMA channel and start it.
    channel_config_set_dreq(&adc_dma_cfg, DREQ_ADC);
    dma_channel_configure(
        adc_dma_channel,
        &adc_dma_cfg,
        adc_ring_buffer,      // Dst
        &adc_hw->fifo,        // Src
        0xFFFFFFFF,           // Count
        true                  // Start immediately
    );

    DEBUG_SERIAL.println("All peripherals configured. Monitoring in loop().");
}

// Helper function to print the contents of a ring buffer.
void print_ring_buffer(const char* name, int dma_chan, volatile uint16_t* buffer) {
    uint32_t current_write_addr = dma_channel_get_write_addr(dma_chan);
    int current_index = ((current_write_addr - (uint32_t)buffer) / sizeof(uint16_t));

    DEBUG_SERIAL.print(name);
    DEBUG_SERIAL.print(" (DMA ");
    DEBUG_SERIAL.print(dma_chan);
    DEBUG_SERIAL.print(", Idx ");
    DEBUG_SERIAL.print(current_index);
    DEBUG_SERIAL.print("): [");

    for (int i = 0; i < RING_BUFFER_SIZE; i++) {
        // Print in HEX for ADC/PWM values for better readability.
        if (buffer == adc_ring_buffer || buffer == pwm_ring_buffer) {
            char hex_buf[5];
            sprintf(hex_buf, "%04X", buffer[i]);
            DEBUG_SERIAL.print(hex_buf);
        } else {
            DEBUG_SERIAL.print(buffer[i]);
        }
        if (i < RING_BUFFER_SIZE - 1) DEBUG_SERIAL.print(", ");
    }
    DEBUG_SERIAL.println("]");
}


void loop() {
    // --- Manual DMA Operation ---
    // The manual DMA still needs to be triggered explicitly.
    manual_source_value++;
    dma_channel_set_trans_count(manual_dma_channel, 1, true);
    dma_channel_wait_for_finish_blocking(manual_dma_channel);

    // --- Monitoring ---
    // The PWM and ADC DMAs run automatically in the background.
    // We just print their state here.
    DEBUG_SERIAL.println("--- Loop Update ---");
    print_ring_buffer("Manual", manual_dma_channel, manual_ring_buffer);
    print_ring_buffer("PWM",    pwm_dma_channel,    pwm_ring_buffer);
    print_ring_buffer("ADC",    adc_dma_channel,    adc_ring_buffer);

    delay(500); // Slow down the output.
}
