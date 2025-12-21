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

#define DEBUG_SERIAL Serial
#define RING_BUFFER_SIZE 16

// A single source value that we will update in the loop.
// The DMA will copy this value into the ring buffer repeatedly.
volatile uint16_t source_value = 0;

// The destination ring buffer, initialized to 0xFFFF so we can see changes.
volatile uint16_t ring_buffer[RING_BUFFER_SIZE];

// The DMA channel we're using.
int dma_channel;

void setup() {
    DEBUG_SERIAL.begin(115200);
    // A simple delay is sufficient for the monitor to connect.
    delay(2000);
    DEBUG_SERIAL.println("\n--- DMA Ring Buffer Example ---");

    // Initialize buffer with a known value.
    for (int i = 0; i < RING_BUFFER_SIZE; i++) {
        ring_buffer[i] = 0xFFFF;
    }

    // --- DMA Configuration ---
    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_channel);

    channel_config_set_transfer_data_size( &cfg, DMA_SIZE_16); // SIZE  - Configure 16-bit transfers.
    channel_config_set_read_increment(     &cfg, false);       // READ  - Keep the address fix.
    channel_config_set_write_increment(    &cfg, true);        // WRITE - Auto-Increment the address.
    channel_config_set_ring(               &cfg, true, 5);     // WRITE - true = "ring for WRITE ", ring size = 2^5 = 32 bytes

    // Configure the channel, but don't start it yet.
    // It will be triggered manually in the loop.
    // We set the transfer count to a large number, but we will re-trigger
    // manually in the loop.
    dma_channel_configure(
        dma_channel,
        &cfg,
        ring_buffer,       // Dst:   Start of our ring buffer
        &source_value,     // Src:   A single value
        0xFFFFFFFF,        // Time:  Run for a long time
        false              // Start: Don't start yet
    );

    DEBUG_SERIAL.println("DMA configured. Manually triggering in loop().");
}

void loop() {
    
    // 1. Update the source value with something new.
    source_value++;

    // 2. Manually trigger the DMA to do exactly ONE transfer.
    // This is the key to ensuring a single, discrete operation. We set the
    // transfer count to 1 and then immediately trigger the transfer.
    dma_channel_set_trans_count(dma_channel, 1, true);

    // 3. Wait for the transfer to complete.
    // This is crucial for synchronization and ensures the data is in the
    // buffer before we try to read and print it.
    dma_channel_wait_for_finish_blocking(dma_channel);

    // 4. Print the state.
    uint32_t current_write_addr = dma_channel_get_write_addr(dma_channel);
    int last_write_pos = ((current_write_addr - (uint32_t)ring_buffer) / sizeof(uint16_t));

    // The write address points to the NEXT location, so the last written element
    // is at the previous index. We handle the wrap-around with modulo.
    int last_written_index = (last_write_pos - 1 + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;

    DEBUG_SERIAL.print("Trigger ");
    DEBUG_SERIAL.print(source_value);
    DEBUG_SERIAL.print(": Copied ");
    DEBUG_SERIAL.print(ring_buffer[last_written_index]);
    DEBUG_SERIAL.print(" to index ");
    DEBUG_SERIAL.print(last_written_index);
    DEBUG_SERIAL.print(". Buffer: [");

    for (int i = 0; i < RING_BUFFER_SIZE; i++) {
        DEBUG_SERIAL.print(ring_buffer[i]);
        if (i < RING_BUFFER_SIZE - 1) DEBUG_SERIAL.print(", ");
    }
    DEBUG_SERIAL.println("]");

    delay(250); // Slow down the loop to make the output readable.
}
