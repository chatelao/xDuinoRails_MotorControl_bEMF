/**
 * @file motor_control_hal_rp2040.cpp
 * @brief RP2040-specific implementation for the motor control HAL.
 *
 * This file provides the concrete implementation of the functions defined in
 * motor_control_hal.h for the Raspberry Pi RP2040 microcontroller. It uses
 * the Pico-SDK's hardware peripherals (PWM, ADC, DMA) for efficient,
 * non-blocking motor control and BEMF measurement.
 */

#include "motor_control_hal.h"

#if defined(ARDUINO_ARCH_RP2040)

#include <Arduino.h>
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/irq.h"

//== Hardware PWM & BEMF Measurement Parameters ==

//== Static Globals for Hardware Control ==
static uint16_t g_pwm_wrap_value;
static float g_pwm_divider = 1.0f;
static uint dma_channel;     // DMA channel for ADC->memory transfers
static uint motor_pwm_slice;     // The RP2040 PWM slice driving the motor
static uint g_motor_pwm_slice_a; // The RP2040 PWM slice(s) driving the motor
static uint g_motor_pwm_slice_b; // The RP2040 PWM slice(s) driving the motor

// Volatile is required as this buffer is written by DMA and read by the CPU in an ISR.
static volatile uint16_t bemf_ring_buffer[BEMF_RING_BUFFER_SIZE];
static hal_bemf_update_callback_t bemf_callback = nullptr;
static uint8_t g_pwm_a_pin; // Motor PWM A pin
static uint8_t g_pwm_b_pin; // Motor PWM B pin
static uint8_t g_bemf_a_pin; // BEMF A pin
static uint8_t g_bemf_b_pin; // BEMF B pin

// Dynamic delay for ADC trigger, calculated based on duty cycle to ensure sampling during OFF phase.
static volatile uint32_t g_adc_trigger_delay_us = BEMF_MEASUREMENT_DELAY_US;
static volatile bool g_skip_measurement = false;

// Forward Declarations
static void dma_irq_handler();
static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data);
static void on_pwm_wrap();

// DMA ISR: triggered when the BEMF ring buffer is full.
// This is the core of the measurement, processing raw ADC data and passing it to the controller.
static void dma_irq_handler() {
    // Stop the ADC immediately to prevent sampling during the ON phase.
    adc_run(false);

    // Clear the DMA interrupt flag for our channel using a bitmask.
    dma_hw->ints0 = 1u << dma_channel;

    uint32_t sum_A = 0, sum_B = 0;
    // De-interleave and sum the ADC samples for each motor terminal (B, A, B, A, ...).
    for (uint i = 0; i < BEMF_RING_BUFFER_SIZE; i += 2) {
        sum_B += bemf_ring_buffer[i];
        sum_A += bemf_ring_buffer[i + 1];
    }
    // Calculate the average differential BEMF.
    int measured_bemf = abs((int)(sum_A / (BEMF_RING_BUFFER_SIZE / 2)) - (int)(sum_B / (BEMF_RING_BUFFER_SIZE / 2)));

    if (bemf_callback) {
        bemf_callback(measured_bemf);
    }
    // Restart DMA transfer to refill the buffer for the next cycle.
    dma_channel_set_trans_count(dma_channel, BEMF_RING_BUFFER_SIZE, false);
    dma_channel_set_write_addr(dma_channel, bemf_ring_buffer, true);
}

// One-shot hardware timer callback to trigger the ADC conversion.
// This is called after the BEMF_MEASUREMENT_DELAY_US to ensure a stable reading.
static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data) {
#if defined(MOTOR_CURRENT_PIN)
    // --- FAST PROTECTION CHECK ---
    // Manually read the current pin *before* starting the BEMF sequence.
    // This allows us to check for overcurrent without messing up the BEMF DMA buffer.
    adc_select_input(MOTOR_CURRENT_PIN - 26);
    uint16_t current_val = adc_read();

    const float V_limit = MAX_CURRENT_AMPS * SHUNT_RESISTOR_OHMS;
    const uint16_t adc_threshold = (uint16_t)((V_limit / 3.3f) * 4095.0f);

    if (current_val > adc_threshold) {
        // Fast Shutdown
#ifdef LED_EDITION
        pwm_set_gpio_level(g_pwm_a_pin, g_pwm_wrap_value);
        pwm_set_gpio_level(g_pwm_b_pin, g_pwm_wrap_value);
#else
        pwm_set_gpio_level(g_pwm_a_pin, 0);
        pwm_set_gpio_level(g_pwm_b_pin, 0);
#endif
        // Do not proceed to BEMF measurement
        return 0;
    }

    // Restore the correct input for BEMF (Start with B to maintain [B, A] order)
    adc_select_input(g_bemf_b_pin - 26);
#endif

    // Drain any stale data from the FIFO before starting a new burst.
    adc_fifo_drain();

    // Start a single ADC conversion sequence that will run until the DMA buffer is full.
    adc_run(true);
    return 0; // Returning 0 prevents the timer from rescheduling.
}

// PWM Wrap ISR: synchronizes ADC measurement with the PWM cycle.
// This is triggered at the end of each PWM cycle.
static void on_pwm_wrap() {
    pwm_clear_irq(motor_pwm_slice);
    // Schedule the ADC trigger to run after the calculated delay.
    // We only trigger if the delay point falls within the OFF phase of the cycle.
    if (!g_skip_measurement) {
        add_alarm_in_us(g_adc_trigger_delay_us, delayed_adc_trigger_callback, NULL, true);
    }
}

// == Public HAL Function Implementations ==

void hal_motor_init(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback) {

    g_pwm_a_pin = pwm_a_pin;
    g_pwm_b_pin = pwm_b_pin;
    g_bemf_a_pin = bemf_a_pin;
    g_bemf_b_pin = bemf_b_pin;

    // --- PWM Setup ---
    gpio_set_function(g_pwm_a_pin, GPIO_FUNC_PWM);
    gpio_set_function(g_pwm_b_pin, GPIO_FUNC_PWM);

    // Wichtig: Initialwerte setzen, um unerwartetes Anlaufen zu verhindern
    pwm_set_gpio_level(g_pwm_a_pin, 0);
    pwm_set_gpio_level(g_pwm_b_pin, 0);

    // Config calculation (Original logic preserved, looks okay for standard use)
    uint32_t system_clock = 125000000;
    float divider = 1.0f;
    uint32_t top = system_clock / PWM_FREQUENCY_HZ;

    if (top > 65535) {
        divider = (float)top / 65535.0f;
        if (divider > 255.0f) divider = 255.0f;
    }
    
    g_pwm_divider = divider;
    g_pwm_wrap_value = (uint16_t)(system_clock / (PWM_FREQUENCY_HZ * divider)) - 1;

    pwm_config motor_pwm_conf = pwm_get_default_config();
    pwm_config_set_clkdiv(&motor_pwm_conf, g_pwm_divider);
    pwm_config_set_wrap(  &motor_pwm_conf, g_pwm_wrap_value);
      
    // Apply the configuration
    g_motor_pwm_slice_a = pwm_gpio_to_slice_num(g_pwm_a_pin);
    g_motor_pwm_slice_b = pwm_gpio_to_slice_num(g_pwm_b_pin);
    pwm_init(g_motor_pwm_slice_a, &motor_pwm_conf, true);
    pwm_init(g_motor_pwm_slice_b, &motor_pwm_conf, true);

    /*
    bemf_callback = callback;

    // --- Safety Checks ---
    // Sicherstellen, dass beide PWM Pins auf demselben Slice liegen
    assert(pwm_gpio_to_slice_num(pwm_a_pin) == pwm_gpio_to_slice_num(pwm_b_pin));

    // --- ADC and DMA Setup ---
    adc_init();
    adc_gpio_init(bemf_a_pin);
    adc_gpio_init(bemf_b_pin);

    // Round Robin Setup
    // KORREKTUR: Input Select sollte deterministisch auf den niedrigeren Kanal gesetzt werden,
    // damit die Sequenz immer [Low_Ch, High_Ch, Low_Ch...] ist und nicht zufällig startet.
    uint8_t adc_ch_a = bemf_a_pin - 26;
    uint8_t adc_ch_b = bemf_b_pin - 26;
    adc_select_input(adc_ch_a < adc_ch_b ? adc_ch_a : adc_ch_b);
    
    adc_set_round_robin((1u << adc_ch_a) | (1u << adc_ch_b));
    
    // FIFO leeren, um Kanalverschiebung zu vermeiden
    adc_fifo_drain();
    adc_fifo_setup(true, true, 1, false, false);

    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16); 
    channel_config_set_read_increment(&dma_config, false);           
    channel_config_set_write_increment(&dma_config, true);           
    channel_config_set_dreq(&dma_config, DREQ_ADC);                  

    // KORREKTUR 2: Ring-Buffer Größe in Bytes berechnen!
    // __builtin_ctz liefert trailing zeros. 
    // Bei 1024 Elementen (uint16) sind es 2048 Bytes. log2(2048) = 11.
    channel_config_set_ring(&dma_config, true, __builtin_ctz(BEMF_RING_BUFFER_SIZE * sizeof(uint16_t)));

    dma_channel_configure(dma_channel, &dma_config, bemf_ring_buffer, &adc_hw->fifo, BEMF_RING_BUFFER_SIZE, false);

    // KORREKTUR 3: IRQ Handling
    // Exclusive Handler ist okay, wenn du die volle Kontrolle hast. 
    // Shared Handler wäre sicherer in großen Systemen.
    dma_channel_set_irq0_enabled(dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // --- Sync & Start ---
    // Start DMA first -> Waiting for DREQ
    dma_channel_set_write_addr(dma_channel, bemf_ring_buffer, true);
    
    // ADC starten (Free Running - oder besser: via PWM getriggert in späterer Ausbaustufe)
    adc_run(true); 
    */
}

void hal_motor_set_pwm(int duty_cycle, bool forward) {

    // Map the 8-bit duty cycle (0-255) to the PWM counter's range.
    uint16_t level = map(duty_cycle, 0, 255, 0, g_pwm_wrap_value);

/*
    // Calculate timing for BEMF measurement.
    // The PWM Wrap interrupt fires at the start of the ON phase (Counter = 0).
    // The Falling Edge (transition to OFF phase) occurs at `level`.
    // We want to sample after the Falling Edge + Settling Time.

    // Convert settling time from microseconds to PWM ticks.
    // 1 tick = g_pwm_divider / 125MHz.
    // So 1 us = 125 / g_pwm_divider ticks.
    uint32_t settling_ticks = (BEMF_MEASUREMENT_DELAY_US * 125) / g_pwm_divider;
    uint32_t trigger_tick_pos = level + settling_ticks;

    // If the sampling point extends beyond the PWM period (g_pwm_wrap_value),
    // it means the OFF phase is too short or non-existent (high duty cycle).
    // In this case, we skip measurement to avoid sampling during the next ON phase.
    if (trigger_tick_pos >= g_pwm_wrap_value) {
        g_skip_measurement = true;
    } else {
        g_skip_measurement = false;
        // Convert the total delay (from Wrap IRQ) back to microseconds for add_alarm_in_us.
        // Time = trigger_tick_pos * (g_pwm_divider / 125)
        g_adc_trigger_delay_us = (uint32_t)(trigger_tick_pos * g_pwm_divider / 125);
    }
*/

    // For Active Low LEDs (Seeed XIAO RP2040 LED Edition), logic is inverted:
    // - High (g_pwm_wrap_value) = LED OFF.
    // - Low (0) = LED ON (Full Brightness).
    // - level is the ON time (Low time).
    if (forward) {
       pwm_set_gpio_level(g_pwm_a_pin, g_pwm_wrap_value - level);
       pwm_set_gpio_level(g_pwm_b_pin, g_pwm_wrap_value - 1); // OFF
    } else {
       pwm_set_gpio_level(g_pwm_a_pin, g_pwm_wrap_value - 1); // OFF
       pwm_set_gpio_level(g_pwm_b_pin, g_pwm_wrap_value - level);
   }
}

int hal_motor_get_bemf_buffer(volatile uint16_t** buffer, int* last_write_pos) {
    *buffer = bemf_ring_buffer;

    // Get the current write address from the DMA controller.
    // We access the register directly because dma_channel_get_write_addr() is not available in all SDK versions.
    uintptr_t current_write_addr = (uintptr_t)dma_hw->ch[dma_channel].write_addr;
    // Calculate the offset from the beginning of the buffer.
    uintptr_t buffer_start_addr = (uintptr_t)bemf_ring_buffer;
    int byte_offset = current_write_addr - buffer_start_addr;

    // Convert the byte offset to a sample index.
    *last_write_pos = byte_offset / sizeof(uint16_t);

    return BEMF_RING_BUFFER_SIZE;
}

int hal_motor_get_current_buffer(volatile uint16_t** buffer, int* last_write_pos) {
    *buffer         = nullptr;
    *last_write_pos = 0;
    return 0;
}

#endif // ARDUINO_ARCH_RP2040
