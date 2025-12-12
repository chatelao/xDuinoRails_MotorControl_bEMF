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

struct MotorContext {
    bool is_initialized;
    uint8_t pwm_a_pin;
    uint8_t pwm_b_pin;
    uint8_t bemf_a_pin;
    uint8_t bemf_b_pin;

    uint motor_pwm_slice_a;
    uint motor_pwm_slice_b;
    uint16_t pwm_wrap_value;

    // DMA and ADC
    int dma_channel;
    volatile uint16_t bemf_ring_buffer[BEMF_RING_BUFFER_SIZE];
    hal_bemf_update_callback_t bemf_callback;

    // Dynamic delay for ADC trigger
    volatile uint32_t adc_trigger_delay_us;
    volatile bool skip_measurement;
};

// Static storage for motor contexts
static MotorContext g_motors[MAX_MOTORS];
// Concurrency control for the single shared ADC
static volatile bool g_adc_busy = false;

//== Static Globals for Hardware Control ==
// static uint16_t g_pwm_wrap_value; // Moved to context
// static float g_pwm_divider = 1.0f; // Local variable in init

// Forward Declarations
static void dma_irq_handler();
static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data);
static void on_pwm_wrap();

// DMA ISR: triggered when the BEMF ring buffer is full.
static void dma_irq_handler() {
    // Iterate through all initialized motors to see which one triggered the IRQ
    for (int id = 0; id < MAX_MOTORS; id++) {
        MotorContext* ctx = &g_motors[id];
        if (!ctx->is_initialized || ctx->dma_channel < 0) continue;

        // Check if this channel triggered the interrupt
        if (dma_hw->ints0 & (1u << ctx->dma_channel)) {

            // Stop the ADC immediately to prevent sampling during the ON phase.
            // Note: If multiple motors run simultaneously, stopping the ADC globally might affect others.
            // But ADC is a shared resource anyway. Round-robin is handled by the hardware if properly configured,
            // but here we are doing triggered bursts.
            // For now, we assume sequential or compatible timing, or accept that `adc_run(false)` stops all.
            adc_run(false);
            g_adc_busy = false; // Release the ADC lock

            // Clear the DMA interrupt flag for our channel
            dma_hw->ints0 = 1u << ctx->dma_channel;

            uint32_t sum_A = 0, sum_B = 0;
            // De-interleave and sum the ADC samples [B, A, B, A...]
            for (uint i = 0; i < BEMF_RING_BUFFER_SIZE; i += 2) {
                sum_B += ctx->bemf_ring_buffer[i];
                sum_A += ctx->bemf_ring_buffer[i + 1];
            }
            // Calculate the average differential BEMF.
            int measured_bemf = abs((int)(sum_A / (BEMF_RING_BUFFER_SIZE / 2)) - (int)(sum_B / (BEMF_RING_BUFFER_SIZE / 2)));

            if (ctx->bemf_callback) {
                ctx->bemf_callback(measured_bemf);
            }

            // Do NOT re-arm the DMA channel here.
            // We wait for the next PWM Wrap trigger to decide who gets the ADC.
            // If we re-arm here, the DMA channel becomes active and might race for the ADC FIFO.

            // dma_channel_set_trans_count(ctx->dma_channel, BEMF_RING_BUFFER_SIZE, false);
            // dma_channel_set_write_addr(ctx->dma_channel, ctx->bemf_ring_buffer, true);
        }
    }
}

// One-shot hardware timer callback to trigger the ADC conversion.
static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data) {
    MotorContext* ctx = (MotorContext*)user_data;
    if (!ctx) return 0;

    // Concurrency Check: If ADC is already running for another motor, skip this measurement.
    if (g_adc_busy) return 0;

#if defined(MOTOR_CURRENT_PIN)
    // --- FAST PROTECTION CHECK ---
    // Note: With multiple motors, specific current pins would need to be part of the context.
    // Assuming global MOTOR_CURRENT_PIN for now or ignoring if undefined.
    adc_select_input(MOTOR_CURRENT_PIN - 26);
    uint16_t current_val = adc_read();

    const float V_limit = MAX_CURRENT_AMPS * SHUNT_RESISTOR_OHMS;
    const uint16_t adc_threshold = (uint16_t)((V_limit / 3.3f) * 4095.0f);

    if (current_val > adc_threshold) {
        // Fast Shutdown
        pwm_set_gpio_level(ctx->pwm_a_pin, 0);
        pwm_set_gpio_level(ctx->pwm_b_pin, 0);
        return 0;
    }

    // Restore the correct input for BEMF (Start with B to maintain [B, A] order)
    adc_select_input(ctx->bemf_b_pin - 26);
#endif

    // Acquire lock
    g_adc_busy = true;

    // Select the correct input channel for this motor (Start with B for [B, A] order)
    // Note: If using Round Robin, we need to re-configure it here if multiple motors use different pairs.
    uint8_t adc_ch_a = ctx->bemf_a_pin - 26;
    uint8_t adc_ch_b = ctx->bemf_b_pin - 26;

    adc_select_input(adc_ch_b); // Start with B
    adc_set_round_robin((1u << adc_ch_a) | (1u << adc_ch_b));

    // Drain any stale data
    adc_fifo_drain();

    // Re-arm the DMA channel for THIS motor.
    dma_channel_set_trans_count(ctx->dma_channel, BEMF_RING_BUFFER_SIZE, false);
    dma_channel_set_write_addr(ctx->dma_channel, ctx->bemf_ring_buffer, true);

    // Start a single ADC conversion sequence
    adc_run(true);
    return 0;
}

// PWM Wrap ISR
static void on_pwm_wrap() {
    // Check which slice triggered.
    // Loop through motors.
    for (int id = 0; id < MAX_MOTORS; id++) {
        MotorContext* ctx = &g_motors[id];
        if (!ctx->is_initialized) continue;

        // We check slice A. (Assuming A and B are on the same slice or synced).
        if (pwm_get_irq_status_mask() & (1u << ctx->motor_pwm_slice_a)) {
            pwm_clear_irq(ctx->motor_pwm_slice_a);

            if (!ctx->skip_measurement) {
                // Pass the context to the callback
                add_alarm_in_us(ctx->adc_trigger_delay_us, delayed_adc_trigger_callback, ctx, true);
            }
        }
    }
}


// == Public HAL Function Implementations ==

void hal_motor_init(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return;

    MotorContext* ctx = &g_motors[motor_id];

    // Store Pin Config
    ctx->pwm_a_pin  = pwm_a_pin;
    ctx->pwm_b_pin  = pwm_b_pin;
    ctx->bemf_a_pin = bemf_a_pin;
    ctx->bemf_b_pin = bemf_b_pin;
    ctx->bemf_callback = callback;
    ctx->adc_trigger_delay_us = BEMF_MEASUREMENT_DELAY_US;
    ctx->skip_measurement = false;

    // --- PWM Setup ---
    gpio_set_function( ctx->pwm_a_pin, GPIO_FUNC_PWM );
    gpio_set_function( ctx->pwm_b_pin, GPIO_FUNC_PWM );
    
    gpio_set_outover(  ctx->pwm_b_pin, GPIO_OVERRIDE_INVERT);
    gpio_set_outover(  ctx->pwm_a_pin, GPIO_OVERRIDE_INVERT);

    // IMPORTANT: Set initial values to avoid unexpected start of the motor
    pwm_set_gpio_level(ctx->pwm_a_pin, 65535);
    pwm_set_gpio_level(ctx->pwm_b_pin, 65535);

    // Config calculation
    uint32_t system_clock = 125000000;
    float divider = 1.0f;
    uint32_t top = system_clock / PWM_FREQUENCY_HZ;

    if (top > 65535) {
        divider = (float)top / 65535.0f;
        if (divider > 255.0f) divider = 255.0f;
    }
    
    ctx->pwm_wrap_value = (uint16_t)(system_clock / (PWM_FREQUENCY_HZ * divider)) - 1;

    pwm_config motor_pwm_conf = pwm_get_default_config();
    pwm_config_set_clkdiv(        &motor_pwm_conf, divider );
    pwm_config_set_wrap(          &motor_pwm_conf, ctx->pwm_wrap_value );
    pwm_config_set_phase_correct( &motor_pwm_conf, true );
      
    // Apply the configuration
    ctx->motor_pwm_slice_a = pwm_gpio_to_slice_num(ctx->pwm_a_pin);
    ctx->motor_pwm_slice_b = pwm_gpio_to_slice_num(ctx->pwm_b_pin);
    
    // Set the counter of both slices to zero to run synchronous
    pwm_set_counter(ctx->motor_pwm_slice_a, 0);
    pwm_set_counter(ctx->motor_pwm_slice_b, 0);

    // Init both slices WITHOUT starting (yet)
    pwm_init(ctx->motor_pwm_slice_a, &motor_pwm_conf, false);
    pwm_init(ctx->motor_pwm_slice_b, &motor_pwm_conf, false);
    
#ifdef USE_IRQ_TRIGGER
    pwm_clear_irq(ctx->motor_pwm_slice_a);
    pwm_set_irq_enabled(ctx->motor_pwm_slice_a, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);
#endif
    
    // Start both slices exactly synchronous
    // Note: if multiple motors are on different slices, this mask only covers this motor's slices.
    uint32_t mask = (1u << ctx->motor_pwm_slice_a) | (1u << ctx->motor_pwm_slice_b);
    hw_set_bits(&pwm_hw->en, mask);
        
    // --- ADC and DMA Setup ---
    // Initialize ADC only once!
    static bool adc_initialized = false;
    if (!adc_initialized) {
        adc_init();
        adc_fifo_setup(true, true, 1, false, false);
        adc_initialized = true;
    }
    
    adc_gpio_init(ctx->bemf_a_pin);
    adc_gpio_init(ctx->bemf_b_pin);

    // DMA Channel
    ctx->dma_channel = dma_claim_unused_channel(true);
    dma_channel_config dma_config = dma_channel_get_default_config(ctx->dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16); 
    channel_config_set_read_increment(&dma_config, false);           
    channel_config_set_write_increment(&dma_config, true);           
    channel_config_set_dreq(&dma_config, DREQ_ADC);                  

    channel_config_set_ring(&dma_config, true, __builtin_ctz(BEMF_RING_BUFFER_SIZE * sizeof(uint16_t)));

    dma_channel_configure(ctx->dma_channel, &dma_config, ctx->bemf_ring_buffer, &adc_hw->fifo, BEMF_RING_BUFFER_SIZE, false);

    // Setup DMA IRQ (Shared handler)
    dma_channel_set_irq0_enabled(ctx->dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // DO NOT Start DMA here. Wait for trigger.
    // dma_channel_set_write_addr(ctx->dma_channel, ctx->bemf_ring_buffer, true);
    
    ctx->is_initialized = true;
}

void hal_motor_set_pwm(int duty_cycle, bool forward, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return;
    MotorContext* ctx = &g_motors[motor_id];
    if (!ctx->is_initialized) return;

    // Map the 8-bit duty cycle (0-255) to the PWM counter's range.
    uint16_t level = map(duty_cycle, 0, 255, ctx->pwm_wrap_value, 0);

    if (forward) {
       pwm_set_gpio_level(ctx->pwm_a_pin, level);
       pwm_set_gpio_level(ctx->pwm_b_pin, 0); // OFF
    } else {
       pwm_set_gpio_level(ctx->pwm_a_pin, 0); // OFF
       pwm_set_gpio_level(ctx->pwm_b_pin, level);
   }
}

int hal_motor_get_bemf_buffer(volatile uint16_t** buffer, int* last_write_pos, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return 0;
    MotorContext* ctx = &g_motors[motor_id];
    if (!ctx->is_initialized) return 0;

    *buffer = ctx->bemf_ring_buffer;

    uintptr_t current_write_addr = (uintptr_t)dma_hw->ch[ctx->dma_channel].write_addr;
    uintptr_t buffer_start_addr = (uintptr_t)ctx->bemf_ring_buffer;
    int byte_offset = current_write_addr - buffer_start_addr;

    *last_write_pos = byte_offset / sizeof(uint16_t);

    return BEMF_RING_BUFFER_SIZE;
}

int hal_motor_get_current_buffer(volatile uint16_t** buffer, int* last_write_pos, uint8_t motor_id) {
    *buffer         = nullptr;
    *last_write_pos = 0;
    return 0;
}

#endif // ARDUINO_ARCH_RP2040
