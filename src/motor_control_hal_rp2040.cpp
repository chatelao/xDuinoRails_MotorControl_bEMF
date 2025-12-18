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

// =============================================================================
// Constants & Magic Values
// =============================================================================

// The RP2040 system clock is typically 125 MHz.
const uint32_t RP2040_SYSTEM_CLOCK_HZ = 125000000;

// ADC Configuration
const int      ADC_RESOLUTION_BITS    = 12;
const float    ADC_MAX_VALUE          = 4095.0f; // 2^12 - 1
const float    ADC_REF_VOLTAGE        = 3.3f;    // Standard RP2040 logic voltage
const uint     MOTOR_ADC_BASE_PIN     = 26;      // GPIO 26 is ADC0
const uint     ADC_FIFO_THRESHOLD     = 1;       // Number of samples to trigger DREQ

// PWM Configuration
const uint16_t PWM_MAX_TOP            = 65535;   // 16-bit counter max value
const float    PWM_MAX_DIVIDER        = 255.0f;  // 8-bit integer + 4-bit fractional divider
const int      PWM_DUTY_MIN           = 0;       // Minimum input duty cycle (0%)
const int      PWM_DUTY_MAX           = 255;     // Maximum input duty cycle (100%)
const uint16_t PWM_DEAD_TIME_CYCLES   = 50;      // Dead time in system clock cycles (~400ns at 125MHz)

// =============================================================================
// Data Structures
// =============================================================================

struct MotorContext {
    bool is_initialized;
    bool is_discrete_mode; // True for 4-pin H-Bridge

    // Pins
    uint8_t pwm_a_pin; // Acts as HS_A in discrete mode
    uint8_t pwm_b_pin; // Acts as HS_B in discrete mode
    // Discrete Mode Only Pins
    uint8_t ls_a_pin;
    uint8_t ls_b_pin;

    uint8_t bemf_a_pin;
    uint8_t bemf_b_pin;

    uint motor_pwm_slice_a;
    uint motor_pwm_slice_b;
    uint16_t pwm_wrap_value;

    // DMA and ADC
    int dma_channel;
    // Buffer to store raw ADC samples transferred via DMA.
    // Size is defined in motor_control_hal.h
    volatile uint16_t bemf_ring_buffer[BEMF_RING_BUFFER_SIZE];
    hal_bemf_update_callback_t bemf_callback;

    // Dynamic delay for ADC trigger (calculated based on duty cycle)
    volatile uint32_t adc_trigger_delay_us;
    volatile bool skip_measurement;
};

// Static storage for motor contexts
static MotorContext g_motors[MAX_MOTORS];
// Concurrency control for the single shared ADC peripheral
static volatile bool g_adc_busy = false;

// =============================================================================
// Forward Declarations
// =============================================================================

static void dma_irq_handler();
static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data);
static void on_pwm_wrap();
static void common_motor_init(MotorContext* ctx);

// =============================================================================
// Internal Helper Functions & ISRs
// =============================================================================

/**
 * @brief DMA Interrupt Handler.
 *
 * Triggered when the DMA channel finishes filling the BEMF ring buffer.
 * This indicates that a full burst of ADC samples has been collected.
 */
static void dma_irq_handler() {
    // Iterate through all initialized motors to see which one triggered the IRQ
    for (int id = 0; id < MAX_MOTORS; id++) {
        MotorContext* ctx = &g_motors[id];
        if (!ctx->is_initialized || ctx->dma_channel < 0) continue;

        // Check if this channel triggered the interrupt
        if (dma_hw->ints0 & (1u << ctx->dma_channel)) {

            // Stop the ADC immediately to prevent sampling during the ON phase (next PWM cycle).
            // We only want samples during the "quiet" OFF phase.
            adc_run(false);
            g_adc_busy = false; // Release the ADC lock for other motors

            // Clear the DMA interrupt flag for our channel
            dma_hw->ints0 = 1u << ctx->dma_channel;

            uint32_t sum_A = 0, sum_B = 0;

            // Process the collected samples.
            // The ADC round-robin was configured to sample [Pin B, Pin A].
            // We de-interleave the buffer: even indices are B, odd indices are A.
            for (uint i = 0; i < BEMF_RING_BUFFER_SIZE; i += 2) {
                sum_B += ctx->bemf_ring_buffer[i];
                sum_A += ctx->bemf_ring_buffer[i + 1];
            }

            // Calculate the average differential BEMF.
            // BEMF = V_a - V_b (or vice versa, handled by abs()).
            // This cancels out common-mode noise.
            int count = BEMF_RING_BUFFER_SIZE / 2;
            int measured_bemf = abs((int)(sum_A / count) - (int)(sum_B / count));

            if (ctx->bemf_callback) {
                ctx->bemf_callback(measured_bemf);
            }

            // Note: We do NOT re-arm the DMA channel here.
            // Re-arming is done in the delayed_adc_trigger_callback, synchronized
            // with the PWM OFF phase.
        }
    }
}

/**
 * @brief Triggers a BEMF ADC measurement sequence.
 *
 * This function initiates the process of sampling the BEMF pins. It handles
 * ADC resource locking, overcurrent protection, and DMA re-arming. It can be
 * called directly from an ISR (for discrete mode) or from a timer callback
 * (for standard mode).
 * @param ctx Pointer to the motor context.
 */
static void trigger_adc_measurement(MotorContext* ctx) {
    if (!ctx) return;

    // Concurrency Check: The RP2040 has only one ADC. If it's busy, we skip this cycle.
    if (g_adc_busy) return;

#if defined(MOTOR_CURRENT_PIN)
    // --- FAST OVERCURRENT PROTECTION ---
    // Perform a blocking read of the current sense pin.
    // NOTE: This assumes a shared current pin or handled by macro.
    // (GPIO 26 is ADC channel 0, hence subtraction)
    adc_select_input(MOTOR_CURRENT_PIN - MOTOR_ADC_BASE_PIN);
    uint16_t current_val = adc_read();

    // Calculate threshold: (Limit_Amps * Shunt_Ohms / 3.3V) * 4095
    const float V_limit = MAX_CURRENT_AMPS * SHUNT_RESISTOR_OHMS;
    const uint16_t adc_threshold = (uint16_t)((V_limit / ADC_REF_VOLTAGE) * ADC_MAX_VALUE);

    if (current_val > adc_threshold) {
        // Fast Shutdown: Set both pins to OFF state immediately.
        // In discrete mode, this means turning everything OFF (High-Z).
        if (ctx->is_discrete_mode) {
             pwm_set_gpio_level(ctx->pwm_a_pin, 0); // HS_A
             pwm_set_gpio_level(ctx->pwm_b_pin, 0); // HS_B
             pwm_set_gpio_level(ctx->ls_a_pin,  0); // LS_A
             pwm_set_gpio_level(ctx->ls_b_pin,  0); // LS_B
        } else {
             // For standard mode with Inversion, PWM_MAX_TOP is OFF.
             pwm_set_gpio_level(ctx->pwm_a_pin, PWM_MAX_TOP);
             pwm_set_gpio_level(ctx->pwm_b_pin, PWM_MAX_TOP);
        }
        return;
    }
#endif

    // Acquire lock for ADC usage
    g_adc_busy = true;

    // Configure ADC for Differential BEMF Measurement
    // We sample the two motor pins (A and B) in rapid succession.
    uint8_t adc_ch_a = ctx->bemf_a_pin - MOTOR_ADC_BASE_PIN;
    uint8_t adc_ch_b = ctx->bemf_b_pin - MOTOR_ADC_BASE_PIN;

    // Start with channel B, then round-robin to A.
    // This creates the [B, A, B, A...] sequence in the buffer.
    adc_select_input(adc_ch_b);
    adc_set_round_robin((1u << adc_ch_a) | (1u << adc_ch_b));

    // Drain any stale data from the ADC FIFO to ensure alignment
    adc_fifo_drain();

    // Re-arm the DMA channel to receive the new burst of samples.
    // dma_channel_set_trans_count resets the transfer counter.
    dma_channel_set_trans_count(ctx->dma_channel, BEMF_RING_BUFFER_SIZE, false);
    // Point DMA back to the start of the buffer.
    dma_channel_set_write_addr(ctx->dma_channel, ctx->bemf_ring_buffer, true);

    // Start the ADC in free-running mode (driven by internal timer, pushed to FIFO)
    adc_run(true);
}

/**
 * @brief Timer Callback to Trigger ADC (Standard Mode).
 *
 * This function acts as a wrapper for `trigger_adc_measurement` to be
 * compatible with the `add_alarm_in_us` API.
 */
static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data) {
    trigger_adc_measurement((MotorContext*)user_data);
    return 0; // Return 0 to stop the alarm from repeating
}

/**
 * @brief PWM Wrap Interrupt Handler.
 *
 * Triggered at the beginning of every PWM cycle (counter wraps to 0).
 * We use this to schedule the ADC measurement at the correct point in the cycle.
 */
static void on_pwm_wrap() {
    // Check which slice triggered the interrupt and map it to a motor.
    for (int id = 0; id < MAX_MOTORS; id++) {
        MotorContext* ctx = &g_motors[id];
        if (!ctx->is_initialized) continue;

        // Check if this motor's PWM slice triggered the IRQ
        if (pwm_get_irq_status_mask() & (1u << ctx->motor_pwm_slice_a)) {
            pwm_clear_irq(ctx->motor_pwm_slice_a);

            if (!ctx->skip_measurement) {
                // For discrete H-Bridge with inverted logic, the wrap point IS the middle of the OFF phase.
                // We can trigger the measurement directly without a delay.
                if (ctx->is_discrete_mode) {
                    trigger_adc_measurement(ctx);
                } else {
                    // For the standard driver, we still need a delay to reach the OFF phase.
                    add_alarm_in_us(ctx->adc_trigger_delay_us, delayed_adc_trigger_callback, ctx, true);
                }
            }
        }
    }
}

/**
 * @brief Common Initialization Logic for Motor Context.
 *
 * Configures PWM frequency, slices, interrupts, and ADC/DMA.
 * Shared between standard and discrete init modes.
 */
static void common_motor_init(MotorContext* ctx) {
    bool bemf_enabled = (ctx->bemf_a_pin != MOTOR_PIN_UNDEFINED) && (ctx->bemf_b_pin != MOTOR_PIN_UNDEFINED);
    ctx->skip_measurement = !bemf_enabled;

    // --- PWM Setup ---
    gpio_set_function( ctx->pwm_a_pin, GPIO_FUNC_PWM );
    gpio_set_function( ctx->pwm_b_pin, GPIO_FUNC_PWM );
    
    if (ctx->is_discrete_mode) {
        // Discrete Mode: Use INVERTED polarity.
        // This clever trick shifts the PWM_IRQ_WRAP interrupt to occur exactly
        // in the middle of the OFF-phase (null-phase), allowing us to trigger
        // the ADC for BEMF measurement with perfect hardware timing, eliminating
        // the need for a software delay timer.
        gpio_set_outover(  ctx->pwm_b_pin, GPIO_OVERRIDE_INVERT );
        gpio_set_outover(  ctx->pwm_a_pin, GPIO_OVERRIDE_INVERT );

        gpio_set_function( ctx->ls_a_pin, GPIO_FUNC_PWM );
        gpio_set_function( ctx->ls_b_pin, GPIO_FUNC_PWM );
        gpio_set_outover(  ctx->ls_a_pin, GPIO_OVERRIDE_INVERT );
        gpio_set_outover(  ctx->ls_b_pin, GPIO_OVERRIDE_INVERT );

        // Initial OFF state (Level TOP = OFF due to inversion)
        pwm_set_gpio_level(ctx->ls_a_pin, PWM_MAX_TOP);
        pwm_set_gpio_level(ctx->ls_b_pin, PWM_MAX_TOP);
        pwm_set_gpio_level(ctx->pwm_a_pin, PWM_MAX_TOP);
        pwm_set_gpio_level(ctx->pwm_b_pin, PWM_MAX_TOP);
    } else {
        // Standard Mode: Invert the PWM output.
        // Often required for driver logic or to match historical behavior.
        gpio_set_outover(  ctx->pwm_b_pin, GPIO_OVERRIDE_INVERT);
        gpio_set_outover(  ctx->pwm_a_pin, GPIO_OVERRIDE_INVERT);

        // Set initial duty to "OFF" state. (Wrap/Top = OFF due to Inversion)
        pwm_set_gpio_level(ctx->pwm_a_pin, PWM_MAX_TOP);
        pwm_set_gpio_level(ctx->pwm_b_pin, PWM_MAX_TOP);
    }

    // --- PWM Frequency Calculation ---
    // Formula: SystemClock = Frequency * (WrapValue + 1) * Divider

    uint32_t system_clock = RP2040_SYSTEM_CLOCK_HZ;
    float divider = 1.0f;
    uint32_t top = system_clock / PWM_FREQUENCY_HZ;

    // If 'top' exceeds the 16-bit counter limit (65535), we must use the clock divider.
    if (top > PWM_MAX_TOP) {
        divider = (float)top / (float)PWM_MAX_TOP;
        if (divider > PWM_MAX_DIVIDER) divider = PWM_MAX_DIVIDER;
    }
    
    // Calculate the final wrap value with the chosen divider.
    ctx->pwm_wrap_value = (uint16_t)(system_clock / (PWM_FREQUENCY_HZ * divider)) - 1;

    pwm_config motor_pwm_conf = pwm_get_default_config();
    pwm_config_set_clkdiv(        &motor_pwm_conf, divider );
    pwm_config_set_wrap(          &motor_pwm_conf, ctx->pwm_wrap_value );
    pwm_config_set_phase_correct( &motor_pwm_conf, true ); // Phase correct for symmetric pulses
      
    // Apply configuration to slices
    ctx->motor_pwm_slice_a = pwm_gpio_to_slice_num(ctx->pwm_a_pin);
    ctx->motor_pwm_slice_b = pwm_gpio_to_slice_num(ctx->pwm_b_pin);
    
    // Reset counters to 0 to ensure synchronization
    pwm_set_counter(ctx->motor_pwm_slice_a, 0);
    pwm_set_counter(ctx->motor_pwm_slice_b, 0);

    // Initialize slices but do NOT enable them yet.
    pwm_init(ctx->motor_pwm_slice_a, &motor_pwm_conf, false);
    pwm_init(ctx->motor_pwm_slice_b, &motor_pwm_conf, false);

    // Enable Dead Time if in discrete mode
    if (ctx->is_discrete_mode) {
        // Dead-time is implemented by offsetting the compare values in hal_motor_set_pwm
    }
    
#ifdef USE_IRQ_TRIGGER
    if (bemf_enabled) {
        // Enable PWM Wrap Interrupt for ADC synchronization
        pwm_clear_irq(ctx->motor_pwm_slice_a);
        pwm_set_irq_enabled(ctx->motor_pwm_slice_a, true);
        irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
        irq_set_enabled(PWM_IRQ_WRAP, true);
    }
#endif
    
    // Start both slices exactly synchronously.
    uint32_t mask = (1u << ctx->motor_pwm_slice_a) | (1u << ctx->motor_pwm_slice_b);
    hw_set_bits(&pwm_hw->en, mask);
        
    // --- ADC and DMA Setup ---
    if (bemf_enabled) {
        static bool adc_initialized = false;
        if (!adc_initialized) {
            adc_init();
            // Setup ADC FIFO:
            // - Shift: true (required for DMA to read correctly)
            // - DREQ: true (enable DMA request from ADC)
            // - Threshold: ADC_FIFO_THRESHOLD (trigger DREQ after N samples)
            // - Err: false, Scale: false
            adc_fifo_setup(true, true, ADC_FIFO_THRESHOLD, false, false);
            adc_initialized = true;
        }

        adc_gpio_init(ctx->bemf_a_pin);
        adc_gpio_init(ctx->bemf_b_pin);

        // --- DMA Configuration ---
        ctx->dma_channel = dma_claim_unused_channel(true);
        dma_channel_config dma_config = dma_channel_get_default_config(ctx->dma_channel);
        channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16); // 16-bit transfers (uint16_t samples)
        channel_config_set_read_increment(&dma_config, false);           // Read from fixed ADC FIFO address
        channel_config_set_write_increment(&dma_config, true);           // Increment write address (fill buffer)
        channel_config_set_dreq(&dma_config, DREQ_ADC);                  // Pace transfers based on ADC availability

        // Configure Ring Buffer.
        channel_config_set_ring(&dma_config, true, __builtin_ctz(BEMF_RING_BUFFER_SIZE * sizeof(uint16_t)));

        dma_channel_configure(
            ctx->dma_channel,
            &dma_config,
            ctx->bemf_ring_buffer, // Destination
            &adc_hw->fifo,         // Source
            BEMF_RING_BUFFER_SIZE, // Transfer count
            false                  // Do not start yet
        );

        // Setup DMA Interrupt
        dma_channel_set_irq0_enabled(ctx->dma_channel, true);
        irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
        irq_set_enabled(DMA_IRQ_0, true);
    }
    
    ctx->is_initialized = true;
}


// =============================================================================
// Public HAL Function Implementations
// =============================================================================

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
    ctx->dma_channel = -1;
    ctx->is_discrete_mode = false;

    common_motor_init(ctx);
}

void hal_motor_init_discrete(uint8_t hs_a_pin, uint8_t ls_a_pin, uint8_t hs_b_pin, uint8_t ls_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return;

    // Validate that HS and LS pins of each half-bridge are on the same PWM Slice.
    // This is required to use hardware dead-time and ensure synchronization.
    if (pwm_gpio_to_slice_num(hs_a_pin) != pwm_gpio_to_slice_num(ls_a_pin)) return;
    if (pwm_gpio_to_slice_num(hs_b_pin) != pwm_gpio_to_slice_num(ls_b_pin)) return;

    MotorContext* ctx = &g_motors[motor_id];

    // Map pins: pwm_a/b used for HS, ls_a/b for LS.
    ctx->pwm_a_pin  = hs_a_pin;
    ctx->ls_a_pin   = ls_a_pin;
    ctx->pwm_b_pin  = hs_b_pin;
    ctx->ls_b_pin   = ls_b_pin;

    ctx->bemf_a_pin = bemf_a_pin;
    ctx->bemf_b_pin = bemf_b_pin;
    ctx->bemf_callback = callback;
    ctx->adc_trigger_delay_us = BEMF_MEASUREMENT_DELAY_US;
    ctx->dma_channel = -1;
    ctx->is_discrete_mode = true;

    common_motor_init(ctx);
}

void hal_motor_set_pwm(int duty_cycle, bool forward, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return;
    MotorContext* ctx = &g_motors[motor_id];
    if (!ctx->is_initialized) return;

    if (ctx->is_discrete_mode) {
        // Discrete H-Bridge Control (Fast Decay / Coasting Mode)
        // We use INVERTED polarity. The wrap interrupt now happens in the middle of the OFF phase.
        // A lower threshold means a longer ON time.
        // Map 0->Wrap (OFF), 255->0 (ON)
        uint16_t on_level = map(duty_cycle, PWM_DUTY_MIN, PWM_DUTY_MAX, ctx->pwm_wrap_value, 0);
        uint16_t off_level = ctx->pwm_wrap_value; // Safe OFF

        // Add dead-time to the on_level threshold.
        // A higher threshold shortens the ON pulse, creating dead time.
        uint16_t on_level_dead_time;
        if ((uint32_t)on_level + PWM_DEAD_TIME_CYCLES < ctx->pwm_wrap_value) {
            on_level_dead_time = on_level + PWM_DEAD_TIME_CYCLES;
        } else {
            on_level_dead_time = ctx->pwm_wrap_value;
        }

        // During the OFF phase, all FETs must be off to allow the motor to coast
        // and generate a measurable BEMF. We achieve this by driving the active
        // Low-Side FET with the same PWM signal as the High-Side (without dead-time),
        // ensuring they switch off together into a high-impedance state.
        if (forward) {
             // Forward: Left HS (A) + Right LS (B)
             pwm_set_gpio_level(ctx->pwm_a_pin, on_level_dead_time);  // HS_A = PWM
             pwm_set_gpio_level(ctx->ls_a_pin,  off_level); // LS_A = OFF

             pwm_set_gpio_level(ctx->pwm_b_pin, off_level); // HS_B = OFF
             pwm_set_gpio_level(ctx->ls_b_pin,  on_level);  // LS_B = PWM
        } else {
             // Reverse: Right HS (B) + Left LS (A)
             pwm_set_gpio_level(ctx->pwm_a_pin, off_level); // HS_A = OFF
             pwm_set_gpio_level(ctx->ls_a_pin,  on_level);  // LS_A = PWM

             pwm_set_gpio_level(ctx->pwm_b_pin, on_level_dead_time);  // HS_B = PWM
             pwm_set_gpio_level(ctx->ls_b_pin,  off_level); // LS_B = OFF
        }

    } else {
        // Standard Integrated Driver (2-Pin)
        // Inverted Logic: 255 -> 0 (ON), 0 -> Wrap (OFF)
        uint16_t on_level = map(duty_cycle, PWM_DUTY_MIN, PWM_DUTY_MAX, ctx->pwm_wrap_value, 0);
        uint16_t off_level = PWM_MAX_TOP;

        if (forward) {
           pwm_set_gpio_level(ctx->pwm_a_pin, on_level);
           pwm_set_gpio_level(ctx->pwm_b_pin, off_level);
        } else {
           pwm_set_gpio_level(ctx->pwm_a_pin, off_level);
           pwm_set_gpio_level(ctx->pwm_b_pin, on_level);
       }
    }
}

int hal_motor_get_bemf_buffer(volatile uint16_t** buffer, int* last_write_pos, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return 0;
    MotorContext* ctx = &g_motors[motor_id];
    if (!ctx->is_initialized) return 0;

    *buffer = ctx->bemf_ring_buffer;

    // Calculate current write position in the ring buffer.
    // This allows the caller to see where the DMA is currently writing,
    // although with burst transfers, this is usually 0 or full.
    uintptr_t current_write_addr = (uintptr_t)dma_hw->ch[ctx->dma_channel].write_addr;
    uintptr_t buffer_start_addr = (uintptr_t)ctx->bemf_ring_buffer;
    int byte_offset = current_write_addr - buffer_start_addr;

    *last_write_pos = byte_offset / sizeof(uint16_t);

    return BEMF_RING_BUFFER_SIZE;
}

int hal_motor_get_current_buffer(volatile uint16_t** buffer, int* last_write_pos, uint8_t motor_id) {
    // Current measurement not yet implemented in ring buffer for this HAL.
    *buffer         = nullptr;
    *last_write_pos = 0;
    return 0;
}

#endif // ARDUINO_ARCH_RP2040
