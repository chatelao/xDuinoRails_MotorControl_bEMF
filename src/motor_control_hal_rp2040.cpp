/**
 * @file motor_control_hal_rp2040.cpp
 * @brief RP2040-specific implementation for the motor control HAL.
 *
 * @details This file provides the concrete implementation of the functions
 * defined in motor_control_hal.h for the Raspberry Pi RP2040 microcontroller.
 * It leverages the RP2040's powerful hardware peripherals (PWM, ADC, DMA) to
 * create an efficient, non-blocking (CPU-free) motor control system.
 *
 * @section high-level-overview High-Level Overview for Product Managers
 *
 * This code is the "digital engine" that drives a physical motor. It translates
 * simple commands like "turn at 50% speed" into the precise electrical pulses
 * the motor needs. Key features include:
 *
 * - **CPU-Free Operation:** Once started, the motor control runs entirely on
 *   dedicated hardware. This frees up the main processor (CPU) to handle other
 *   tasks, like communication or user interface logic, making the whole
- **Sensorless Feedback (BEMF):** By listening to the motor's electrical "echo"
 *   (Back-EMF) when it's not being actively powered, we can determine its
 *   speed and position without needing extra sensors. This reduces cost and
 *   complexity.
 * - **Flexible Driver Support:** The code supports both simple, integrated motor
 *   driver chips (2-pin control) and more complex, discrete H-bridges (4-pin
 *   control), allowing for flexibility in hardware design.
 * - **Safety First:** Includes basic overcurrent protection to shut down the
 *   motor if it draws too much power, preventing damage.
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
/**
 * @brief Defines the core operating parameters of the motor control system.
 * @note For Product Managers: Think of this section as the "spec sheet" for our
 * motor controller. It defines the physical limits and properties, like the
 * processor's speed, the precision of our sensors (ADC), and the range of
 * our throttle (PWM).
 */

// The RP2040 system clock is the heartbeat of the microcontroller, 125 MHz.
// It's the basis for all timing calculations.
const uint32_t RP2040_SYSTEM_CLOCK_HZ   = 125000000;

// --- ADC (Analog-to-Digital Converter) Configuration ---
// This peripheral measures real-world voltages, like the BEMF from the motor.
const int      ADC_RESOLUTION_BITS  = 12;      // RP2040's ADC has 12-bit resolution.
const float    ADC_MAX_VALUE        = 4095.0f; // 2^12 - 1
const float    ADC_REF_VOLTAGE      = 3.3f;    // The internal voltage reference for the ADC.
const uint     MOTOR_ADC_BASE_PIN   = 26;      // GPIO26 corresponds to ADC channel 0.
const uint     ADC_FIFO_THRESHOLD   = 1;       // DMA request is triggered for every single sample.

// --- PWM (Pulse-Width Modulation) Configuration ---
// This peripheral creates the digital signals that control the motor's speed.
const uint16_t PWM_MAX_TOP          = 65535;   // The PWM counter is 16-bit.
const float    PWM_MAX_DIVIDER      = 255.0f;  // The clock divider has 8 integer and 4 fractional bits.
const int      PWM_DUTY_MIN         = 0;       // Corresponds to 0% duty cycle.
const int      PWM_DUTY_MAX         = 255;     // Corresponds to 100% duty cycle.
const uint16_t PWM_DEAD_TIME_CYCLES = 50;      // Dead time in system clock cycles. At 125MHz, 1 cycle = 8ns. 50*8 = 400ns.

// =============================================================================
// Data Structures
// =============================================================================
/**
 * @brief Defines the structures used to manage the state of each motor.
 * @note For Product Managers: This is the "control panel" for each motor.
 * The `MotorContext` structure holds all the important information for a single
 * motor, like which pins it's connected to, its current settings, and the data
 * buffers for its sensor readings.
 */

// Differentiates between a BEMF measurement (during PWM OFF phase) and a
// shunt current measurement (during PWM ON phase).
enum MeasurementType {
    BEMF,
    SHUNT
};

struct MotorContext; // Forward declaration for use in AlarmUserData.

// Bundles the motor context and measurement type to be passed to the Pico SDK's
// timer ("alarm") callback. The callback API only accepts a single `void*` argument.
struct AlarmUserData {
    MotorContext* ctx;
    MeasurementType type;
};

// Encapsulates all state variables for a single motor instance.
struct MotorContext {
    bool     is_initialized;          // Flag to prevent use before configuration.
    // --- Hardware Pin Assignments ---
    uint8_t  pwm_a_pin;               // For 2-pin, IN1. For 4-pin, High-Side A.
    uint8_t  pwm_b_pin;               // For 2-pin, IN2. For 4-pin, High-Side B.
    uint8_t  ls_a_pin;                // Low-Side A (4-pin discrete mode only).
    uint8_t  ls_b_pin;                // Low-Side B (4-pin discrete mode only).
    uint8_t  bemf_a_pin;              // ADC pin for motor terminal A.
    uint8_t  bemf_b_pin;              // ADC pin for motor terminal B.
    uint8_t  shunt_a_pin;             // ADC pin for shunt resistor A.
    uint8_t  shunt_b_pin;             // ADC pin for shunt resistor B.
    // --- PWM & Timing Configuration ---
    uint     motor_pwm_slice_a;       // Each pair of PWM pins is a "slice".
    uint     motor_pwm_slice_b;
    uint16_t pwm_wrap_value;          // The value the PWM counter resets at, defining the period.
    uint32_t pwm_frequency;           // The configured PWM frequency in Hz.
    // --- BEMF DMA and ADC ---
    int      dma_channel_bemf;        // The DMA channel number for BEMF transfers.
    // The `volatile` keyword prevents the compiler from optimizing away reads
    // of this buffer, as it's written to by hardware (DMA).
    volatile uint16_t          bemf_ring_buffer[BEMF_RING_BUFFER_SIZE];
    hal_bemf_update_callback_t bemf_callback; // Function pointer to user code.
    // --- Shunt DMA and ADC ---
    int                        dma_channel_shunt;
    volatile uint16_t          shunt_ring_buffer[SHUNT_RING_BUFFER_SIZE];
    hal_shunt_update_callback_t shunt_callback;
    // --- Internal State ---
    volatile uint32_t           adc_trigger_delay_us; // Delay from PWM wrap to ADC start.
    volatile bool               skip_measurement;     // Flag to disable BEMF for a motor (e.g., LEDs).
    AlarmUserData               alarm_user_data;
};

// A static array to hold the context for each supported motor.
static MotorContext g_motors[MAX_MOTORS];
// A global flag to ensure only one motor uses the ADC at a time, preventing conflicts.
static volatile bool g_adc_busy = false;

// =============================================================================
// Forward Declarations
// =============================================================================
/**
 * @brief Declares internal functions before they are defined.
 * @note For Product Managers: This is like a table of contents for the internal
 * helper functions, allowing us to organize the code logically.
 */

static void dma_irq_handler();
static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data);
static void on_pwm_wrap();
static void pwm_init_common(MotorContext* ctx);

// =============================================================================
// Internal Helper Functions & Interrupt Service Routines (ISRs)
// =============================================================================
/**
 * @brief The "autopilot" functions that run automatically in response to hardware events.
 * @note For Product Managers: These functions are the core of the "CPU-free"
 * operation. They are triggered by hardware events (like the end of a PWM
 * cycle or a full data buffer) and handle the BEMF measurement and data
 * processing without any intervention from the main application code.
 */

/**
 * @brief DMA Interrupt Handler: The "Mailbox Checker".
 * @details This function is automatically called by the hardware when the DMA
 * controller has finished filling a data buffer (e.g., `bemf_ring_buffer`)
 * with fresh ADC samples. Its job is to quickly process the new data, calculate
 * the average BEMF, and notify the main application via the callback.
 * @note This function executes in an interrupt context (ISR). It must be fast
 * and avoid blocking operations (like `delay` or serial prints).
 */
static void dma_irq_handler() {
    // Iterate through all possible motors to find which DMA channel triggered the interrupt.
    for (int id = 0; id < MAX_MOTORS; id++) {
        MotorContext* ctx = &g_motors[id];
        if (!ctx->is_initialized) continue;

        // --- BEMF Data Processing ---
        // Check if the interrupt was triggered by this motor's BEMF DMA channel.
        // `dma_hw->ints0` is a raw hardware register showing pending interrupts.
        if (ctx->dma_channel_bemf >= 0 && (dma_hw->ints0 & (1u << ctx->dma_channel_bemf))) {

            // Immediately stop the ADC. This is critical to prevent the ADC from
            // continuing to sample into the next PWM "ON" phase, which would
            // contaminate the BEMF readings with the driving voltage.
            adc_run(false);
            g_adc_busy = false; // Release the ADC resource lock.

            // Clear the interrupt flag for this channel by writing a '1' to it.
            // This tells the hardware we've handled the interrupt.
            dma_hw->ints0 = 1u << ctx->dma_channel_bemf;

            uint32_t sum_A = 0, sum_B = 0;

            // The ADC was configured in round-robin mode, so the DMA buffer contains
            // interleaved samples: [B0, A0, B1, A1, ...]. We de-interleave them.
            for (uint i = 0; i < BEMF_RING_BUFFER_SIZE; i += 2) {
                sum_B += ctx->bemf_ring_buffer[i];
                sum_A += ctx->bemf_ring_buffer[i + 1];
            }

            // Averaging the samples reduces noise. We then calculate the absolute
            // difference (differential voltage), which further helps to reject
            // common-mode noise affecting both ADC inputs equally.
            int count = BEMF_RING_BUFFER_SIZE / 2;
            int measured_bemf = abs((int)(sum_A / count) - (int)(sum_B / count));

            // If a user callback was registered, call it to deliver the result.
            if (ctx->bemf_callback) {
                ctx->bemf_callback(measured_bemf);
            }

            // Note: The DMA is not re-armed here. It is explicitly re-armed by
            // `trigger_adc_measurement` which is synchronized to the PWM cycle.
        }

        // --- Shunt Data Processing (identical logic to BEMF) ---
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

/**
 * @brief Kicks off an ADC measurement sequence for BEMF or shunt current.
 *
 * @details This function is the gatekeeper for the ADC. It ensures that only one
 * measurement is happening at a time (`g_adc_busy`), configures the ADC to
 * listen to the correct pair of pins, and then arms the DMA to automatically
 * capture the results.
 *
 * @param ctx Pointer to the motor's context.
 * @param type The type of measurement to perform (BEMF or SHUNT).
 */
static void trigger_adc_measurement(MotorContext* ctx, MeasurementType type) {
    if (!ctx) return;

    // The RP2040 has only one ADC peripheral. If it's currently busy with a
    // measurement for another motor, we must skip this cycle to avoid a collision.
    // The `g_adc_busy` flag is volatile and provides a basic mutex.
    if (g_adc_busy) return;

    // "Lock" the ADC to prevent other processes from using it.
    g_adc_busy = true;

    if (type == BEMF) {
        // --- Configure ADC for BEMF Measurement ---
        // Convert GPIO numbers to ADC channel numbers.
        uint8_t adc_ch_a = ctx->bemf_a_pin - MOTOR_ADC_BASE_PIN;
        uint8_t adc_ch_b = ctx->bemf_b_pin - MOTOR_ADC_BASE_PIN;
        // Select the starting channel. The round-robin will proceed from here.
        adc_select_input(adc_ch_b);
        // Set a bitmask for the channels to be included in the round-robin sequence.
        adc_set_round_robin((1u << adc_ch_a) | (1u << adc_ch_b));
        // The ADC has a 4-level deep hardware FIFO. Draining it ensures no
        // stale data from previous measurements is present.
        adc_fifo_drain();

        // --- Re-arm the DMA channel ---
        // This must be done before every new sequence.
        // `dma_channel_set_trans_count`: How many transfers to perform.
        // `dma_channel_set_write_addr`: Reset the write pointer to the beginning of our buffer.
        // The `true` argument triggers the DMA channel to start immediately once configured.
        dma_channel_set_trans_count(ctx->dma_channel_bemf, BEMF_RING_BUFFER_SIZE, false);
        dma_channel_set_write_addr(ctx->dma_channel_bemf, ctx->bemf_ring_buffer, true);

    } else if (type == SHUNT) {
        // --- Configure ADC for Shunt Measurement ---
        // Logic is identical to BEMF, just using the shunt-related context variables.
        uint8_t adc_ch_a = ctx->shunt_a_pin - MOTOR_ADC_BASE_PIN;
        uint8_t adc_ch_b = ctx->shunt_b_pin - MOTOR_ADC_BASE_PIN;
        adc_select_input(adc_ch_b);
        adc_set_round_robin((1u << adc_ch_a) | (1u << adc_ch_b));
        adc_fifo_drain();

        // --- Re-arm the DMA channel for the shunt buffer ---
        dma_channel_set_trans_count(ctx->dma_channel_shunt, SHUNT_RING_BUFFER_SIZE, false);
        dma_channel_set_write_addr(ctx->dma_channel_shunt, ctx->shunt_ring_buffer, true);
    }

    // Start the ADC in free-running mode. The ADC will now digitize samples
    // as fast as it can, paced by its clock. The DMA's DREQ mechanism will
    // automatically transfer samples from the ADC FIFO to our buffer in memory.
    adc_run(true);
}

/**
 * @brief A lightweight wrapper to trigger an ADC measurement from a timer.
 * @details This function is needed because the RP2040's timer ("alarm") API
 * requires a specific function signature. This simply unpacks the user data
 * and calls the main `trigger_adc_measurement` function.
 */
static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data) {
    AlarmUserData* data = (AlarmUserData*)user_data;
    trigger_adc_measurement(data->ctx, data->type);
    return 0; // A non-zero return value would make the alarm repeat.
}

/**
 * @brief PWM Wrap Interrupt Handler: The "Conductor".
 * @details This function is the master timer for the entire measurement system.
 * It is automatically triggered by the PWM hardware at the precise start of
 * each cycle. Its main job is to schedule the BEMF measurement to occur at
 * the exact right moment during the "off" part of the cycle.
 * @note This is an ISR and must be fast.
 */
static void on_pwm_wrap() {
    // Iterate through motors to see which PWM slice triggered the IRQ.
    for (int id = 0; id < MAX_MOTORS; id++) {
        MotorContext* ctx = &g_motors[id];
        if (!ctx->is_initialized) continue;

        // `pwm_get_irq_status_mask()` returns a bitmask of all PWM slices with pending IRQs.
        // We check if our primary motor slice is among them.
        if (pwm_get_irq_status_mask() & (1u << ctx->motor_pwm_slice_a)) {
            pwm_clear_irq(ctx->motor_pwm_slice_a); // Acknowledge the interrupt.

            #if defined(MOTOR_CURRENT_PIN)
                // --- FAST, HARDWARE-LEVEL OVERCURRENT PROTECTION ---
                // This provides a rapid, albeit simple, safety mechanism.
                // It's a blocking ADC read, but it's very fast.
                if (!g_adc_busy) {
                    adc_select_input(MOTOR_CURRENT_PIN - MOTOR_ADC_BASE_PIN);
                    uint16_t current_val = adc_read();
                    const float V_limit = MAX_CURRENT_AMPS * SHUNT_RESISTOR_OHMS;
                    const uint16_t adc_threshold = (uint16_t)((V_limit / ADC_REF_VOLTAGE) * ADC_MAX_VALUE);

                    if (current_val > adc_threshold) {
                        // If the current threshold is exceeded, we immediately disable the PWM
                        // outputs by setting their level to a state that corresponds to "off",
                        // considering the inverted output override.
                        pwm_set_gpio_level(ctx->pwm_a_pin, PWM_MAX_TOP); pwm_set_gpio_level(ctx->pwm_b_pin, PWM_MAX_TOP);
                        return; // Halt all further processing for this motor this cycle.
                    }
                }
            #endif

            // Trigger the shunt measurement immediately at the start of the PWM cycle.
            // This is timed to capture the peak current during the ON phase.
            if (ctx->shunt_callback) {
                trigger_adc_measurement(ctx, SHUNT);
            }

            // --- Schedule the BEMF Measurement ---
            if (!ctx->skip_measurement) {
                // For a standard driver, the wrap occurs at the start of the ON phase.
                // We must wait for the PWM pulse to end plus a small settling time.
                // A hardware timer (`add_alarm_in_us`) is used for this to avoid
                // blocking the CPU. The delay is calculated from the duty cycle.
                ctx->alarm_user_data.ctx = ctx;
                ctx->alarm_user_data.type = BEMF;
                add_alarm_in_us(ctx->adc_trigger_delay_us, delayed_adc_trigger_callback, &ctx->alarm_user_data, true);
            }
        }
    }
}

/**
 * @brief Common logic for initializing the PWM hardware.
 * @details This helper function is shared by the public `hal_motor_init_pwm`
 * and `hal_motor_init_pwm_discrete` functions. It handles the complex calculations
 * for setting the PWM frequency and configures the GPIO pins and PWM slices.
 */
static void pwm_init_common(MotorContext* ctx) {
    // --- PWM Pin Setup ---
    // Assign the specified GPIO pins to be controlled by the PWM peripheral.
    gpio_set_function(ctx->pwm_a_pin, GPIO_FUNC_PWM);
    gpio_set_function(ctx->pwm_b_pin, GPIO_FUNC_PWM);
    
    // Standard drivers also often benefit from inverted logic depending on their datasheets.
    gpio_set_outover(ctx->pwm_b_pin, GPIO_OVERRIDE_INVERT);
    gpio_set_outover(ctx->pwm_a_pin, GPIO_OVERRIDE_INVERT);
    pwm_set_gpio_level(ctx->pwm_a_pin, PWM_MAX_TOP);
    pwm_set_gpio_level(ctx->pwm_b_pin, PWM_MAX_TOP);

    // --- PWM Frequency Calculation ---
    // This logic determines the clock divider and `wrap` (or "top") value to
    // achieve the target frequency. If the required `top` value is > 16 bits,
    // it calculates the smallest possible clock divider to bring it into range.
    uint32_t system_clock = RP2040_SYSTEM_CLOCK_HZ;
    float divider = 1.0f;
    uint32_t top = system_clock / ctx->pwm_frequency;
    if (top > PWM_MAX_TOP) {
        divider = (float)top / (float)PWM_MAX_TOP;
        if (divider > PWM_MAX_DIVIDER) divider = PWM_MAX_DIVIDER;
    }
    ctx->pwm_wrap_value = (uint16_t)(system_clock / (ctx->pwm_frequency * divider)) - 1;

    // --- PWM Peripheral Configuration ---
    pwm_config motor_pwm_conf = pwm_get_default_config();
    pwm_config_set_clkdiv(&motor_pwm_conf, divider);
    pwm_config_set_wrap(&motor_pwm_conf, ctx->pwm_wrap_value);
    // Phase-correct (triangle) PWM is used. This is important because it means
    // the center of the PWM "off" period is always at the wrap point of the counter,
    // simplifying BEMF measurement timing.
    pwm_config_set_phase_correct(&motor_pwm_conf, true);
      
    // Get the slice number associated with each GPIO pin.
    ctx->motor_pwm_slice_a = pwm_gpio_to_slice_num(ctx->pwm_a_pin);
    ctx->motor_pwm_slice_b = pwm_gpio_to_slice_num(ctx->pwm_b_pin);
    
    // Reset counters and initialize the slices with our configuration, but don't start them yet.
    pwm_set_counter(ctx->motor_pwm_slice_a, 0);
    pwm_set_counter(ctx->motor_pwm_slice_b, 0);
    pwm_init(ctx->motor_pwm_slice_a, &motor_pwm_conf, false);
    pwm_init(ctx->motor_pwm_slice_b, &motor_pwm_conf, false);
    
    // Atomically start both PWM slices using a bitmask on the hardware enable register.
    // This ensures both slices start on the exact same clock cycle.
    uint32_t mask = (1u << ctx->motor_pwm_slice_a) | (1u << ctx->motor_pwm_slice_b);
    hw_set_bits(&pwm_hw->en, mask);
        
    ctx->is_initialized = true;
}

// =============================================================================
// Public HAL Function Implementations
// =============================================================================
/**
 * @brief The public-facing functions that the main application will call.
 * @note For Product Managers: This is the official API (Application Programming
 * Interface). These are the simple, high-level commands our users (other
 * developers) will use to control the motors, without needing to know about
 * the complex hardware interactions happening under the hood.
 */

void hal_motor_init_pwm(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint32_t pwm_frequency, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return;
    MotorContext* ctx = &g_motors[motor_id];

    // Populate the context struct with the configuration for a 2-pin driver.
    ctx->pwm_frequency = pwm_frequency;
    ctx->pwm_a_pin = pwm_a_pin;
    ctx->pwm_b_pin = pwm_b_pin;
    ctx->dma_channel_bemf = -1;  // Mark DMA as uninitialized.
    ctx->dma_channel_shunt = -1;

    pwm_init_common(ctx); // Call the shared helper to configure the hardware.
}

void hal_motor_init_bemf_adc_dma(uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return;
    MotorContext* ctx = &g_motors[motor_id];
    // BEMF sensing is synchronized to the PWM cycle, so PWM must be initialized first.
    if (!ctx->is_initialized) return;

    ctx->bemf_a_pin = bemf_a_pin;
    ctx->bemf_b_pin = bemf_b_pin;
    ctx->bemf_callback = callback;
    ctx->adc_trigger_delay_us = BEMF_MEASUREMENT_DELAY_US;

    // If MOTOR_PIN_UNDEFINED is passed, we can disable BEMF for this motor instance.
    bool bemf_enabled = (bemf_a_pin != MOTOR_PIN_UNDEFINED) && (bemf_b_pin != MOTOR_PIN_UNDEFINED);
    ctx->skip_measurement = !bemf_enabled;
    if (!bemf_enabled) return;

    // --- ADC and DMA Hardware Setup for BEMF ---
    // The `static` keyword ensures these initializations happen only once,
    // even if this function is called for multiple motors.
    static bool adc_initialized = false;
    if (!adc_initialized) {
        adc_init();
        // Configure ADC to use its FIFO and generate a DMA request (DREQ)
        // when the FIFO has at least one sample.
        adc_fifo_setup(true, true, ADC_FIFO_THRESHOLD, false, false);
        adc_initialized = true;
    }

    // Initialize the GPIO pins for analog input.
    adc_gpio_init(ctx->bemf_a_pin);
    adc_gpio_init(ctx->bemf_b_pin);

    // Claim an unused DMA channel. The `true` makes the function panic if none are available.
    ctx->dma_channel_bemf = dma_claim_unused_channel(true);
    dma_channel_config dma_config = dma_channel_get_default_config(ctx->dma_channel_bemf);
    // Configure the DMA transfer: 16-bit reads, no read address increment,
    // write address increment, and trigger on ADC DREQ.
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_config, false);
    channel_config_set_write_increment(&dma_config, true);
    channel_config_set_dreq(&dma_config, DREQ_ADC);
    // Configure the write address to wrap around to the start of the buffer when it reaches the end.
    // The size must be a power of two. `__builtin_ctz` counts trailing zeros to find the power.
    channel_config_set_ring(&dma_config, true, __builtin_ctz(BEMF_RING_BUFFER_SIZE * sizeof(uint16_t)));

    dma_channel_configure(
        ctx->dma_channel_bemf, &dma_config,
        ctx->bemf_ring_buffer, // Destination address
        &adc_hw->fifo,         // Source address
        BEMF_RING_BUFFER_SIZE, // Number of transfers
        false                  // Don't start immediately
    );

    // Enable the IRQ for DMA channel 0, and set up our shared handler.
    dma_channel_set_irq0_enabled(ctx->dma_channel_bemf, true);
    static bool dma_irq_initialized = false;
    if (!dma_irq_initialized) {
        irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
        irq_set_enabled(DMA_IRQ_0, true);
        dma_irq_initialized = true;
    }

    // Finally, enable the PWM Wrap Interrupt. This is the master metronome that
    // synchronizes the ADC measurements to the PWM cycles.
    #ifdef USE_IRQ_TRIGGER
        pwm_clear_irq(ctx->motor_pwm_slice_a);
        pwm_set_irq_enabled(ctx->motor_pwm_slice_a, true);
        irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
        irq_set_enabled(PWM_IRQ_WRAP, true);
    #endif
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

    // The ADC and DMA setup for shunt current sensing is virtually identical to the
    // BEMF setup, just using a different set of pins, context variables, and buffer.
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

    // --- Standard Integrated Driver (2-Pin) ---
    // This is much simpler. Map the 0-255 duty to the counter levels.
    uint16_t on_level = map(duty_cycle, PWM_DUTY_MIN, PWM_DUTY_MAX, ctx->pwm_wrap_value, 0);
    uint16_t off_level = PWM_MAX_TOP; // A value that ensures the channel is always off.

    // Determine the correct PWM levels for each pin based on direction.
    uint16_t level_a = forward ? on_level : off_level;
    uint16_t level_b = forward ? off_level : on_level;

    // For efficiency, if both motor pins are on the same PWM slice, we can update
    // their levels simultaneously. This prevents a potential race condition where
    // one pin is high and the other is low for a very brief period.
    if (ctx->motor_pwm_slice_a == ctx->motor_pwm_slice_b) {
        // pwm_set_both_levels expects the levels for channel A and B respectively.
        pwm_set_both_levels(ctx->motor_pwm_slice_a, level_a, level_b);
    } else {
        // Fallback for pins on different slices.
        pwm_set_gpio_level(ctx->pwm_a_pin, level_a);
        pwm_set_gpio_level(ctx->pwm_b_pin, level_b);
    }
}

int hal_motor_get_bemf_buffer(volatile uint16_t** buffer, int* last_write_pos, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return 0;
    MotorContext* ctx = &g_motors[motor_id];
    if (!ctx->is_initialized) return 0;

    *buffer = ctx->bemf_ring_buffer;

    // This provides a raw, real-time view of the DMA's current write address.
    // It's mainly for diagnostic purposes, allowing tools to visualize the
    // buffer being filled. We read the hardware register directly.
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

    // Identical logic to the BEMF buffer getter.
    uintptr_t current_write_addr = (uintptr_t)dma_hw->ch[ctx->dma_channel_shunt].write_addr;
    uintptr_t buffer_start_addr = (uintptr_t)ctx->shunt_ring_buffer;
    int byte_offset = current_write_addr - buffer_start_addr;
    *last_write_pos = byte_offset / sizeof(uint16_t);

    return SHUNT_RING_BUFFER_SIZE;
}

#endif // ARDUINO_ARCH_RP2040
