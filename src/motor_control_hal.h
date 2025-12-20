/**
 * @file motor_control_hal.h
 * @brief Hardware Abstraction Layer (HAL) for low-level motor control.
 *
 * This file defines a platform-agnostic interface for hardware-accelerated
 * PWM motor control and BEMF (Back-EMF) measurement. The implementation for
 * a specific microcontroller must be provided separately.
 */
#ifndef MOTOR_CONTROL_HAL_H
#define MOTOR_CONTROL_HAL_H

#include <cstdint>

// --- Debugging Flags ---
// #define DEBUG_PWM

#ifdef DEBUG_PWM
// Use a dedicated pin (e.g., GPIO25 on XIAO RP2040's blue LED)
// to output a short pulse at the start of each PWM cycle.
#define DEBUG_PWM_PIN 25
#endif


// Maximum number of motors supported by the library
#ifndef MAX_MOTORS
#define MAX_MOTORS 2
#endif

// BEMF ring buffer size, adapted to the PWM frequency
#ifdef LED_EDITION
const uint32_t BEMF_RING_BUFFER_SIZE = 64;
const uint32_t SHUNT_RING_BUFFER_SIZE = 64;
#else
const uint32_t BEMF_RING_BUFFER_SIZE = 4;
const uint32_t SHUNT_RING_BUFFER_SIZE = 4;
#endif

// Delay after the PWM cycle before triggering ADC, allows the motor coils' magnetic field to collapse.
const uint32_t BEMF_MEASUREMENT_DELAY_US = 10;

// Default Shunt Resistor value in Ohms (can be overridden by build flags)
#ifndef SHUNT_RESISTOR_OHMS
#define SHUNT_RESISTOR_OHMS 0.5f
#endif

// Default Short Circuit Current Limit in Amps (can be overridden)
#ifndef MAX_CURRENT_AMPS
#define MAX_CURRENT_AMPS 2.0f
#endif

// Value to indicate that a pin is not used/undefined.
const uint8_t MOTOR_PIN_UNDEFINED = 0xFF;

/**
 * @brief Callback function pointer type for BEMF updates.
 *
 * This function is called from an interrupt context whenever a new
 * differential BEMF measurement is available from the hardware. It is the
 * responsibility of the callee to perform any necessary filtering, processing,
 * and control logic adjustments.
 *
 * @param raw_bemf_value The raw, unfiltered differential BEMF value, calculated
 *                       as the absolute difference between the two ADC readings.
 */
typedef void (*hal_bemf_update_callback_t)(int raw_bemf_value);

/**
 * @brief Callback function pointer type for Shunt updates.
 *
 * This function is called from an interrupt context whenever a new
 * shunt measurement is available from the hardware.
 *
 * @param raw_shunt_value The raw, unfiltered shunt value.
 */
typedef void (*hal_shunt_update_callback_t)(int raw_shunt_value);

/**
 * @brief Initializes the PWM hardware for a standard 2-pin motor driver.
 *
 * @param pwm_a_pin The GPIO pin for PWM channel A (e.g., forward).
 * @param pwm_b_pin The GPIO pin for PWM channel B (e.g., reverse).
 * @param pwm_frequency The desired PWM frequency in Hz.
 * @param motor_id The index of the motor to control.
 */
void hal_motor_init_pwm(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint32_t pwm_frequency = 20000, uint8_t motor_id = 0);

/**
 * @brief Initializes the BEMF sensing hardware using DMA.
 *
 * @param bemf_a_pin The ADC pin for motor terminal A.
 * @param bemf_b_pin The ADC pin for motor terminal B.
 * @param callback Function to be called with new BEMF data.
 * @param motor_id The index of the motor to control.
 */
void hal_motor_init_bemf_adc_dma(uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback, uint8_t motor_id = 0);

/**
 * @brief Initializes the Shunt sensing hardware using DMA.
 *
 * @param shunt_a_pin The ADC pin for motor terminal A.
 * @param shunt_b_pin The ADC pin for motor terminal B.
 * @param callback Function to be called with new shunt data.
 * @param motor_id The index of the motor to control.
 */
void hal_motor_init_shunt_adc_dma(uint8_t shunt_a_pin, uint8_t shunt_b_pin, hal_shunt_update_callback_t callback, uint8_t motor_id = 0);

/**
 * @brief Sets the motor's PWM duty cycle and direction.
 *
 * This function updates the PWM hardware with the new duty cycle. It should
 * be called periodically from the main application loop to reflect the latest
 * output from the motor control algorithm (e.g., a PI controller).
 *
 * @param duty_cycle The desired duty cycle, typically in a range from 0 to 255.
 * @param forward The desired motor direction (true for forward, false for reverse).
 * @param motor_id The index of the motor to control (0 to MAX_MOTORS-1). Defaults to 0.
 */
void hal_motor_set_pwm(int duty_cycle, bool forward, uint8_t motor_id = 0);

/**
 * @brief Retrieves the BEMF ring buffer for diagnostics.
 *
 * This function provides low-level access to the raw ADC sample buffer.
 * It is intended for debugging and visualization, not for real-time control.
 * The buffer contains interleaved samples from ADC A and ADC B.
 *
 * @param[out] buffer A pointer to a uint16_t pointer that will be set to the
 *                    address of the internal ring buffer.
 * @param[out] last_write_pos A pointer to an integer that will be set to the
 *                            last written position in the buffer.
 * @param motor_id The index of the motor to control (0 to MAX_MOTORS-1). Defaults to 0.
 * @return The total size of the ring buffer (number of samples).
 */
int hal_motor_get_bemf_buffer(volatile uint16_t** buffer, int* last_write_pos, uint8_t motor_id = 0);

/**
 * @brief Retrieves the Shunt Sensing ring buffer.
 *
 * Optional: Only available if enabled/supported by the platform and configuration.
 *
 * @param[out] buffer Pointer to the uint16_t pointer that will be set to the address of the buffer.
 * @param[out] last_write_pos Pointer to an integer that will be set to the last written position.
 * @param motor_id The index of the motor to control (0 to MAX_MOTORS-1). Defaults to 0.
 * @return The size of the ring buffer, or 0 if not supported.
 */
int hal_motor_get_shunt_buffer(volatile uint16_t** buffer, int* last_write_pos, uint8_t motor_id = 0);

/**
 * @brief Checks the solenoid/motor position by pinging both directions and measuring the response.
 *
 * This function performs a diagnostic test:
 * 1. Pings the "forward" direction (PWM A).
 * 2. Measures the BEMF/Response immediately after switch-off.
 * 3. Pings the "reverse" direction (PWM B).
 * 4. Measures the BEMF/Response immediately after switch-off.
 *
 * @param ping_pwm_value The PWM duty cycle (0-255) for the ping pulse.
 * @param ping_duration_ms The duration of the ping pulse in milliseconds.
 * @param measurement_delay_us The delay after switching off before measuring the response.
 * @param[out] response_a The measured response for the forward/A direction.
 * @param[out] response_b The measured response for the reverse/B direction.
 * @param motor_id The index of the motor to control (0 to MAX_MOTORS-1). Defaults to 0.
 */
void hal_motor_check_solenoid_position(int ping_pwm_value, int ping_duration_ms, int measurement_delay_us, int* response_a, int* response_b, uint8_t motor_id = 0);

#endif // MOTOR_CONTROL_HAL_H
