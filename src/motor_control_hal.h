/**
 * @file motor_control_hal.h
 * @brief Hardware Abstraction Layer for motor control.
 *
 * Defines a platform-agnostic interface for PWM motor control and BEMF sensing.
 */
#ifndef MOTOR_CONTROL_HAL_H
#define MOTOR_CONTROL_HAL_H

#include <cstdint>

// Maximum number of motors supported.
#ifndef MAX_MOTORS
#define MAX_MOTORS 2
#endif

// Default size of the BEMF ADC reading ring buffer.
const uint32_t BEMF_RING_BUFFER_SIZE = 4;

// Represents an undefined/unused pin.
const uint8_t MOTOR_PIN_UNDEFINED = 0xFF;

/**
 * @brief Callback function for BEMF (Back-EMF) updates.
 * @param raw_bemf_value The raw, unfiltered differential BEMF ADC value.
 *
 * Called from an interrupt when a new BEMF measurement is available.
 */
typedef void (*hal_bemf_update_callback_t)(int raw_bemf_value);

/**
 * @brief Initializes PWM for a 2-pin motor driver.
 * @param pwm_a_pin GPIO pin for PWM channel A.
 * @param pwm_b_pin GPIO pin for PWM channel B.
 * @param pwm_frequency PWM frequency in Hz.
 * @param motor_id Motor index to control.
 */
void hal_motor_init_pwm(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint32_t pwm_frequency = 20000, uint8_t motor_id = 0);

/**
 * @brief (Deprecated) Initializes ADC and DMA for BEMF sensing.
 * @param bemf_a_pin ADC pin for motor terminal A.
 * @param bemf_b_pin ADC pin for motor terminal B.
 * @param callback Function to call with new BEMF data.
 * @param motor_id Motor index to control.
 */
void hal_motor_init_bemf_adc_dma(uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback, uint8_t motor_id = 0) __attribute__((deprecated("Use hal_motor_init_bemf_adc and hal_motor_init_bemf_dma instead")));

/**
 * @brief Initializes ADC for BEMF sensing.
 * @param bemf_a_pin ADC pin for motor terminal A.
 * @param bemf_b_pin ADC pin for motor terminal B.
 * @param motor_id Motor index to control.
 */
void hal_motor_init_bemf_adc(uint8_t bemf_a_pin, uint8_t bemf_b_pin, uint8_t motor_id = 0);

/**
 * @brief Initializes DMA for BEMF sensing.
 * @param callback Function to call with new BEMF data.
 * @param motor_id Motor index to control.
 */
void hal_motor_init_bemf_dma(hal_bemf_update_callback_t callback, uint8_t motor_id = 0);

/**
 * @brief Sets motor PWM duty cycle and direction.
 * @param duty_cycle PWM duty cycle (e.g., 0-255).
 * @param forward True for forward, false for reverse.
 * @param motor_id Motor index to control.
 */
void hal_motor_set_duty(int duty_cycle, bool forward, uint8_t motor_id = 0);

/**
 * @brief Retrieves the BEMF ring buffer for diagnostics.
 * @param[out] buffer Pointer to the internal ring buffer.
 * @param[out] last_write_pos Last written position in the buffer.
 * @param motor_id Motor index to control.
 * @return Total size of the ring buffer.
 *
 * Provides low-level access to the raw ADC sample buffer for debugging.
 * Contains interleaved samples from ADC A and B.
 */
int hal_motor_get_bemf_buffer(volatile uint16_t** buffer, int* last_write_pos, uint8_t motor_id = 0);

/**
 * @brief Pings a solenoid/motor and measures the BEMF response.
 * @param ping_pwm_value PWM duty cycle for the ping pulse.
 * @param ping_duration_ms Duration of the ping pulse in ms.
 * @param measurement_delay_us Delay after ping before measuring response.
 * @param[out] response_a Measured response for the forward/A direction.
 * @param[out] response_b Measured response for the reverse/B direction.
 * @param motor_id Motor index to control.
 *
 * Pings forward, measures response, pings reverse, measures response.
 */
void hal_motor_check_solenoid_position(int ping_pwm_value, int ping_duration_ms, int measurement_delay_us, int* response_a, int* response_b, uint8_t motor_id = 0);

#endif // MOTOR_CONTROL_HAL_H
