#ifndef MOTOR_CONTROL_HAL_STM32G431_INTERNAL_H
#define MOTOR_CONTROL_HAL_STM32G431_INTERNAL_H

#if defined(ARDUINO_ARCH_STM32)

#include "stm32g4xx_hal.h"
#include "motor_control_hal.h"

// --- Typeefs ---
// Motor context structure
typedef struct {
    // PWM
    uint8_t pwm_a_pin;
    uint8_t pwm_b_pin;
    uint32_t pwm_frequency;
    TIM_HandleTypeDef htim;
    uint32_t channel_a;
    uint32_t channel_b;

    // BEMF
    ADC_HandleTypeDef hadc;
    DMA_HandleTypeDef hdma_adc;
    volatile uint16_t bemf_buffer[BEMF_RING_BUFFER_SIZE * 2]; // Interleaved A and B
    hal_bemf_update_callback_t bemf_callback;

} MotorContext;

// --- Variables ---
// Array to hold the context for each motor
extern MotorContext motor_contexts[MAX_MOTORS];

#endif // ARDUINO_ARCH_STM32

#endif // MOTOR_CONTROL_HAL_STM32G431_INTERNAL_H
