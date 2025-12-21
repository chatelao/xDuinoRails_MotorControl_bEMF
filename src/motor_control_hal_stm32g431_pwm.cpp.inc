#if defined(ARDUINO_ARCH_STM32)

#include <Arduino.h>
#include "pinmap.h"

void hal_motor_init_pwm(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint32_t pwm_frequency, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) {
        return;
    }

    MotorContext* context = &motor_contexts[motor_id];
    context->pwm_a_pin = pwm_a_pin;
    context->pwm_b_pin = pwm_b_pin;
    context->pwm_frequency = pwm_frequency;

    // --- Pin Mapping ---
    PinName pin_a = digitalPinToPinName(pwm_a_pin);
    PinName pin_b = digitalPinToPinName(pwm_b_pin);

    TIM_TypeDef *Instance_a = (TIM_TypeDef *)pinmap_peripheral(pin_a, PinMap_PWM);
    TIM_TypeDef *Instance_b = (TIM_TypeDef *)pinmap_peripheral(pin_b, PinMap_PWM);

    if (Instance_a == NP || Instance_b == NP) {
        // One of the pins is not a PWM pin
        return;
    }

    // For now, we only support both pins on the same timer
    if (Instance_a != Instance_b) {
        return;
    }

    context->htim.Instance = Instance_a;
    context->channel_a = STM_PIN_CHANNEL(pinmap_function(pin_a, PinMap_PWM));
    context->channel_b = STM_PIN_CHANNEL(pinmap_function(pin_b, PinMap_PWM));

    // --- Timer Clock Enable ---
    // This is handled by the pinmap_function, so we don't need to do it manually.


    // --- Timer Configuration ---
    uint32_t uhPrescalerValue = (uint32_t)(SystemCoreClock / pwm_frequency / 65536);
    context->htim.Init.Prescaler = uhPrescalerValue;
    context->htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    context->htim.Init.Period = (SystemCoreClock / (pwm_frequency * (uhPrescalerValue + 1))) - 1;
    context->htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    context->htim.Init.RepetitionCounter = 0;
    context->htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&context->htim);


    // --- PWM Channel Configuration ---
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_PWM_ConfigChannel(&context->htim, &sConfigOC, context->channel_a);
    HAL_TIM_PWM_ConfigChannel(&context->htim, &sConfigOC, context->channel_b);

    // --- GPIO Configuration ---
    pinmap_pinout(pin_a, PinMap_PWM);
    pinmap_pinout(pin_b, PinMap_PWM);

    // --- Start PWM ---
    HAL_TIM_PWM_Start(&context->htim, context->channel_a);
    HAL_TIM_PWM_Start(&context->htim, context->channel_b);
}

void hal_motor_set_duty(int duty_cycle, bool forward, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) {
        return;
    }

    MotorContext* context = &motor_contexts[motor_id];
    uint32_t pulse = (context->htim.Init.Period * duty_cycle) / 255;

    if (forward) {
        __HAL_TIM_SET_COMPARE(&context->htim, context->channel_a, pulse);
        __HAL_TIM_SET_COMPARE(&context->htim, context->channel_b, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&context->htim, context->channel_a, 0);
        __HAL_TIM_SET_COMPARE(&context->htim, context->channel_b, pulse);
    }
}


#endif // ARDUINO_ARCH_STM32
