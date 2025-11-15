/**
 * @file motor_control_hal_esp32.cpp
 * @brief ESP32-specific implementation for the motor control HAL.
 */

#include "motor_control_hal.h"

#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/adc.h"

// PWM frequency for the motor driver
const uint32_t PWM_FREQUENCY_HZ = 25000;

// --- Static Globals for Hardware Control ---
static hal_bemf_update_callback_t bemf_callback = nullptr;

// Pin definitions
static uint8_t g_pwm_a_pin;
static uint8_t g_pwm_b_pin;
static uint8_t g_bemf_a_pin;
static uint8_t g_bemf_b_pin;

void hal_motor_init(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback) {
    g_pwm_a_pin = pwm_a_pin;
    g_pwm_b_pin = pwm_b_pin;
    g_bemf_a_pin = bemf_a_pin;
    g_bemf_b_pin = bemf_b_pin;
    bemf_callback = callback;

    // --- MCPWM Setup ---
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, g_pwm_a_pin);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, g_pwm_b_pin);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQUENCY_HZ;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    // --- ADC Setup ---
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten((adc1_channel_t)digitalPinToAnalogChannel(g_bemf_a_pin), ADC_ATTEN_DB_11);
    adc1_config_channel_atten((adc1_channel_t)digitalPinToAnalogChannel(g_bemf_b_pin), ADC_ATTEN_DB_11);
}

void hal_motor_set_pwm(int duty_cycle, bool forward) {
    float duty_percent = (float)duty_cycle / 255.0f * 100.0f;
    if (forward) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_percent);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);

    } else {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_percent);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
}

void hal_read_and_process_bemf() {
    int adc_val_a = adc1_get_raw((adc1_channel_t)digitalPinToAnalogChannel(g_bemf_a_pin));
    int adc_val_b = adc1_get_raw((adc1_channel_t)digitalPinToAnalogChannel(g_bemf_b_pin));

    if (bemf_callback) {
        bemf_callback(abs(adc_val_a - adc_val_b));
    }
}


void hal_motor_deinit() {
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
}

#endif // ARDUINO_ARCH_ESP32
