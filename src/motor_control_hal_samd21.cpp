#include "motor_control_hal.h"

#if defined(ARDUINO_ARCH_SAMD)

#include <Arduino.h>

// Store pin numbers for later use.
static uint8_t g_pwm_a_pin;
static uint8_t g_pwm_b_pin;
static uint8_t g_bemf_a_pin;
static uint8_t g_bemf_b_pin;
static hal_bemf_update_callback_t g_bemf_callback;

void hal_motor_init(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback) {
    g_pwm_a_pin = pwm_a_pin;
    g_pwm_b_pin = pwm_b_pin;
    g_bemf_a_pin = bemf_a_pin;
    g_bemf_b_pin = bemf_b_pin;
    g_bemf_callback = callback;

    pinMode(g_pwm_a_pin, OUTPUT);
    pinMode(g_pwm_b_pin, OUTPUT);
    pinMode(g_bemf_a_pin, INPUT);
    pinMode(g_bemf_b_pin, INPUT);
}

void hal_motor_set_pwm(int duty_cycle, bool forward) {
    if (forward) {
        analogWrite(g_pwm_a_pin, duty_cycle);
        analogWrite(g_pwm_b_pin, 0);
    } else {
        analogWrite(g_pwm_a_pin, 0);
        analogWrite(g_pwm_b_pin, duty_cycle);
    }
}

void hal_read_and_process_bemf() {
    if (g_bemf_callback) {
        int bemf_a = analogRead(g_bemf_a_pin);
        int bemf_b = analogRead(g_bemf_b_pin);
        g_bemf_callback(abs(bemf_a - bemf_b));
    }
}

void hal_motor_deinit() {
    // No explicit de-initialization needed for this simplified implementation.
}

#endif // ARDUINO_ARCH_SAMD
