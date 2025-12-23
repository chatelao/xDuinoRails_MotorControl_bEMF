#ifndef MOTOR_CONTROL_HAL_RP2040_INTERNAL_H
#define MOTOR_CONTROL_HAL_RP2040_INTERNAL_H

#include "motor_control_hal.h"

#ifdef ARDUINO_ARCH_RP2040

#include <Arduino.h>
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/irq.h"

// =============================================================================
// Constants & Magic Values
// =============================================================================
const uint32_t RP2040_SYSTEM_CLOCK_HZ   = 125000000;
const int      ADC_RESOLUTION_BITS  = 12;
const float    ADC_MAX_VALUE        = 4095.0f;
const float    ADC_REF_VOLTAGE      = 3.3f;
const uint     MOTOR_ADC_BASE_PIN   = 26;
const uint     ADC_FIFO_THRESHOLD   = 1;
const uint16_t PWM_MAX_TOP          = 65535;
const float    PWM_MAX_DIVIDER      = 255.0f;
const int      PWM_DUTY_MIN         = 0;
const int      PWM_DUTY_MAX         = 255;
const uint16_t PWM_DEAD_TIME_CYCLES = 50;

// =============================================================================
// Data Structures
// =============================================================================
struct MotorContext;

struct AlarmUserData {
    MotorContext* ctx;
};

struct MotorContext {
    bool     is_initialized;
    bool     skip_measurement;
    uint8_t  pwm_a_pin;
    uint8_t  pwm_b_pin;
    uint8_t  bemf_a_pin;
    uint8_t  bemf_b_pin;
    uint     motor_pwm_slice_a;
    uint     motor_pwm_slice_b;
    uint16_t pwm_wrap_value;
    uint16_t next_duty_level_a;
    uint16_t next_duty_level_b;
    uint32_t pwm_frequency;
    int      dma_channel_bemf;
    volatile uint16_t          bemf_ring_buffer[BEMF_RING_BUFFER_SIZE];
    hal_bemf_update_callback_t bemf_callback;
    AlarmUserData               alarm_user_data;
};

extern MotorContext g_motors[MAX_MOTORS];
extern volatile bool g_adc_busy;

// =============================================================================
// Forward Declarations
// =============================================================================
static void adc_init_common();
static void trigger_adc_measurement(MotorContext* ctx);
static void dma_bemf_start_capture(MotorContext* ctx);
static void on_pwm_wrap();
#ifdef USE_IRQ_TRIGGER
    static void pwm_init_irq(MotorContext* ctx);
#endif
static void pwm_init(MotorContext* ctx);
static void pwm_set_duty_level(MotorContext* ctx);

#endif // ARDUINO_ARCH_RP2040
#endif // MOTOR_CONTROL_HAL_RP2040_INTERNAL_H
