#ifndef MOTOR_CONTROL_HAL_RP2040_INTERNAL_H
#define MOTOR_CONTROL_HAL_RP2040_INTERNAL_H

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
enum MeasurementType {
    BEMF,
    SHUNT
};

struct MotorContext;

struct AlarmUserData {
    MotorContext* ctx;
    MeasurementType type;
};

struct MotorContext {
    bool     is_initialized;
    uint8_t  pwm_a_pin;
    uint8_t  pwm_b_pin;
    uint8_t  bemf_a_pin;
    uint8_t  bemf_b_pin;
    uint8_t  shunt_a_pin;
    uint8_t  shunt_b_pin;
    uint     motor_pwm_slice_a;
    uint     motor_pwm_slice_b;
    uint16_t pwm_wrap_value;
    uint16_t next_level_a;
    uint16_t next_level_b;
    uint32_t pwm_frequency;
    int      dma_channel_bemf;
    volatile uint16_t          bemf_ring_buffer[BEMF_RING_BUFFER_SIZE];
    hal_bemf_update_callback_t bemf_callback;
    int                        dma_channel_shunt;
    volatile uint16_t          shunt_ring_buffer[SHUNT_RING_BUFFER_SIZE];
    hal_shunt_update_callback_t shunt_callback;
    volatile uint32_t           adc_trigger_delay_us;
    volatile bool               skip_measurement;
    AlarmUserData               alarm_user_data;
};

extern MotorContext g_motors[MAX_MOTORS];
extern volatile bool g_adc_busy;

// =============================================================================
// Forward Declarations
// =============================================================================
static void trigger_adc_measurement(MotorContext* ctx, MeasurementType type);
static void dma_irq_handler();
static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data);
static void on_pwm_wrap();
static void pwm_init_common(MotorContext* ctx);
static void pwm_set_motor_levels(MotorContext* ctx, uint16_t level_a, uint16_t level_b);

#endif // ARDUINO_ARCH_RP2040
#endif // MOTOR_CONTROL_HAL_RP2040_INTERNAL_H
