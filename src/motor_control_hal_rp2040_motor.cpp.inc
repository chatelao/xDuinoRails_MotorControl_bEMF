#include "motor_control_hal_rp2040_internal.h"

#if defined(ARDUINO_ARCH_RP2040)

// =============================================================================
// Global Variables
// =============================================================================
MotorContext g_motors[MAX_MOTORS];
volatile bool g_adc_busy = false;

// =============================================================================
// Public HAL Function Implementations
// =============================================================================

void hal_motor_init_pwm(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint32_t pwm_frequency, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return;
    MotorContext* ctx = &g_motors[motor_id];

    ctx->pwm_frequency = pwm_frequency;
    ctx->pwm_a_pin = pwm_a_pin;
    ctx->pwm_b_pin = pwm_b_pin;
    pwm_init(ctx);
}

void hal_motor_init_bemf_adc(uint8_t bemf_a_pin, uint8_t bemf_b_pin, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return;
    MotorContext* ctx = &g_motors[motor_id];
    if (!ctx->is_initialized) return;

    ctx->dma_channel_bemf = -1;
    ctx->bemf_a_pin = bemf_a_pin;
    ctx->bemf_b_pin = bemf_b_pin;

    bool bemf_enabled = (bemf_a_pin != MOTOR_PIN_UNDEFINED) && (bemf_b_pin != MOTOR_PIN_UNDEFINED);
    if (!bemf_enabled) return;

    static bool adc_initialized = false;
    if (!adc_initialized) {
        adc_init();
        adc_fifo_setup(true, true, ADC_FIFO_THRESHOLD, false, false);
        adc_initialized = true;
    }

    adc_gpio_init(ctx->bemf_a_pin);
    adc_gpio_init(ctx->bemf_b_pin);
}

void hal_motor_set_duty(int duty_cycle, bool forward, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) return;
    MotorContext* ctx = &g_motors[motor_id];
    if (!ctx->is_initialized) return;

    uint16_t on_level  = map(duty_cycle, PWM_DUTY_MIN, PWM_DUTY_MAX, ctx->pwm_wrap_value, 0);
    uint16_t off_level = PWM_MAX_TOP;

    ctx->next_duty_level_a = forward ? on_level : off_level;
    ctx->next_duty_level_b = forward ? off_level : on_level;

    if (ctx->motor_pwm_slice_a == ctx->motor_pwm_slice_b) {
        pwm_set_duty_level(ctx);
    }
}

#endif // ARDUINO_ARCH_RP2040
