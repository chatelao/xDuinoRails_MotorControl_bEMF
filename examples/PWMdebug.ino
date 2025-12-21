#include <Arduino.h>
#include "hardware/pwm.h"

const uint32_t pwm_frequency            =     20000;

// Same PWM slice test
// const uint8_t  pwm_a_pin                =        D6;
// const uint8_t  pwm_b_pin                =        D7;

// Different PWM slice test
const uint8_t  pwm_a_pin                =        D6;
const uint8_t  pwm_b_pin                =        D8;

// =============================================================================
// Constants & Magic Values
// =============================================================================
uint     motor_pwm_slice_a;
uint     motor_pwm_slice_b;

const uint16_t PWM_MAX_TOP              =     65535;
const float    PWM_MAX_DIVIDER          =    255.0f;
const int      PWM_DUTY_MIN             =         0;
const int      PWM_DUTY_MAX             =       255;
const uint16_t PWM_DEAD_TIME_CYCLES     =        50;

uint16_t next_level_a = PWM_MAX_TOP; // Inverted Off
uint16_t next_level_b = PWM_MAX_TOP; // Inverted Off

bool     is_initialized;
uint16_t pwm_wrap_value;

#define PULSE_PIN D0

// Die ISR muss so schlank wie möglich sein, läuft im RAM
void __not_in_flash_func(on_pwm_wrap)() {
    
    pwm_clear_irq(pwm_gpio_to_slice_num(pwm_a_pin));

    if (motor_pwm_slice_a != motor_pwm_slice_b) {
        pwm_set_gpio_level(pwm_a_pin, next_level_a);
        pwm_set_gpio_level(pwm_b_pin, next_level_b);
    }
    
    // Kurzen Puls erzeugen
    gpio_put(PULSE_PIN, 1);
    asm volatile("nop \n nop \n nop");     
    gpio_put(PULSE_PIN, 0);
}

static uint16_t pwm_set_frequency(pwm_config *pwm_conf, uint32_t pwm_frequency) {

    uint32_t total_div      = F_CPU / pwm_frequency / 2;
    float    clk_divider    =                      1.0f;
    uint16_t pwm_wrap_value =                         0;
    
    if (total_div > PWM_MAX_TOP) {  // PWM_MAX_TOP: 65535.0f
        // Frequenz ist so niedrig, dass wir den Clock Divider brauchen
        clk_divider = total_div / PWM_MAX_TOP;
        
        // Hardware-Limit des Dividers prüfen (max 255.9375)
        if (clk_divider > PWM_MAX_DIVIDER) clk_divider = PWM_MAX_DIVIDER; 
        
        pwm_wrap_value = PWM_MAX_TOP;
    } else if (total_div < 1.0f) {
        // Frequenz ist zu hoch für den Systemtakt
        clk_divider = 1.0f;
        pwm_wrap_value = 1; 
    } else {
        // Frequenz passt ohne Divider -> Maximale Auflösung
        clk_divider = 1.0f;
        pwm_wrap_value = (uint16_t)total_div;
    }

    pwm_config_set_clkdiv( pwm_conf, clk_divider);
    pwm_config_set_wrap(   pwm_conf, pwm_wrap_value);

    return pwm_wrap_value;

}

static uint pwm_init_pin(uint8_t pwm_pin) {

    gpio_set_function( pwm_pin, GPIO_FUNC_PWM);             // Pin   - Use it with PWM mode
    gpio_set_outover(  pwm_pin, GPIO_OVERRIDE_INVERT);      // Pin   - Invert Output
    pwm_set_gpio_level(pwm_pin, PWM_MAX_TOP);               // Pin   - Setup Output to OFF (after inversion)
    uint motor_pwm_slice = pwm_gpio_to_slice_num(pwm_pin);  // Slice - Find the slice of the pin
    pwm_set_counter(motor_pwm_slice, 0);                    // Slice - Reset the slice counter to zero to sync all PWM

    return motor_pwm_slice;
}

static void pwm_init_simple() {

    motor_pwm_slice_a = pwm_init_pin(pwm_a_pin);
    motor_pwm_slice_b = pwm_init_pin(pwm_b_pin);
    
    pwm_config motor_pwm_conf = pwm_get_default_config();
    pwm_wrap_value = pwm_set_frequency( &motor_pwm_conf, pwm_frequency);
    pwm_config_set_phase_correct(       &motor_pwm_conf, true);

    pwm_init(motor_pwm_slice_a,   &motor_pwm_conf, false);
    pwm_init(motor_pwm_slice_b,   &motor_pwm_conf, false);

    // 1. Interrupt für diesen spezifischen PWM-Slice aktivieren
    pwm_set_irq_enabled(       motor_pwm_slice_a, true );
    irq_set_exclusive_handler( PWM_IRQ_WRAP, on_pwm_wrap );
    irq_set_enabled(           PWM_IRQ_WRAP, true );

    uint32_t mask = (1u << motor_pwm_slice_a) | (1u << motor_pwm_slice_b);
    hw_set_bits(&pwm_hw->en, mask);
}

void pwm_set_duty(int duty_cycle, bool forward) {

    uint16_t on_level  = map(duty_cycle, 0, 255, pwm_wrap_value, 0);
    uint16_t off_level = PWM_MAX_TOP;

    next_level_a = forward ? on_level : off_level;
    next_level_b = forward ? off_level : on_level;

    if (motor_pwm_slice_a == motor_pwm_slice_b) {
        pwm_set_both_levels(motor_pwm_slice_a, next_level_a, next_level_b);
    }
}

void setup() {
  
  // Puls-Pin initialisieren
  gpio_init(PULSE_PIN);
  gpio_set_dir(PULSE_PIN, GPIO_OUT);

  pwm_init_simple();
  pwm_set_duty(100, false);

  // pwm_set_gpio_level(pwm_a_pin, 100);
  // pwm_set_gpio_level(pwm_b_pin,  50);

}

void loop() {

}
