#include <Arduino.h>
#include "motor_control_hal.h"

// Define the GPIO pins for the motor driver
// IMPORTANT: These pins are placeholders. You must update them to match your
// specific hardware setup.
const uint8_t PWM_A_PIN = 25; // Example pin, change to your setup
const uint8_t PWM_B_PIN = 26; // Example pin, change to your setup
const uint8_t BEMF_A_PIN = 32; // Example pin, change to your setup
const uint8_t BEMF_B_PIN = 33; // Example pin, change to your setup

// BEMF callback function (can be left empty for a minimal example)
void on_bemf_update(int raw_bemf_value) {
  // This function is called when new BEMF data is available.
  // For this minimal example, we don't need to do anything with it.
}

void setup() {
  // Initialize the motor control hardware. This function must be called
  // once during setup. It configures the PWM, ADC, and other peripherals
  // required for motor control.
  hal_motor_init(PWM_A_PIN, PWM_B_PIN, BEMF_A_PIN, BEMF_B_PIN, on_bemf_update);
}

void loop() {
  // Set the motor to full speed forward. The duty cycle is set to 255,
  // which corresponds to the maximum possible speed. The 'true' argument
  // indicates a forward direction.
  hal_motor_set_pwm(255, true);
  delay(1000); // Wait for 1 second

  // Stop the motor. A duty cycle of 0 will stop the motor. The direction
  // doesn't matter in this case, but we'll keep it as 'true'.
  hal_motor_set_pwm(0, true);
  delay(1000); // Wait for 1 second

  // Set the motor to half speed reverse. The duty cycle is set to 128,
  // which is approximately half of the maximum speed. The 'false' argument
  // indicates a reverse direction.
  hal_motor_set_pwm(128, false);
  delay(1000); // Wait for 1 second

  // Stop the motor again before the next cycle.
  hal_motor_set_pwm(0, false);
  delay(1000); // Wait for 1 second
}
