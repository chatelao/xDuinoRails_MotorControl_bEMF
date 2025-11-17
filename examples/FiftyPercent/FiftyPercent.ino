/**
 * @file FiftyPercent.ino
 * @brief A minimalistic example to run the motor at 50% speed.
 *
 * This sketch demonstrates the simplest way to run the motor. It initializes
 * the hardware abstraction layer and sets the motor's PWM duty cycle to 50%
 * of its maximum value.
 *
 * ## Hardware Setup
 * For detailed wiring instructions, please see the README.md file located in
 * this same directory.
 */

#include <Arduino.h>
#include "motor_control_hal.h"

// --- Pin Definitions ---
#if defined(ARDUINO_SEEED_XIAO_RP2040)
// For Seeed XIAO RP2040
const int MOTOR_PWM_A_PIN = D9;
const int MOTOR_PWM_B_PIN = D10;
const int MOTOR_BEMF_A_PIN = D7;
const int MOTOR_BEMF_B_PIN = D8;
#else
// Default pins for other boards
const int MOTOR_PWM_A_PIN = 7;
const int MOTOR_PWM_B_PIN = 8;
const int MOTOR_BEMF_A_PIN = A3;
const int MOTOR_BEMF_B_PIN = A2;
#endif

void setup() {
  // Start serial communication for debugging output.
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for the serial port to connect. Needed for native USB port only.
  }
  Serial.println("Fifty Percent Motor Example");

  // Initialize the motor hardware abstraction layer.
  // We provide nullptr for the BEMF callback as we don't need it.
  hal_motor_init(MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN, MOTOR_BEMF_A_PIN, MOTOR_BEMF_B_PIN, nullptr);

  // Set motor speed to 50% (127 out of 255) in the forward direction.
  const int fifty_percent_speed = 127;
  const bool forward_direction = true;
  hal_motor_set_pwm(fifty_percent_speed, forward_direction);

  Serial.println("Motor running at 50% speed.");
}

void loop() {
  // Nothing to do here. The motor speed is set once in setup().
}
