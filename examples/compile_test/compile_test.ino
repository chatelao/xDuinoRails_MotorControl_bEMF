#include <Arduino.h>
#include "motor_control_hal.h"

// Define the GPIO pins for the motor driver
const uint8_t PWM_A_PIN = 25;
const uint8_t PWM_B_PIN = 26;
const uint8_t BEMF_A_PIN = 32;
const uint8_t BEMF_B_PIN = 33;

// BEMF callback function
void on_bemf_update(int raw_bemf_value) {
  // Process the BEMF data
  Serial.printf("BEMF: %d\n", raw_bemf_value);
}

void setup() {
  Serial.begin(115200);

  // Initialize the motor control hardware
  hal_motor_init(PWM_A_PIN, PWM_B_PIN, BEMF_A_PIN, BEMF_B_PIN, on_bemf_update);
}

void loop() {
  // Read and process BEMF data
  hal_read_and_process_bemf();

  // Set the motor to full speed forward
  hal_motor_set_pwm(255, true);
  delay(1000);

  // Set the motor to half speed reverse
  hal_motor_set_pwm(128, false);
  delay(1000);
}
