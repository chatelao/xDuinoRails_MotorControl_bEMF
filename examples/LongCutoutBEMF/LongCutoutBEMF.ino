#include <motor_control_hal.h>

// Pin definitions for Seeed XIAO RP2040
const int MOTOR_PWM_A_PIN = D9;
const int MOTOR_PWM_B_PIN = D10;
const int MOTOR_BEMF_A_PIN = D7;
const int MOTOR_BEMF_B_PIN = D8;

// Low PWM frequency suitable for the long-cutout mode
const int PWM_FREQUENCY_HZ = 50;

// Callback function to handle BEMF updates
void bemf_callback(int raw_bemf_value) {
  Serial.print("Average BEMF: ");
  Serial.println(raw_bemf_value);
}

void setup() {
  Serial.begin(115200);

  // Wait for Serial to connect, with a timeout for battery-powered operation
  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime < 2000));

  Serial.println("Long-Cutout BEMF Measurement Example");

  // 1. Initialize PWM with a low frequency
  hal_motor_init_pwm(MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN, PWM_FREQUENCY_HZ);

  // 2. Initialize the ADC pins for BEMF measurement
  hal_motor_init_bemf_adc(MOTOR_BEMF_A_PIN, MOTOR_BEMF_B_PIN);

  // 3. Initialize the timer-based BEMF measurement mode
  //    - Callback: bemf_callback
  //    - Delay: 1500us (default)
  //    - Samples: 3 (default)
  hal_motor_init_bemf_timer_adc(bemf_callback);

  // 4. Start the motor at 50% duty cycle
  // Duty cycle (0-255), forward (true/false)
  hal_motor_set_duty(128, true);
  Serial.println("Motor started at 50% duty cycle.");
}

void loop() {
  // Everything is handled by interrupts, so the main loop can be empty
  // or used for other tasks.
  delay(1000);
}
