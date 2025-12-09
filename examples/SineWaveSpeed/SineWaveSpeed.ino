/**
 * @file SineWaveSpeed.ino
 * @brief An example of varying motor speed in a sine wave pattern.
 *
 * This sketch demonstrates how to control the motor's speed to follow a sine
 * wave, varying between 0% and 75% of its maximum speed over a period of 2.5
 * seconds.
 *
 * ## How it Works
 * - The motor speed is modulated using a sine function.
 * - The sine wave is scaled to have a minimum of 0% and a maximum of 75% duty
 *   cycle.
 * - One full cycle of the sine wave (from 0% up to 75% and back to 0%) is
 *   completed every 2.5 seconds.
 */

#include <Arduino.h>
#include "motor_control_hal.h"

#if defined(ARDUINO_SEEED_XIAO_RP2040)

    // --- Neopixel LED ---
    #include <Adafruit_NeoPixel.h>
    #define NEOPIXEL_DATA_PIN  12   // Data pin for on-bard neopixel
    #define NEOPIXEL_POWER_PIN 11   // Power-On pin for on-bard neopixel
    #define NEOPIXEL_BIGHTNESS 10   // Keep the NEOPIXEL just dark enough

    Adafruit_NeoPixel pixels(1, NEOPIXEL_DATA_PIN, NEO_GRB + NEO_KHZ800);


    #ifndef LED_EDITION
        // For standard Seeed XIAO RP2040
        const int MOTOR_PWM_A_PIN       =  D9;
        const int MOTOR_PWM_B_PIN       = D10;

        const int MOTOR_BEMF_A_PIN      = A2; // GPIO28
        const int MOTOR_BEMF_B_PIN      = A3; // GPIO29
    #else
        // For Seeed XIAO RP2040 "LED Edition"
        const int MOTOR_PWM_A_PIN       = 17; // LED Red   , PWM slice 0, channel A
        const int MOTOR_PWM_B_PIN       = 16; // LED Green , PWM slice 0, channel B

        const int MOTOR_BEMF_A_PIN      = A2; // GPIO28
        const int MOTOR_BEMF_B_PIN      = A3; // GPIO29

        const int THIRD_LED_PIN         = 25; // LED Blue  , just turn dark please
    #endif
#else
// Default pins for other boards
const int MOTOR_PWM_A_PIN       =   7;
const int MOTOR_PWM_B_PIN       =   8;
const int MOTOR_BEMF_A_PIN      =  A3;
const int MOTOR_BEMF_B_PIN      =  A2;
#endif

// --- Sine Wave Parameters ---
const float SINE_WAVE_PERIOD    = 2500; // 2.5 seconds in milliseconds
const int MIN_PWM_DUTY_CYCLE    =    0;   // 0% of 255
const int MAX_PWM_DUTY_CYCLE    =  191;  // 75% of 255
bool motorDirection             = true;   // Motor direction: true for forward

void setup() {
  Serial.begin(115200);
  Serial.println("Sine Wave Motor Speed Control Example");

  // Initialize the motor hardware abstraction layer.
  hal_motor_init(MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN, MOTOR_BEMF_A_PIN, MOTOR_BEMF_B_PIN, NULL);

#if defined(ARDUINO_SEEED_XIAO_RP2040)

  #ifdef LED_EDITION
    pinMode(THIRD_LED_PIN, OUTPUT);
    digitalWrite(THIRD_LED_PIN, HIGH);
  #endif

  // Initialize Neopixel
  pinMode(NEOPIXEL_POWER_PIN, OUTPUT);
  digitalWrite(NEOPIXEL_POWER_PIN, HIGH);

  delay(10); // Wait for power to stabilize
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(0, 0, NEOPIXEL_BIGHTNESS)); // Blue to indicate SineWave mode
  pixels.show();
#endif
}

void loop() {
  // Calculate the elapsed time.
  float elapsedTime = millis();

  // Calculate the sine wave value.
  float sineValue = sin(2 * PI * (elapsedTime / SINE_WAVE_PERIOD));

  // Map the sine value (-1 to 1) to the direction and PWM range (MIN_PWM to MAX_PWM).
  motorDirection = sineValue > 0.0;  // Motor direction: true for forward
  int pwmValue = map( abs(sineValue) * 1000, 0, 1000, MIN_PWM_DUTY_CYCLE, MAX_PWM_DUTY_CYCLE);

  // Set the motor PWM.
  hal_motor_set_pwm(pwmValue, motorDirection);

#if defined(ARDUINO_SEEED_XIAO_RP2040)
  // Indicate motor direction with color (Green for Forward, Red for Backward)
  if (motorDirection) {
    pixels.setPixelColor(0, pixels.Color(0, NEOPIXEL_BIGHTNESS, 0));
  } else {
    pixels.setPixelColor(0, pixels.Color(NEOPIXEL_BIGHTNESS, 0, 0));
  }
  pixels.show();
#endif

  // Print the current PWM value for debugging.
  Serial.print("PWM: ");
  Serial.println(pwmValue);

  // A small delay to prevent spamming the serial port.
  delay(10);
}
