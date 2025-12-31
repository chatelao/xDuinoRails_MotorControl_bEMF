/**
 * @file MinimalDcc.ino
 * @brief A minimal example for DCC or Märklin Motorola control.
 *
 * This sketch allows you to control a motor using DCC or Märklin Motorola commands.
 * The locomotive address is set to 3.
 *
 * This example is based on the MotorSerialDebug example.
 *
 * ## Hardware Setup
 * For detailed wiring instructions, please see the README.md file located in
 * this same directory.
 */

#include <Arduino.h>
#include <motor_control_hal.h>

#if defined(USE_NMRA_DCC)
#include <NmraDcc.h>
#elif defined(USE_MAERKLIN_MOTOROLA)
#include <MaerklinMotorola.h>
#endif

#if defined(USE_NMRA_DCC)
// --- DCC Settings ---
#define DECODER_ADDRESS 3

// Create a DCC object
NmraDcc Dcc;
#elif defined(USE_MAERKLIN_MOTOROLA)
MaerklinMotorola Motorola;
#endif

// Global variables for motor state
int current_speed = 0;
bool current_direction = true;

#if defined(USE_NMRA_DCC)
// --- DCC Callback Function ---
void notifyDccSpeed(uint16_t Addr, DCC_SPEED_COMMAND Speed) {
  if (Addr == DECODER_ADDRESS) {
    // Map DCC speed steps (0-126) to PWM duty cycle (0-255)
    int speed_val = Speed.getSpeed();
    if (speed_val > 0) {
      // Map 1-126 to a usable PWM range, e.g., 50-255 to avoid motor stall
      // For simplicity, we'll do a direct linear mapping first.
      // 126 steps -> 255 PWM. Multiply by 2.
      current_speed = min(speed_val * 2, 255);
    } else {
      current_speed = 0;
    }
    current_direction = Speed.getDirection();
    hal_motor_set_duty(current_speed, current_direction);

    Serial.print("DCC Speed Command: Addr=");
    Serial.print(Addr);
    Serial.print(", Speed=");
    Serial.print(speed_val);
    Serial.print(", Dir=");
    Serial.println(current_direction);
  }
}
#endif

#if defined(USE_MAERKLIN_MOTOROLA)
// --- Märklin Motorola Callback Function ---
void Motorola_Event(const unsigned char address, const unsigned char data) {
  if (address == DECODER_ADDRESS) {
    // MM2-protocol for locomotives has 14 speedsteps
    // data bits: xxxxdddd
    // dddd: 0000=stop, 0001=step1, ... 1110=step14, 1111=change of direction
    if (data == 0x0F) { // Change of direction
      current_direction = !current_direction;
    } else {
      current_speed = map(data, 0, 14, 0, 255);
    }
    hal_motor_set_duty(current_speed, current_direction);

    Serial.print("Motorola Event: Addr=");
    Serial.print(address);
    Serial.print(", Data=");
    Serial.print(data, HEX);
    Serial.print(", Speed=");
    Serial.print(current_speed);
    Serial.print(", Dir=");
    Serial.println(current_direction);
  }
}
#endif

// --- Pin Definitions ---
#if defined(ARDUINO_SEEED_XIAO_RP2040)
    // For standard Seeed XIAO RP2040
    const int MOTOR_PWM_A_PIN       =  D9;
    const int MOTOR_PWM_B_PIN       = D10;
    const int MOTOR_BEMF_A_PIN      =  D7;
    const int MOTOR_BEMF_B_PIN      =  D8;
    const int DCC_PIN               =  D1;
#else
// Default pins for other boards
    const int MOTOR_PWM_A_PIN       =   7;
    const int MOTOR_PWM_B_PIN       =   8;
    const int MOTOR_BEMF_A_PIN      =  A3;
    const int MOTOR_BEMF_B_PIN      =  A2;
    const int DCC_PIN               =   2;
#endif

void setup() {
  Serial.begin(115200);
  Serial.println("MinimalDcc Example");

  // Initialize the motor hardware abstraction layer.
  hal_motor_init_pwm(MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN);
  hal_motor_set_duty(0, true);

#if defined(USE_NMRA_DCC)
  Dcc.init(DCC_PIN, 0);
  Dcc.setAddr(DECODER_ADDRESS);
#elif defined(USE_MAERKLIN_MOTOROLA)
  Motorola.init(DCC_PIN);
  Motorola.setEvent(Motorola_Event);
#endif
}

void loop() {
#if defined(USE_NMRA_DCC)
  Dcc.process();
#elif defined(USE_MAERKLIN_MOTOROLA)
  Motorola.process();
#endif
}
