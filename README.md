# xDuinoRails_MotorControl

`xDuinoRails_MotorControl` is a high-level Arduino library for hardware-accelerated DC motor control. It provides a platform-agnostic interface for PWM control and BEMF (Back-EMF) measurement, with specific implementations for various microcontrollers.

This library is designed for applications that require precise and efficient motor control, such as robotics, CNC machines, and other automated systems.

## Features

- Hardware-accelerated PWM motor control for high-performance applications.
- BEMF measurement for sensorless feedback and control.
- Platform-agnostic interface with support for multiple microcontrollers.
- Non-blocking architecture for efficient use of CPU resources.
- Easy-to-use API for quick integration into Arduino projects.

## Supported Hardware

- ESP32
- nRF52833
- RP2040
- SAMD21
- STM32

## API Reference

The library provides a simple yet powerful API for motor control and BEMF measurement.

### `hal_motor_init(pwm_a_pin, pwm_b_pin, bemf_a_pin, bemf_b_pin, callback)`

Initializes the low-level hardware for motor control.

- `pwm_a_pin`: GPIO pin for PWM channel A.
- `pwm_b_pin`: GPIO pin for PWM channel B.
- `bemf_a_pin`: GPIO pin for ADC input connected to motor terminal A.
- `bemf_b_pin`: GPIO pin for ADC input connected to motor terminal B.
- `callback`: A function pointer to be called with new BEMF data.

### `hal_motor_set_pwm(duty_cycle, forward)`

Sets the motor's PWM duty cycle and direction.

- `duty_cycle`: The desired duty cycle (0-255).
- `forward`: The desired motor direction (`true` for forward, `false` for reverse).

### `hal_bemf_update_callback_t`

A callback function to handle BEMF updates.

- `raw_bemf_value`: The raw, unfiltered differential BEMF value.

## Usage Example

```cpp
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
  // Set the motor to full speed forward
  hal_motor_set_pwm(255, true);
  delay(1000);

  // Set the motor to half speed reverse
  hal_motor_set_pwm(128, false);
  delay(1000);
}
```

## How it Works (ESP32)

The ESP32 implementation of `xDuinoRails_MotorControl` leverages the advanced peripherals of the microcontroller to achieve high-performance motor control with minimal CPU intervention. Here's a high-level overview of how it works:

1. **MCPWM (Motor Control PWM):** The MCPWM peripheral generates the PWM signals for the motor driver, offloading the CPU from this task.

2. **ADC Digital Controller (DMA Mode):** The ADC digital controller, configured in DMA mode, automatically samples the BEMF pins and stores the results in a buffer without any CPU intervention.

3. **GPTimer (General Purpose Timer):** A high-precision GPTimer is used to create a short delay between the PWM pulse and the BEMF measurement, ensuring a clean and accurate reading.

4. **ETM (Event Task Matrix):** The ETM connects the MCPWM, GPTimer, and ADC peripherals, creating a chain of events and tasks that run without any CPU intervention. This allows for a fully automated motor control and BEMF measurement process.
