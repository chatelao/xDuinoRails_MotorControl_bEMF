# xDuinoRails Motor Control bEMF HAL

A hardware abstraction layer (HAL) for PlatformIO, designed for controlling DC motors with high precision. This library provides a low-level interface for PWM motor control and back-EMF (BEMF) sensing. This library is ideal for projects like model trains, robotics, or any application that requires direct hardware control.

## Documentation

*   [How to Use](docs/HOW_TO_USE.en.md)
*   [User Manual](docs/USER_MANUAL.en.md)
*   [Core Concepts](docs/CORE_CONCEPTS.en.md)
*   [Developer Reference](docs/DEVELOPER_REFERENCE.en.md)
*   [Hardware Control Details](docs/HARDWARE_CONTROL.en.md)
*   [BEMF Measurement Techniques](docs/BEMF_MEASUREMENT_TECHNIQUES.en.md)
*   [PWM Frequencies](docs/PWM_FREQUENCIES.en.md)
*   [Motor Driver ICs](docs/MOTOR_DRIVER_ICS.en.md)

### Supported Hardware
*   [Seeed Studio XIAO RP2040](docs/SEEED_XIAO_RP2040.en.md)

## Features

*   **Low-Level Control:** Direct access to hardware peripherals for PWM and ADC.
*   **Back-EMF Sensing:** Accurately measures motor speed without the need for an external encoder.
*   **Platform Agnostic:** Defines a common interface that can be implemented for various microcontrollers.
*   **Callback-Based:** Uses a callback function to provide real-time BEMF data.
*   **Arduino and PlatformIO Compatible:** Works seamlessly with both development environments.

## Installation

### PlatformIO

1.  Add this repository to the `lib_deps` section of your `platformio.ini` file:
    ```ini
    lib_deps =
        https://github.com/OpenRailAssociation/xDuinoRails_MotorControl_bEMF.git
    ```
2.  PlatformIO will automatically download and install the library the next time you build your project.

## Wiring Diagram

This diagram shows a typical wiring setup using a Seeed Studio XIAO RP2040 and a BDR-6133 motor driver.

```
                     +--------------------+      +--------------------+         +---------------+
                     |      RP2040        |      |     BDR-6133       |         |     Motor     |
                     |    (Top View)      |      |    Motor Driver    |         | DC brushed    |
                     +--------------------+      +--------------------+         +---------------+
                     |                5v  |      |                    |         |               |
                     |                GND |      |                    |         |               |
                     |                3v3 |      |                    |         |               |
                     |        (PWM B) D8  |----->| InB           OutB |=====+==>| B (-> D8)     |
                     |        (PWM A) D7  |----->| InA           OutA |==+==|==>| A (-> D7)     |
                     |                    |      +---------+----------+  |  |   +---------------+
                     |                    |                |             |  |
                     |       (Shunt)  A0  |<---------------/             |  |
                     |       (bEMF B) A1  |<----------------------------/   |
                     |       (bEMF A) A2  |<-------------------------------/
                     +--------------------+
```

## Getting Started

This simple example demonstrates how to get your motor up and running.

```cpp
#include <Arduino.h>
#include "motor_control_hal.h"

// 1. Define Pin Connections
// These pins are for the Seeed XIAO RP2040 (Standard Edition)
const int MOTOR_PWM_A_PIN = D9;
const int MOTOR_PWM_B_PIN = D10;
const int MOTOR_BEMF_A_PIN = D7;
const int MOTOR_BEMF_B_PIN = D8;

// 2. BEMF Callback Function
// This function is called from an interrupt whenever a new BEMF value is available.
void on_bemf_update(int raw_bemf) {
  // In a real application, you would filter and process this data.
  // Note: Serial printing in ISR is not recommended for production, only for simple debug.
  // Serial.println(raw_bemf);
}

void setup() {
  Serial.begin(115200);

  // 3. Initialize the motor hardware abstraction layer
  hal_motor_init(MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN, MOTOR_BEMF_A_PIN, MOTOR_BEMF_B_PIN, on_bemf_update);

  // 4. Set a motor speed and direction
  // The duty_cycle is a value between 0 and 255.
  // The second parameter is the direction (true for forward, false for reverse).
  hal_motor_set_pwm(128, true);
}

void loop() {
  // The main loop can be used for other tasks.
  // The BEMF reading and motor control are handled by interrupts.
}
```

You can find this and other examples in the `examples` folder of this repository.

## API Reference

### `void hal_motor_init(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback)`

Initializes the low-level hardware for motor control. This function configures the hardware timers, PWM peripherals, ADC, and DMA for hardware-accelerated motor control and BEMF measurement. It must be called once during the application's setup phase.

### `void hal_motor_set_pwm(int duty_cycle, bool forward)`

Sets the motor's PWM duty cycle and direction. This function updates the PWM hardware with the new duty cycle. It should be called periodically from the main application loop to reflect the latest output from the motor control algorithm (e.g., a PI controller).

### `int hal_motor_get_bemf_buffer(volatile uint16_t** buffer, int* last_write_pos)`

Retrieves the BEMF ring buffer for diagnostics. This function provides low-level access to the raw ADC sample buffer. It is intended for debugging and visualization, not for real-time control.

### `int hal_motor_get_current_buffer(volatile uint16_t** buffer, int* last_write_pos)`

Retrieves the optional Current Sensing ring buffer for diagnostics. This function works similarly to `hal_motor_get_bemf_buffer` but for current sensing data, if the hardware supports it.

### `void hal_motor_check_solenoid_position(int ping_pwm_value, int ping_duration_ms, int measurement_delay_us, int* response_a, int* response_b)`

Performs a diagnostic test to check the motor/solenoid position. It pings the motor in both directions and measures the BEMF response, allowing for basic position estimation or connection verification.

### `hal_bemf_update_callback_t`

This is a `typedef` for a function pointer that is used to handle BEMF updates. A function matching this signature must be passed to `hal_motor_init`. This callback is executed from an interrupt context whenever a new differential BEMF measurement is available from the hardware.

**Signature:**
```cpp
void your_callback_function_name(int raw_bemf_value);
```
-   `raw_bemf_value`: The raw, unfiltered differential BEMF value, calculated as the absolute difference between the two ADC readings.

**Example:**
```cpp
void on_bemf_update(int raw_bemf) {
  // IMPORTANT: Keep this function short and fast as it runs in an interrupt.
  // Perform filtering, processing, and control logic adjustments here.
  // volatile int latest_bemf = raw_bemf;
}
```
