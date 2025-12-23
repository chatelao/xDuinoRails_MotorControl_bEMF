# xDuinoRails Motor Control bEMF HAL

A hardware abstraction layer (HAL) for PlatformIO, designed for controlling DC motors with high precision. This library provides a low-level interface for PWM motor control and back-EMF (BEMF) sensing. This library is ideal for projects like model trains, robotics, or any application that requires direct hardware control.

## Documentation

Our full documentation is available [here](https://OpenRailAssociation.github.io/xDuinoRails_MotorControl_bEMF/).

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

## Supported Platforms

### Raspberry Pi RP2040

The HAL is implemented for the Raspberry Pi RP2040, and has been tested with the Seeed Studio XIAO RP2040.

**Wiring Diagram**

This diagram shows a typical wiring setup using a Seeed Studio XIAO RP2040 and a BDR-6133 motor driver.

```
                     +--------------------+      +--------------------+         +---------------+
                     |      RP2040        |      |     BDR-6133       |         |     Motor     |
                     |    (Top View)      |      |    Motor Driver    |         | DC brushed    |
                     +--------------------+      +--------------------+         +---------------+
                     |                5v  |      |                    |         |               |
          ---``|<----| D15 (LED B)    GND |      |                    |         |               |
          ---``|<----| D16 (LED A)    3v3 |      |                    |         |               |
                     |                    |      |                    |         |               |
                     |        (PWM B) D8  |----->| InB           OutB |=====+==>| B (-> D8)     |
                     |        (PWM A) D7  |----->| InA           OutA |==+==|==>| A (-> D7)     |
                     |                    |      +---------+----------+  |  |   +---------------+
                     |                    |                |             |  |
                     |       (bEMF B) A1  |<----------------------------/   |
                     |       (bEMF A) A2  |<-------------------------------/
                     +--------------------+
```

## How it Works

The RP2040 implementation leverages the RP2040's powerful peripherals to achieve CPU-free motor control.

*   **PWM:** Two PWM slices are used to generate the motor control signals.
*   **ADC:** The ADC is used to sample the BEMF voltage from the motor.
*   **DMA:** A DMA channel is used to transfer the ADC readings to a ring buffer in memory.
*   **PIO:** The Programmable I/O (PIO) is not used in this implementation.

For more details, see the official Raspberry Pi Pico C/C++ SDK documentation:
* [Section 4.3 PWM](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf#%5B%7B%22num%22%3A330%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C117%2C825%2C0%5D)
* [Section 4.5 ADC](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf#%5B%7B%22num%22%3A345%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C117%2C825%2C0%5D)
* [Section 2.5 DMA](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf#%5B%7B%22num%22%3A129%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C117%2C825%2C0%5D)

## Getting Started

This simple example demonstrates how to get your motor up and running with a Seeed Studio XIAO RP2040.

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
  hal_motor_init_pwm(MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN);
  hal_motor_init_bemf_adc(MOTOR_BEMF_A_PIN, MOTOR_BEMF_B_PIN);
  hal_motor_init_bemf_dma(on_bemf_update);


  // 4. Set a motor speed and direction
  // The duty_cycle is a value between 0 and 255.
  // The second parameter is the direction (true for forward, false for reverse).
  hal_motor_set_duty(128, true);
}

void loop() {
  // The main loop can be used for other tasks.
  // The BEMF reading and motor control are handled by interrupts.
}
```

You can find this and other examples in the `examples` folder of this repository.

## API Reference

### `void hal_motor_init_pwm(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint32_t pwm_frequency = 20000, uint8_t motor_id = 0)`

Initializes the PWM hardware for a standard 2-pin motor driver.

### `void hal_motor_init_bemf_adc(uint8_t bemf_a_pin, uint8_t bemf_b_pin, uint8_t motor_id = 0)`

Initializes the ADC for BEMF sensing.

### `void hal_motor_init_bemf_dma(hal_bemf_update_callback_t callback, uint8_t motor_id = 0)`

Initializes the DMA for BEMF sensing.

### `void hal_motor_set_duty(int duty_cycle, bool forward, uint8_t motor_id = 0)`

Sets the motor's PWM duty cycle and direction. This function updates the PWM hardware with the new duty cycle. It should be called periodically from the main application loop to reflect the latest output from the motor control algorithm (e.g., a PI controller).

### `int hal_motor_get_bemf_buffer(volatile uint16_t** buffer, int* last_write_pos)`

Retrieves the BEMF ring buffer for diagnostics. This function provides low-level access to the raw ADC sample buffer. It is intended for debugging and visualization, not for real-time control.

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
