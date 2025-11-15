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

The ESP32 implementation of `xDuinoRails_MotorControl` leverages the advanced peripherals of the microcontroller to achieve high-performance motor control with minimal CPU intervention. For a deep dive into the ESP32's peripherals, refer to the [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf). Here's a high-level overview of how it works:

1. **MCPWM (Motor Control PWM):** The MCPWM peripheral generates the PWM signals for the motor driver, offloading the CPU from this task.

2. **ADC Digital Controller (DMA Mode):** The ADC digital controller, configured in DMA mode, automatically samples the BEMF pins and stores the results in a buffer without any CPU intervention.

3. **GPTimer (General Purpose Timer):** A high-precision GPTimer is used to create a short delay between the PWM pulse and the BEMF measurement, ensuring a clean and accurate reading.

4. **ETM (Event Task Matrix):** The ETM connects the MCPWM, GPTimer, and ADC peripherals, creating a chain of events and tasks that run without any CPU intervention. This allows for a fully automated motor control and BEMF measurement process.

## How it Works (nRF52833)

The nRF52833 implementation relies on its powerful and flexible peripheral system to achieve efficient motor control. For a detailed description of the nRF52833's peripherals, see the [nRF52833 Product Specification](https://infocenter.nordicsemi.com/pdf/nRF52833_PS_v1.0.pdf).

1. **PWM (Pulse Width Modulator):** The PWM peripheral generates the motor control signals. It features EasyDMA, allowing for complex PWM sequences without CPU intervention.

2. **ADC (Analog-to-Digital Converter):** The 12-bit ADC is used to measure the BEMF from the motor windings.

3. **Timer:** A 32-bit timer is used to create a precise delay between the PWM signal and the ADC measurement, ensuring an accurate BEMF reading.

4. **PPI (Programmable Peripheral Interconnect):** The PPI is the key to automating the process. It is used to connect the timer and ADC peripherals, triggering the ADC sampling at the precise moment required after the PWM pulse, all without CPU involvement.

## How it Works (RP2040)

The RP2040 implementation takes advantage of its unique PIO (Programmable I/O) subsystem to create a flexible and efficient motor control solution. The [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf) provides a comprehensive overview of the microcontroller's peripherals.

1. **PWM (Pulse Width Modulator):** The RP2040 has dedicated PWM blocks that are used to generate the motor control signals.

2. **ADC (Analog-to-Digital Converter):** The ADC is used to measure the BEMF from the motor.

3. **PIO (Programmable I/O):** The PIO is a powerful and flexible peripheral that can be programmed to create custom logic. In this library, it's used to create a state machine that precisely times the ADC sampling after a PWM pulse.

4. **DMA (Direct Memory Access):** DMA is used to transfer the ADC results to memory without CPU intervention, minimizing the CPU load.

## How it Works (SAMD21)

The SAMD21 implementation uses a combination of its powerful peripherals to create an efficient and autonomous motor control system. The [SAM D21 Family Datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/SAM-D21-DA1-Family-Data-Sheet-DS40001882G.pdf) provides a complete overview of the device's capabilities.

1. **TCC (Timer/Counter for Control Applications):** The TCC peripheral is a highly configurable timer/counter that's used to generate the PWM signals for the motor driver.

2. **ADC (Analog-to-Digital Converter):** The ADC measures the BEMF from the motor windings.

3. **TC (Timer/Counter):** A separate Timer/Counter is used to create a precise delay between the PWM pulse and the ADC measurement.

4. **Event System:** The SAMD21's Event System is used to connect the TCC, TC, and ADC peripherals. This allows for the automated triggering of the ADC sampling after the PWM pulse, without any CPU intervention.

5. **DMA (Direct Memory Access):** DMA is used to transfer the ADC results to memory, further reducing the CPU load.

## How it Works (STM32)

STM32 microcontrollers are well-suited for motor control applications, and this library takes full advantage of their advanced peripherals. STMicroelectronics provides a wealth of information on motor control with STM32 microcontrollers in their "[Introduction to Motor Control with STM32](https://wiki.st.com/stm32mcu/wiki/STM32MotorControl:Introduction_to_Motor_Control_with_STM32)" guide.

1. **Advanced-Control Timers (TIM1/TIM8):** These timers are specifically designed for motor control applications. They have complementary PWM outputs with dead-time insertion, which is essential for driving H-bridges.

2. **ADC (Analog-to-Digital Converter):** The ADC is used to measure the BEMF from the motor windings. The timers can be configured to trigger the ADC at the precise moment required for an accurate reading.

3. **DMA (Direct Memory Access):** DMA is used to transfer the ADC results to memory without any CPU intervention, which is essential for high-performance applications.

4. **Timer Synchronization:** The STM32's timers can be synchronized with each other, allowing for complex control schemes and precise timing between the PWM signals and the ADC measurements.
