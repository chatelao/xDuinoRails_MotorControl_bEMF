# Seeed Studio XIAO STM32C0

The Seeed Studio XIAO STM32C0 is an ultra-tiny, cost-optimized microcontroller based on the STM32C0 series. It is designed for applications where size and cost are critical factors, offering a robust ARM Cortex-M0+ core in the signature XIAO form factor.

## Hardware Overview

### Features

*   **Ultra-Tiny MCU:** Based on the STM32C031C6, featuring an ARM Cortex-M0+ core running at up to 48 MHz.
*   **Memory:** 32KB of Flash memory and 12KB of SRAM.
*   **Compact Design:** Inherits the standard XIAO footprint (21x17.8mm), ideal for wearable and space-constrained projects.
*   **Rich Peripherals:** Includes standard serial interfaces (UART, I2C, SPI), multiple PWM-capable pins, and a high-performance ADC.
*   **Low Power:** Optimized for minimal power consumption in various sleep modes.

### Specifications

| Item                                     | Value                                           |
| ---------------------------------------- | ----------------------------------------------- |
| CPU                                      | ARM Cortex-M0+ processor up to 48MHz            |
| Flash Memory                             | 32KB                                            |
| SRAM                                     | 12KB                                            |
| Digital I/O Pins                         | 11                                              |
| Analog I/O Pins                          | 11 (Shared)                                     |
| PWM Pins                                 | 11 (Shared)                                     |
| I2C interface                            | 1                                               |
| SPI interface                            | 1                                               |
| UART interface                           | 1                                               |
| Power supply and downloading interface | Type-C                                          |
| Power                                    | 3.3V/5V DC                                      |
| Dimensions                               | 21×17.8×3.5mm                                   |

## Pinout

The XIAO STM32C0 follows the standard XIAO pin mapping.

### External Pins

| Pin | Arduino Pin | GPIO  | Function        | Description                               |
| --- | ----------- | ----- | --------------- | ----------------------------------------- |
| 1   | D0          | PA0   | ADC, PWM        | Digital I/O, Analog Input                 |
| 2   | D1          | PA1   | ADC, PWM        | Digital I/O, Analog Input                 |
| 3   | D2          | PA2   | ADC, PWM        | Digital I/O, Analog Input                 |
| 4   | D3          | PA3   | ADC, PWM        | Digital I/O, Analog Input                 |
| 5   | D4 / SDA    | PA4   | I2C_SDA, PWM    | I2C Data Pin                              |
| 6   | D5 / SCL    | PA5   | I2C_SCL, PWM    | I2C Clock Pin                             |
| 7   | D6 / TX     | PA6   | UART_TX, PWM    | UART Transmit Pin                         |
| 8   | D7 / RX     | PA7   | UART_RX, PWM    | UART Receive Pin                          |
| 9   | D8 / SCK    | PA8   | SPI_SCK, PWM    | SPI Clock Pin                             |
| 10  | D9 / MISO   | PA9   | SPI_MISO, PWM   | SPI Master In Slave Out Pin               |
| 11  | D10 / MOSI  | PA10  | SPI_MOSI, PWM   | SPI Master Out Slave In Pin               |
| 12  | 3V3         | -     | Power           | 3.3V Power Output                         |
| 13  | GND         | -     | Ground          | Ground                                    |
| 14  | 5V          | -     | Power           | 5V Power Input/Output                     |

### On-board Devices

| Device           | GPIO Pin | Description                                                                |
| ---------------- | -------- | -------------------------------------------------------------------------- |
| User LED         | PA5      | Onboard user LED (Shared with D5).                                         |
| Boot Button      | -        | Used for entering bootloader mode.                                         |
| Reset Button     | -        | Hardware reset button.                                                     |

## Getting Started

### PlatformIO

1.  **Install PlatformIO:** Follow the standard installation for your IDE.
2.  **Configure `platformio.ini`:**
    ```ini
    [env:seeed_xiao_stm32c0]
    platform = ststm32
    board = seeed_xiao_stm32c0
    framework = arduino
    ```
3.  **Note on HAL Support:** While the hardware is documented, full HAL-accelerated BEMF support for the STM32C0 variant is currently under development.

## Resources

*   [Seeed Studio XIAO STM32C0 Wiki](https://wiki.seeedstudio.com/xiao_stm32c0_getting_started/)
*   [STM32C031 Datasheet](https://www.st.com/resource/en/datasheet/stm32c031c6.pdf)
*   [STM32C0 Series Reference Manual](https://www.st.com/resource/en/reference_manual/rm0490-stm32c0-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
