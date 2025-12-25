# Hardware-in-the-Loop (HIL) Test Concept

This document outlines a concept for a Hardware-in-the-Loop (HIL) test using two XIAO RP2040 boards.

## Overview

The HIL test setup involves two XIAO RP2040 boards:
1.  **Device Under Test (DUT)**: This board runs the `debugPWM.ino` firmware, which generates PWM signals to be tested.
2.  **Logic Analyzer**: This board runs the `pico-logic-analyzer` firmware and captures the PWM signals from the DUT for analysis.

## Hardware Requirements

*   Two XIAO RP2040 boards
*   USB-C cables
*   Jumper wires

## Firmware

### Device Under Test (DUT)

1.  **Firmware**: `examples/debugPWM/debugPWM.ino`
2.  **Flashing**:
    *   Open the project in PlatformIO.
    *   Connect one XIAO RP2040 board to your computer.
    *   Build and upload the `debugPWM` example to the board.

### Logic Analyzer

1.  **Firmware**: `pico-logic-analyzer`
    *   Download the latest `.uf2` file from the [pico-logic-analyzer releases page](https://github.com/pico-coder/sigrok-pico/releases).
2.  **Flashing**:
    *   Connect the second XIAO RP2040 board to your computer while holding the "Boot" button.
    *   The board will mount as a USB mass storage device.
    *   Drag and drop the downloaded `.uf2` file onto the device.

## Wiring

Connect the two XIAO RP2040 boards as follows:

| DUT Pin | Logic Analyzer Pin | Description                  |
| :------ | :----------------- | :--------------------------- |
| `GND`   | `GND`              | Common Ground                |
| `D8`    | `D0`               | PWM Signal from DUT          |
| `D0`    | `D1`               | Synchronization Pulse from DUT |

## Test Execution

1.  **Connect the Logic Analyzer**: Connect the XIAO RP2040 running the `pico-logic-analyzer` firmware to your computer.
2.  **Start Capture**: Open a terminal and run the following `sigrok-cli` command to start capturing data:

    ```bash
    sigrok-cli -d raspberrypi-pico --continuous -c samplerate=1m -P pwm,pulse -C D0,D1 -o capture.sr
    ```

    *   `-d raspberrypi-pico`: Specifies the logic analyzer device.
    *   `--continuous`: Captures data continuously.
    *   `-c samplerate=1m`: Sets the sample rate to 1 MHz.
    *   `-P pwm,pulse`: Assigns protocol decoders for PWM and pulse signals.
    *   `-C D0,D1`: Specifies the channels to capture (D0 for PWM, D1 for pulse).
    *   `-o capture.sr`: Saves the capture to a file named `capture.sr`.

3.  **Reset the DUT**: While the capture is running, press the "Reset" button on the DUT to restart the `debugPWM` firmware.
4.  **Stop Capture**: Stop the `sigrok-cli` capture by pressing `Ctrl+C`.
5.  **Analyze Data**: You can analyze the captured data using PulseView, the GUI for sigrok.

This setup allows for the automated testing and verification of the PWM signal generation in the `debugPWM.ino` firmware.
