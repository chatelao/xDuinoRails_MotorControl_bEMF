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

---

## Step-by-Step Implementation Guide

### 1. Prepare the Logic Analyzer

1.  **Download Firmware**:
    *   Download the latest `.uf2` file from the [pico-logic-analyzer releases page](https://github.com/pico-coder/sigrok-pico/releases).
2.  **Flash the Board**:
    *   Connect a XIAO RP2040 board to your computer while holding the "Boot" button.
    *   The board will mount as a USB mass storage device.
    *   Drag and drop the downloaded `.uf2` file onto the device.

### 2. Prepare the Device Under Test (DUT)

1.  **Firmware**: `examples/debugPWM/debugPWM.ino`
2.  **Flash the Board**:
    *   Open the project in PlatformIO.
    *   Connect the second XIAO RP2040 board to your computer.
    *   Build and upload the `debugPWM` example to this board.

### 3. Wire the Boards

Connect the two XIAO RP2040 boards as follows:

| DUT Pin | Logic Analyzer Pin | Description                  |
| :------ | :----------------- | :--------------------------- |
| `GND`   | `GND`              | Common Ground                |
| `D8`    | `D0`               | PWM Signal from DUT          |
| `D0`    | `D1`               | Synchronization Pulse from DUT |

### 4. Execute the Test

1.  **Connect Logic Analyzer**:
    *   Connect the XIAO RP2040 running the `pico-logic-analyzer` firmware to your computer.
2.  **Start Capture**:
    *   Open a terminal and run the following `sigrok-cli` command:

      ```bash
      sigrok-cli -d raspberrypi-pico --continuous -c samplerate=1m -P pwm,pulse -C D0,D1 -o capture.sr
      ```
    *   **Command Breakdown**:
        *   `-d raspberrypi-pico`: Specifies the logic analyzer device.
        *   `--continuous`: Captures data continuously.
        *   `-c samplerate=1m`: Sets the sample rate to 1 MHz.
        *   `-P pwm,pulse`: Assigns protocol decoders for PWM and pulse signals.
        *   `-C D0,D1`: Specifies the channels to capture (D0 for PWM, D1 for pulse).
        *   `-o capture.sr`: Saves the capture to a file named `capture.sr`.

3.  **Trigger the DUT**:
    *   While the capture is running, press the "Reset" button on the DUT. This will restart the `debugPWM` firmware and generate the test signals.
4.  **Stop Capture**:
    *   Stop the `sigrok-cli` capture by pressing `Ctrl+C` in the terminal.
5.  **Analyze Data**:
    *   You can now analyze the captured data in `capture.sr` using [PulseView](https://sigrok.org/wiki/PulseView), the GUI for sigrok.

---

This setup allows for the automated testing and verification of the PWM signal generation in the `debugPWM.ino` firmware.
