# Hardware-in-the-Loop (HIL) Test Concept

This document outlines a concept for a Hardware-in-the-Loop (HIL) test using two XIAO RP2040 boards. This guide has been updated with scripts to automate the setup and execution process.

## Overview

The HIL test setup involves two XIAO RP2040 boards:
1.  **Device Under Test (DUT)**: This board runs the `debugPWM.ino` firmware, which generates PWM signals to be tested.
2.  **Logic Analyzer**: This board runs the `pico-logic-analyzer` firmware and captures the PWM signals from the DUT for analysis.

## Hardware Requirements

*   Two XIAO RP2040 boards
*   USB-C cables
*   Jumper wires

## Wiring the Boards

Connect the two XIAO RP2040 boards as follows:

| DUT Pin | Logic Analyzer Pin | Description                  |
| :------ | :----------------- | :--------------------------- |
| `GND`   | `GND`              | Common Ground                |
| `D8`    | `D0`               | PWM Signal from DUT          |
| `D0`    | `D1`               | Synchronization Pulse from DUT |

---

## Automated Test Execution Guide

This process is automated with four scripts. Run them in the following order from your terminal.

### 1. Install Dependencies

This script installs `sigrok-cli` and other necessary tools on a Debian-based Linux system (like Ubuntu).

```bash
./test/hil/install_deps.sh
```

### 2. Setup the Logic Analyzer

This script will guide you through the manual process of downloading and flashing the `pico-logic-analyzer` firmware onto one of your XIAO boards.

```bash
./test/hil/setup_logic_analyzer.sh
```

### 3. Setup the Device Under Test (DUT)

This script will compile and upload the `debugPWM` test firmware onto your second XIAO board using PlatformIO.

```bash
./test/hil/setup_dut.sh
```

### 4. Run the Test and Analyze Results

This script will capture the PWM signals from the DUT for 10 seconds and generate a text-based report (`report.txt`) from the captured data.

```bash
./test/hil/run_analysis.sh
```

After running the final script, you can inspect `report.txt` for a summary of the decoded signals or open `capture.sr` in a GUI tool like [PulseView](https://sigrok.org/wiki/PulseView) for a detailed graphical analysis.
