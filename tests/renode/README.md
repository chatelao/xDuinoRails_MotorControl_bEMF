# Renode Tests

This directory contains automated tests using [Renode](https://renode.io/) and [Robot Framework](https://robotframework.org/).

## Structure

- `simulation.resc`: The Renode script that defines the hardware environment (RP2040) and loads the firmware.
- `test_firmware.robot`: The Robot Framework test suite that executes the tests.

## Running Tests Locally

To run the tests locally, you need:
1.  **Renode** installed and in your PATH.
2.  **Robot Framework** installed (`pip install robotframework`).
3.  A compiled firmware ELF file (e.g., from `pio run`).

Command:
```bash
renode-test tests/renode/test_firmware.robot
```

**Note:** The tests expect the firmware to output to UART0. The standard firmware might use USB.
In CI, the firmware is recompiled with `-DDEBUG_SERIAL=Serial1` to enable UART logging for the test duration.
