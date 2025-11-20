# Implementation Roadmap

This document outlines the status of supported platforms and the plan for adding support for currently unsupported boards.

## Implemented Platforms

### Seeed XIAO ESP32-S3 / ESP32-S3 Sense

- [x] Implement the Hardware Abstraction Layer (HAL) for the ESP32-S3.
- [x] Add a new PlatformIO environment for the `seeed_xiao_esp32s3` board.
- [x] Verify that all examples compile successfully for the new platform.
- [ ] Perform on-hardware testing and validation.
- [ ] Add documentation for the Seeed XIAO ESP32-S3.

### Seeed XIAO nRF52840 / nRF52840 Sense

- [x] Implement the Hardware Abstraction Layer (HAL) for the nRF52840.
- [x] Add a new PlatformIO environment for the `seeed_xiao_nrf52840` board.
- [x] Verify that all examples compile successfully for the new platform.
- [ ] Perform on-hardware testing and validation.
- [ ] Add documentation for the Seeed XIAO nRF52840.

## Planned Platforms

### Seeed XIAO RA4M1 (Renesas) / Arduino Uno R4

- [ ] Implement the Hardware Abstraction Layer (HAL) for the Renesas RA4M1.
- [ ] Add a new PlatformIO environment for the `seeed_xiao_ra4m1` board.
- [ ] Verify that all examples compile successfully for the new platform.
- [ ] Perform on-hardware testing and validation.
- [ ] Add documentation for the Seeed XIAO RA4M1.

### Arduino AVR (Uno, Nano, Mega)

- [ ] Research feasibility of low-level BEMF on AVR.
- [ ] Implement the Hardware Abstraction Layer (HAL) for AVR.
- [ ] Add a new PlatformIO environment for `uno`, `nano`, etc.
- [ ] Verify that all examples compile successfully for the new platform.
- [ ] Add documentation for Arduino AVR.

### Teensy (4.x / 3.x)

- [ ] Implement the Hardware Abstraction Layer (HAL) for Teensy.
- [ ] Add a new PlatformIO environment for Teensy boards.
- [ ] Verify that all examples compile successfully for the new platform.
- [ ] Add documentation for Teensy.

### Seeed XIAO EFR32MG24 (MG24 Sense)

- [ ] Research PlatformIO support for the EFR32MG24.
- [ ] Implement the Hardware Abstraction Layer (HAL) for the EFR32MG24.
- [ ] Add a new PlatformIO environment for the `seeed_xiao_mg24` board.
- [ ] Verify that all examples compile successfully for the new platform.
- [ ] Perform on-hardware testing and validation.
- [ ] Add documentation for the Seeed XIAO EFR32MG24.

### Seeed XIAO RP2350

- [ ] Research PlatformIO support for the RP2350.
- [ ] Implement the Hardware Abstraction Layer (HAL) for the RP2350.
- [ ] Add a new PlatformIO environment for the `seeed_xiao_rp2350` board.
- [ ] Verify that all examples compile successfully for the new platform.
- [ ] Perform on-hardware testing and validation.
- [ ] Add documentation for the Seeed XIAO RP2350.
