# Implementation Roadmap

This document outlines the status of supported platforms, planned features, and known technical debts.

## Implemented Platforms

### Seeed XIAO RP2040

- [x] Implement the Hardware Abstraction Layer (HAL) for the RP2040.
- [x] Add a new PlatformIO environment for the `seeed_xiao_rp2040` board.
- [x] Verify that all examples compile successfully for the new platform.
- [x] Perform on-hardware testing and validation.
- [x] Add documentation for the Seeed XIAO RP2040.

## Planned Platforms

### Seeed XIAO RP2350

- [ ] Research PlatformIO support for the RP2350.
- [ ] Implement the Hardware Abstraction Layer (HAL) for the RP2350.
- [ ] Add a new PlatformIO environment for the `seeed_xiao_rp2350` board.
- [ ] Verify that all examples compile successfully for the new platform.
- [ ] Perform on-hardware testing and validation.
- [ ] Add documentation for the Seeed XIAO RP2350.

## Far Future / Multi-platform Support

### Seeed XIAO ESP32-S3 / ESP32-S3 Sense

- [ ] Implement the Hardware Abstraction Layer (HAL) for the ESP32-S3.
- [ ] Add a new PlatformIO environment for the `seeed_xiao_esp32s3` board.
- [ ] Verify that all examples compile successfully for the new platform.
- [ ] Perform on-hardware testing and validation.
- [ ] Add documentation for the Seeed XIAO ESP32-S3.

### Seeed XIAO nRF52840 / nRF52840 Sense

- [ ] Implement the Hardware Abstraction Layer (HAL) for the nRF52840.
- [ ] Add a new PlatformIO environment for the `seeed_xiao_nrf52840` board.
- [ ] Verify that all examples compile successfully for the new platform.
- [ ] Perform on-hardware testing and validation.
- [ ] Add documentation for the Seeed XIAO nRF52840.

### Arduino AVR (Uno, Nano, Mega)

- [ ] Research feasibility of low-level BEMF on AVR.
- [ ] Implement the Hardware Abstraction Layer (HAL) for AVR.
- [ ] Add a new PlatformIO environment for `uno`.
- [ ] Verify that all examples compile successfully for the new platform.
- [ ] Add documentation for Arduino AVR.

### Seeed XIAO RA4M1 (Renesas) / Arduino Uno R4

- [ ] Implement the Hardware Abstraction Layer (HAL) for the Renesas RA4M1.
- [ ] Add a new PlatformIO environment for the `seeed_xiao_ra4m1` board.
- [ ] Verify that all examples compile successfully for the new platform.
- [ ] Perform on-hardware testing and validation.
- [ ] Add documentation for the Seeed XIAO RA4M1.

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

## Technical Debt & Known Issues

### Critical Bugs
- [ ] **BEMF Disabled on RP2040**: The `USE_IRQ_TRIGGER` macro is undefined in the build configuration, causing the HAL to skip interrupt registration and effectively disabling BEMF measurement.

### Testing & Verification
- [ ] **Missing Test Infrastructure**: The `test/` directory is missing. Unit tests need to be implemented and integrated into the CI pipeline.
- [ ] **Limited Simulation Coverage**: The CI simulation only runs `SineWaveSpeed` on RP2040. It should be expanded to cover other examples and scenarios.

### Documentation
- [ ] **Missing Process Documentation**: `TESTING_IMPROVEMENTS.md` is referenced in `AGENTS.md` but does not exist.
- [ ] **Outdated Roadmap**: The roadmap previously listed deleted platforms (ESP32, nRF52, AVR) as implemented. (Corrected in this update).

### Code Quality
- [ ] **Magic Numbers**: Replace hardcoded values in HAL implementations with named constants or configurable parameters.

## Planned Features

- [x] Allow the init bEMF pins to NULL to disable bEMF readback on a motor
- [x] Add the LED output as second motor to the SineExample
