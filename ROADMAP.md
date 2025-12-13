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

- [ ] Allow the init bEMF pins to NULL to disable bEMF readback on a motor
- [ ] Add the LED output as second motor to the SineExample
