# Technical Debts

This document lists the known technical debts in the `xDuinoRails_MotorControl_bEMF` project.

## Critical Bugs
- **BEMF Disabled on RP2040**: The `USE_IRQ_TRIGGER` macro is undefined in the build configuration, causing the HAL to skip interrupt registration and effectively disabling BEMF measurement.

## Testing & Verification
- **Missing Test Infrastructure**: The `test/` directory is missing. Unit tests need to be implemented and integrated into the CI pipeline.
- **Limited Simulation Coverage**: The CI simulation only runs `SineWaveSpeed` on RP2040. It should be expanded to cover other examples and scenarios.

## Documentation
- **Missing Process Documentation**: `TESTING_IMPROVEMENTS.md` is referenced in `AGENTS.md` but does not exist.
- **Outdated Roadmap**: The roadmap previously listed deleted platforms (ESP32, nRF52, AVR) as implemented.

## Code Quality
- **Magic Numbers**: Replace hardcoded values in HAL implementations with named constants or configurable parameters.
