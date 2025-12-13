# Technical Debts

This document lists the known technical debts in the `xDuinoRails_MotorControl_bEMF` project.

## Testing & Verification
- **Missing Test Infrastructure**: Unit tests need to be implemented and integrated into the CI pipeline.
- **Limited Simulation Coverage**: The CI simulation only runs `SineWaveSpeed` on RP2040. It should be expanded to cover other examples and scenarios.

## Documentation
- **Outdated Roadmap**: The roadmap previously listed deleted platforms (ESP32, nRF52, AVR) as implemented.

## Code Quality
- **Magic Numbers**: Replace hardcoded values in HAL implementations with named constants or configurable parameters.
