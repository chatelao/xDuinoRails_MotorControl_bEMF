# PWM Control and Frequency Selection

Pulse Width Modulation (PWM) is the core technology for controlling motor power. This document describes the current implementation and the implications of the chosen PWM frequency.

## Current Implementation: Hardware-Accelerated PWM

The project uses the RP2040's hardware PWM slices to implement high-frequency, non-blocking motor control.

- **PWM Frequency:** The default frequency is **20 kHz**.
- **Special Edition:** For the "LED Edition", the frequency is set to **10 Hz** to make the PWM pulses visible on LEDs.

## 20 kHz Frequency

The choice of 20 kHz places the switching frequency above the audible range for most humans, resulting in silent operation.

### Advantages

- **Silent Operation:** No audible whine or hum from the motor.
- **Smoother Torque:** The high frequency results in smoother current flow through the motor windings.

### BEMF Measurement

At 20 kHz, the total cycle time is 50 µs. BEMF measurement must occur during the OFF phase of the PWM cycle.

- **Measurement Window:** The measurement happens immediately after the PWM pulse ends.
- **Hardware Trigger:** The PWM wrap interrupt triggers a hardware timer delay (typically 10 µs) to allow for settling, followed by an ADC start trigger.
- **DMA Transfer:** ADC results are automatically transferred to memory via DMA.

This tight timing is handled entirely by hardware triggers and DMA, requiring minimal CPU intervention.
