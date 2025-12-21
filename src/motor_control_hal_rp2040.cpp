/**
 * @file motor_control_hal_rp2040.cpp
 * @brief RP2040-specific implementation for the motor control HAL.
 * @details This file has been refactored to improve modularity. The implementation
 * is now split across five files:
 * - `motor_control_hal_rp2040_motor.cpp.inc`: Core data structures and public API.
 * - `motor_control_hal_rp2040_pwm.cpp.inc`: PWM initialization and control.
 * - `motor_control_hal_rp2040_adc.cpp.inc`: ADC for BEMF.
 * - `motor_control_hal_rp2040_dma.cpp.inc`: DMA for BEMF.
 * - `motor_control_hal_rp2040_irq.cpp.inc`: Interrupt handling.
 * This file serves as the entry point, including the separated implementation files.
 */

#include "motor_control_hal.h"

#if defined(ARDUINO_ARCH_RP2040)

#include "motor_control_hal_rp2040_internal.h"
#include "motor_control_hal_rp2040_adc.cpp.inc"
#include "motor_control_hal_rp2040_dma.cpp.inc"
#include "motor_control_hal_rp2040_irq.cpp.inc"
#include "motor_control_hal_rp2040_pwm.cpp.inc"
#include "motor_control_hal_rp2040_motor.cpp.inc"

#endif // ARDUINO_ARCH_RP2040
