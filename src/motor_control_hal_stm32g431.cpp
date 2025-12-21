#include "motor_control_hal.h"

#if defined(ARDUINO_ARCH_STM32)

#include "motor_control_hal_stm32g431_internal.h"

// --- Variables ---
// Array to hold the context for each motor
MotorContext motor_contexts[MAX_MOTORS];


// Include the HAL implementation for the STM32G431
#include "motor_control_hal_stm32g431_motor.cpp.inc"
#include "motor_control_hal_stm32g431_pwm.cpp.inc"
#include "motor_control_hal_stm32g431_adc.cpp.inc"

#endif // ARDUINO_ARCH_STM32
