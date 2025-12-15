# Plan: Discrete H-Bridge Support (4-Pin)

## Goal
Extend the motor control library to support discrete H-Bridges, which require 4 separate GPIO pins for the High-Side (HS) and Low-Side (LS) MOSFETs.

## Technical Requirements (RP2040)
*   Use 2 PWM slices per motor (one slice per half-bridge) to utilize hardware features.
*   Use RP2040 Hardware Dead-Time to prevent shoot-through.
*   Synchronized control for BEMF measurement.

## Steps

### 1. API Extension (`motor_control_hal.h`)
- [x] Define new initialization function:
  ```cpp
  void hal_motor_init_discrete(
      uint8_t hs_a_pin, uint8_t ls_a_pin, // Left Half-Bridge (Side A)
      uint8_t hs_b_pin, uint8_t ls_b_pin, // Right Half-Bridge (Side B)
      uint8_t bemf_a_pin, uint8_t bemf_b_pin,
      hal_bemf_update_callback_t callback,
      uint8_t motor_id
  );
  ```

### 2. Data Structures (`motor_control_hal_rp2040.cpp`)
- [x] Update `MotorContext`:
  - Store 4 pins instead of 2.
  - Flag for `is_discrete_mode`.
  - Store 2 Slice IDs (Slice A for Left HB, Slice B for Right HB).

### 3. Implementation Init Logic
- [x] Pin Validation:
  - Verify that HS/LS pairs for each side are on the same PWM slice (Required for hardware dead-time).
    - Example: `hs_a_pin` and `ls_a_pin` must map to the same Slice ID.
- [x] PWM Configuration:
  - Enable Dead-Time via SDK (`pwm_set_dead_time`).
  - Configure Polarity for complementary output (A inverted vs B, or handled via levels).

### 4. Implementation Control Logic (`hal_motor_set_pwm`)
- [x] Logic branch in `hal_motor_set_pwm`:
  - If `is_discrete_mode`:
    - Calculate duty cycles for both Half-Bridges.
    - **Forward**: Left HB = High (HS=An, LS=Aus), Rechte HB = PWM (switching LS/HS).
    - **Reverse**: Left HB = PWM, Right HB = High.
    - *Note:* Specific commutation strategy (Slow Decay vs Fast Decay) determines exact switching pattern.

### 5. Verification
- [x] Verify compilation.
- [ ] Logic verification (e.g., Logic Analyzer trace).
