# BEMF Measurement

Accurate measurement of Back-EMF (BEMF) is crucial for precise, sensorless motor control. This document describes the technique implemented in the project to extract stable speed information from a noisy raw signal.

The process is based on a differential measurement.

## 1. Foundation: Differential Measurement

BEMF is measured as the differential voltage between the two motor terminals (`bemfAPin` and `bemfBPin`).

```cpp
// From src/main.cpp
int bemfA = analogRead(bemfAPin);
int bemfB = analogRead(bemfBPin);
int measured_bemf = abs(bemfA - bemfB);
```

**Advantages:**
- **Common-Mode Noise Rejection:** Noise occurring on both lines simultaneously (e.g., induced by PWM switching) is effectively eliminated by differencing.
- **Robust Basis:** Provides a better raw signal than measuring against ground.

## Implementation Summary

The current solution uses a proven technique for robust signal acquisition:

| Stage | Technique | Purpose |
| :--- | :--- | :--- |
| **1** | **Differential Measurement** | Foundation: Capturing the raw signal with common-mode noise rejection. |

This method ensures that a clean raw signal is available for further processing, such as for a PI controller.
