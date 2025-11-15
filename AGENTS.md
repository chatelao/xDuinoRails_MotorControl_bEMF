# Agent Instructions

When working with the Hardware Abstraction Layer (HAL) in this repository, it is crucial to prioritize low-level, low-CPU-load implementations. Do not replace existing low-level code with simpler, higher-level abstractions like `analogRead()` or `analogWrite()`. Instead, if compilation errors or other issues arise, your primary goal should be to fix the existing implementation while preserving its performance characteristics.
