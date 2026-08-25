# Batch 8 Best Merge

Base: `motor_detect_protection`.

Retained because it is the stronger implementation:
- real VESC6 `l_slow_abs_current` wire/runtime semantics;
- selected-axis Ld/Lq capture with lower ISR write load and lower SRAM use;
- locked-rotor Lq speed-abort protection;
- relative hard VIN margins with configured VIN debounce;
- 0.90 VESC-style inductance under-estimation and `Lq-Ld` saliency output.

Merged from `motor_id_protection` / manual audit:
- hard VIN OV/UV checks execute before the 8-cycle startup current blanking window;
- internal filtered/peak absolute-current diagnostics are retained;
- VIN debounce counters are exposed in internal telemetry;
- Batch-8 regression now checks the hard-VIN-before-blanking ordering.

Startup phase-current behavior is intentionally unchanged from the proven prior design: during the first 8 PWM-synchronous enable samples, the gross DC-current guard is used while phase ADC switching transients settle. After that window the physical 25 A phase-current ceiling is first-sample, and `l_slow_abs_current` only affects the configurable lower threshold.

Hardware validation is still required before automatic persistence of detected motor parameters.
