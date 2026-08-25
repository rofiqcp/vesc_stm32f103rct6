# Comparison: motor_id_protection vs motor_detect_protection

## Decision
Use `motor_detect_protection` as the architectural base, with the hard-VIN ordering and internal protection diagnostics retained from the `motor_id_protection` idea set. This directory is that merged result.

## Why motor_detect_protection is the stronger base
- Implements the real VESC6 `l_slow_abs_current` configuration field and round-trip ownership.
- Keeps a physical 25 A board ceiling separate from the configurable absolute-current threshold.
- Uses a selected-axis Ld/Lq capture buffer: only I/V for the active d or q axis are stored in the 16 kHz ISR.
- Reduces inductance-capture storage from four 32xint32 arrays to two 32xint32 arrays per motor and halves capture-store traffic.
- Adds Lq locked-rotor speed-abort protection.
- Uses relative hard VIN margins while retaining four-sample debounce for configured VIN limits.
- Validates saliency relative to average L instead of only using a very loose absolute 20 mH bound.
- Adds a dedicated VESC6 config round-trip regression.

## Useful behavior retained from motor_id_protection
- Internal filtered/peak absolute-current telemetry for post-mortem diagnostics.
- Visibility of VIN debounce counters.
- Hard VIN checks are executed before the eight-sample startup current blanking window.

## Important audit finding
The original `motor_detect_protection` placed hard OV/UV checks after the startup blanking early-return. At 16 kHz and eight blank samples this could defer hard VIN checking for about 0.5 ms after MOE enable. The merged version moves only hard VIN checks ahead of blanking. Phase-current startup behavior remains unchanged: startup uses the dedicated gross DC-current guard while phase-current ADC switching transients settle.

## Inductance identification comparison
Both versions measure separate d- and q-axis inductance and export average L plus `Lq-Ld` with a 0.90 scale. The merged base keeps the motor_detect implementation because it selects the capture axis before arming, uses fewer ISR stores/SRAM, retains Id during q-axis probing, alternates q-pulse polarity, and aborts q probing if fast ERPM indicates the rotor is no longer effectively locked.

## Verification performed
- `verify_vesc_port.py`: PASS
- Batch 2 regression: PASS
- Batch 3 regression: PASS
- Batch 4 regression: PASS
- Batch 5 regression: PASS
- Batch 6 regression: PASS
- Batch 7 regression: PASS
- Batch 8 regression: PASS
- Host GCC `-Wall -Wextra -Werror` inductance estimator test: PASS

Hardware validation is still mandatory before trusting measured R/L/flux/encoder parameters or persisting automatic detection results.
