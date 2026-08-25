# Batch 8 — Motor Identification and Protection

Baseline: `vesc_stm32f103rct6_batch7_controller_semantics.zip`

Batch 8 is deliberately limited to motor-parameter identification and protection behavior. It does **not** change the proven TIM1/TIM8 PWM topology, ADC1/ADC2 dual-DMA sampling cadence, USART3 DMA transport, IWDG, VESC packet routing, B6 Tool ecosystem, or the pinned VESC6 481/493-byte config ABI.

## Implemented

### Separate Ld/Lq identification
- Replaced the previous d-axis-only inductance measurement that always returned `Ld-Lq = 0`.
- Detection remains a blocking calibration transaction; operational HFI is **not** enabled.
- Rotor is electrically locked with a bounded d-axis current.
- D-axis current/voltage steps identify `Ld` using the existing 16-kHz PWM/ADC capture.
- Short alternating `+Iq/-Iq` steps identify `Lq` while cancelling average torque.
- Q-axis probing aborts if fast speed exceeds the locked-rotor safety threshold.
- The hard ISR only captures selected-axis Q15 current and causally preceding Q15 applied voltage. All R/L fitting remains task-side floating point.
- Axis estimates retain the conservative `0.90` scaling used by VESC inductance identification.
- Runtime result convention remains compatible with the existing MTPA/observer code:
  - `foc_motor_l = (Ld + Lq) / 2`
  - `foc_motor_ld_lq_diff = Lq - Ld`
- `detect_apply_all` already consumes both values, so MTPA and saliency compensation now receive a measured saliency parameter rather than a forced zero.

### VESC6 `l_slow_abs_current`
- Activated the real VESC6 byte at offset 62 without changing MCCONF size.
- Added typed config round-trip and runtime ownership for the field.
- The physical board absolute-current ceiling remains an immediate first-sample trip.
- With `l_slow_abs_current=true`, only the configurable lower threshold uses a Q15 low-pass filter plus consecutive-sample debounce.
- The slow mode can therefore reject switching spikes without ever masking the board-level emergency limit.

### Voltage protection split into configured and absolute layers
- Configured `l_min_vin/l_max_vin` violations require four consecutive 16-kHz samples.
- A wider absolute hardware envelope remains immediate:
  - over-voltage hard threshold = `min(l_max_vin + 3 V, 60 V)`
  - under-voltage hard threshold = `max(l_min_vin - 2 V, 4 V)`
- Hard thresholds are precomputed task-side in Q15; the hard FOC path adds no floating-point work.
- Debounce counters reset as soon as voltage returns inside the configured range or PWM is disabled.

## Intentionally not included
- No HFI runtime/sensorless mode.
- No PWM frequency or ADC trigger changes.
- No advanced MXLEMMING/lambda/MXV observer.
- No adaptive motor-resistance estimator yet.
- No VESC7 wire migration.
- No changes to physical CAN/IMU/BMS/NRF/USB/Lisp/LZO/QML/NTC exclusions.

## Verification
- `python tools/verify_vesc_port.py`
- `python tools/test_batch8.py`
- `tools/test_batch8_inductance.c` compiled with host GCC `-Wall -Wextra -Werror`; synthetic 240 uH and 360 uH axes are recovered within the regression tolerance.
- `tools/test_batch8_roundtrip.c` verifies VESC6 slow-absolute-current, VIN and L/Ldq typed round-trip while preserving the 481-byte MCCONF.
- Clang `-Wall -Wextra -Werror -fsyntax-only` passes for the changed core translation units with the focused STM32F1/CMSIS host stubs:
  - `src/motor/mcpwm_foc.c`
  - `src/motor/mc_interface.c`
  - `src/confgenerator.c`
- Functional regression assertions from B2, B3, B4, B5, B6 and B7 were re-run. Historical checksum-only guards that intentionally assert an earlier batch left `mcpwm_foc.c` unchanged are not applicable once B8 intentionally changes that file.

Hardware validation is still required. In particular, first real-board Lq detection should be run with a current-limited supply and the wheel/steering mechanism free of unsafe constraints, because the q-axis calibration intentionally produces short alternating torque pulses.
