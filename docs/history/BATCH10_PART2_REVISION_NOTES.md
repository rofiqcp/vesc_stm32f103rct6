# Batch 10 Part 2 — Adaptive R, Hall Rate Limit, Physical Input-Current Map, FOC Speed Source

Baseline: `vesc_stm32f103rct6_batch10_part1_fast_fw_encoder_slip.zip`
Audit/port date: 2026-08-20

## Scope

Part 2 intentionally stays away from ADC/PWM retiming. TIM1/TIM8 complementary PWM, the 16-kHz DMA1-HT FOC entry, ADC1/ADC2 current ranks, ADC3/DMA2 Vbus acquisition, Batch-9 power-stage safety, Batch-10 Part-1 fast field weakening and encoder-slip protection remain intact.

The excluded ecosystems remain absent: CAN hardware, IMU, BMS/bm_if, NRF, LEDPWM, COM_USB, QML UI, lispif, NTC runtime and LZO.

Implemented:

1. VESC-style online motor-resistance estimate at 1 kHz.
2. Explicit Hall electrical-angle rate limiter in the fixed-point 16-kHz phase path.
3. VESC-style `l_in_current_map_start/filter` pre-limit driven by the hoverboard's physical per-motor DC-current measurement.
4. FOC PLL/FAST/FASTER phase-source selector: corrected/control phase or observer phase.

## 1. Adaptive motor-resistance estimator

The estimator follows current upstream VESC structure with gain `0.00002`, a task-side 1-kHz update and bounds of `0.25 * R_config` to `3.0 * R_config`.

Adaptation is allowed only while PWM is active, the observer is valid, detection is not running and current magnitude is above a noise floor. Reconfiguring/detecting a new motor resistance resets the adaptive state.

Important: `res_est_ohm` is a live estimate/diagnostic value. It does **not** silently overwrite `foc_motor_r` used by the observer. This avoids creating an unvalidated nested adaptation loop on STM32F103. `mcpwm_foc_get_est_res*()` and terminal `observer` expose the live estimate when valid.

## 2. Hall angle rate limiter

After Hall sector interpolation and before Hall-to-observer blending, the target Hall phase is slew-limited with the current-VESC rule:

`max_step = 1.5 * electrical_speed * dt`

In the 16-kHz u16 electrical-phase representation this becomes:

`step = ERPM * 98304 / (60 * 16000)`

The implementation is integer-only in the hard path. It uses the larger of measured Hall edge speed and configured `foc_hall_interp_erpm`, matching the intent of upstream Hall rate limiting. Invalid Hall states reset the limiter rather than retaining a stale phase.

## 3. Physical input-current pre-map

The stock hoverboard has a physical DC-current signal per motor, so Part 2 uses that measured current rather than estimating battery current from `Iq * duty`.

Board-policy defaults while VESC6 wire ABI is pinned:

- `l_in_current_map_start = 0.90`
- `l_in_current_map_filter = 0.005`

The filtered positive DC current begins reducing available positive motor current at 90% of the effective input-current ceiling and reaches zero additional acceleration torque at the ceiling. Negative/braking current remains governed by the independent regen/input-current limit path.

These fields are newer than this project's pinned VESC6 wire representation. VESC6 deserialization supplies the board defaults, while serialization refuses non-default values so runtime tuning cannot be falsely claimed as persistent.

## 4. FOC speed phase source

A runtime `FOC_SPEED_SRC` selector now mirrors current VESC semantics:

- `FOC_SPEED_SRC_CORRECTED` — default, uses the corrected/control phase.
- `FOC_SPEED_SRC_OBSERVER` — uses compensated observer phase when valid and falls back to corrected phase otherwise.

The selected phase drives the fixed-point PLL and FAST/FASTER estimators. A separate `speed_est_fast_corrected_erpm_q16` is retained regardless of selected source for diagnostics and sensor-transition plausibility.

This selector is not part of VESC6 wire ABI. Standard VESC6 decode always chooses CORRECTED and serializer rejects an OBSERVER selection rather than silently losing it.

## Existing offset tracking

No second current-offset estimator was added. The existing `offset_track_isr()` remains the single owner of runtime current-offset drift tracking.

## Verification

Passed together:

- `tools/verify_vesc_port.py`
- Batch 2 through Batch 8 regressions
- Batch 9 Part 1, Part 2 and Part 3 regressions
- Batch 10 Part 1 regression
- `tools/test_batch10_part2.py`
- `tools/debug.py --self-test`

Part-2 strict host syntax checks use GCC `-Wall -Wextra -Wshadow -Wdouble-promotion -Wformat=2 -Werror` for `mc_interface.c`, `mcpwm_foc.c`, `confgenerator.c` and `terminal.c`.

The audit environment does not contain PlatformIO or `arm-none-eabi-gcc`; therefore this is source/host verified, not a claim of target ARM build or physical-board validation.

## Hardware commissioning

1. Build with the actual STM32F103 PlatformIO toolchain.
2. Verify physical left/right DC-current polarity and scale before relying on the 90% input-current map.
3. Log `input_current`, mapped torque limit and Vbus under increasing load; verify smooth reduction near the configured input-current ceiling.
4. Test RIGHT Hall operation at low current first and verify no unexpected phase lag/torque spikes from the new rate limiter.
5. Compare CORRECTED vs OBSERVER speed source only after observer phase has been validated; default CORRECTED is recommended for initial commissioning.
6. Log `Rcfg` and `Rest` during controlled motor heating/load. Do not feed `Rest` back into observer R until the estimate is proven stable on hardware.
