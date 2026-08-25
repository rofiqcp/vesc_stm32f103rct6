# Batch 3 — Sensor / Observer / Fixed-Point PLL

Baseline: `vesc_stm32f103rct6_batch2_deadtime_brake_duty.zip`

## Scope

Batch 3 intentionally changes only the sensor/observer path and the VESC6 configuration fields required by that path. It does **not** change TIM1/TIM8 PWM timing, dual ADC/DMA current sampling, UART transport, watchdog, packet framing, RTOS topology, or the VESC6 wire ABI.

## Implemented

### 1. Fixed-point phase PLL

- Added a hard-path PLL using:
  - phase state in unsigned 16-bit electrical phase,
  - speed in ERPM Q16.16,
  - precomputed `Kp * dt` and `Ki * dt * 60` coefficients.
- `foc_pll_kp` and `foc_pll_ki` are now real runtime parameters rather than configuration-only values.
- Default PLL coefficients at the 16 kHz control rate are regression-tested.
- PLL speed is bounded using the fast phase-speed estimate to prevent runaway windup.

### 2. Encoder AB / observer source selection

LEFT motor only:

- Low-speed control source remains Encoder AB after ABI synchronization.
- High-speed control source changes to the observer with 5% hysteresis around `foc_sl_erpm`.
- The decision uses the fast corrected phase-speed estimate instead of the slow task telemetry speed.
- While observer source is active, the ABI counter is rebased task-side so the return to encoder control does not jump back to a stale electrical origin.
- Mechanical position continuity is retained through the existing mechanical-zero compensation.

RIGHT motor remains Hall-only.

### 3. Fixed-point Ortega observer correction

- Added fixed-point nonlinear flux-magnitude correction for `FOC_OBSERVER_ORTEGA_ORIGINAL`.
- Positive lambda error is clamped according to the Ortega-original behavior used by VESC.
- Added a low-flux anti-collapse safeguard to prevent the observer state from collapsing near zero magnitude.
- No floating-point operation is introduced into the hard ADC/FOC observer or PLL path.

### 4. Observer Ld/Lq saliency adaptation

At the 1 kHz service layer, the effective inductance is computed from:

- `Ld`,
- `Lq`,
- current-vector direction (`Id/Iq`).

Only the resulting fixed-point coefficient is published to the 16 kHz ISR.

### 5. Saturation compensation

`SAT_COMP_FACTOR` now has a real backend:

- saturation factor scales with current magnitude,
- effective inductance and flux linkage are adjusted task-side,
- compensation is bounded conservatively for the STM32F103 hoverboard implementation.

Unsupported lambda-compensation observer modes are rejected rather than silently accepted.

### 6. Dynamic observer gain

Observer correction gain now follows:

- configured observer gain,
- absolute duty,
- DC bus voltage,
- configured slow-gain floor.

The slow layer computes the coefficient and publishes a Q-format value to the ISR.

### 7. VESC6 configuration integration

The following VESC6 MCCONF fields now drive real runtime behavior:

- `foc_pll_kp`
- `foc_pll_ki`
- `foc_observer_gain`
- `foc_observer_gain_slow`
- saturation compensation mode/factor supported by this port

VESC6 wire ABI remains unchanged:

- MCCONF: 481 bytes
- APPCONF: 493 bytes
- existing VESC6 signatures retained

The old Batch-2 placeholder observer pair (`1000 / 1000`) is recognized when importing stored configuration and migrated to the new observer defaults without discarding the rest of the MCCONF image.

## Observer modes intentionally NOT implemented in Batch 3

The hardware validator continues to reject modes that do not have a validated backend in this port, including the additional MXLEMMING/MXV/lambda-compensation observer variants. HFI and other unsupported rotor-sensing ecosystems are not added.

This is intentional: a VESC Tool option must not be accepted unless the STM32F103 backend actually implements it.

## Files changed from Batch 2

- `src/applications/appconf_default.h`
- `src/confgenerator.c`
- `src/datatypes.h`
- `src/motor/foc_math.c`
- `src/motor/foc_math.h`
- `src/motor/mc_interface.c`
- `src/motor/mcconf_default.h`
- `src/motor/mcpwm_foc.c`
- `tools/verify_vesc_port.py`
- `tools/test_batch3.py` (new)
- `BATCH3_REVISION_NOTES.md` (new)

## Regression checks

Batch 3 was checked together with the Batch 2 regressions:

- VESC6 MCCONF/APPCONF layout and signatures
- fixed-point/no-float hard ADC/FOC path
- LEFT Hall/ABI and RIGHT Hall sensor policy
- TIM1/TIM8 + dual-ADC/DMA hardware invariants
- Batch 2 dead-time, duty-mode and brake-transition regressions
- fixed-point PLL convergence
- Encoder AB source hysteresis
- observer-to-encoder rebase behavior
- Ortega fixed-point correction
- low-flux observer safeguard
- Ld/Lq saliency computation
- saturation-factor backend
- dynamic observer-gain behavior
- VESC6 configuration mutability/validation
- strict host compile of the modified control/config translation units

## Hardware validation still required

These tests are source/host regressions. They do not replace physical commissioning. Before high-current operation, verify on the STM32F103RCT6 hoverboard controller:

1. ADC offsets and phase-current polarity.
2. Hall sequence on both motors.
3. LEFT ABI direction/counts and electrical offset.
4. Observer phase vs encoder/Hall phase at low current.
5. Encoder-to-observer source transition around `foc_sl_erpm`.
6. PLL ERPM against a trusted external speed reference.
7. ISR worst-case execution time after the added fixed-point observer/PLL work.

