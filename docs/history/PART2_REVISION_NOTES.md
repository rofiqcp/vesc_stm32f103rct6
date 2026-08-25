# PART 2 Revision Notes — v28 modern observers + hardening

Base: `vesc_stm32f103rct6_PART1_v27_sensorless_autodetect.zip`
Firmware identity: `vesc-f103-hoverboard-v28-part2`
Protocol/config target: VESC 6.00 wire ABI (MCCONF 481 B / APPCONF unchanged)

## Scope completed

### 1. Fixed-point modern non-HFI observer family
The 16-kHz hard FOC path now has executable fixed-point branches for:

- `FOC_OBSERVER_ORTEGA_ORIGINAL`
- `FOC_OBSERVER_MXLEMMING`
- `FOC_OBSERVER_ORTEGA_LAMBDA_COMP`
- `FOC_OBSERVER_MXLEMMING_LAMBDA_COMP`
- `FOC_OBSERVER_MXV`
- `FOC_OBSERVER_MXV_LAMBDA_COMP`
- `FOC_OBSERVER_MXV_LAMBDA_COMP_LIN`

MXLEMMING attribution to David Molony / MESC is retained in-source. Lambda-comp variants use bounded adaptive lambda state. The hard observer function contains no float trig, libm sqrt, pow, or double operations. MXV vector clipping uses a bounded restoring integer square-root only when clipping/magnitude estimation is required. A previous alpha-max approximation was removed because host trajectory testing showed it could over-clamp a valid rotating flux vector and create phase lag.

The upstream algorithm reference used for the port is current VESC `motor/foc_math.c`:
https://github.com/vedderb/bldc/blob/master/motor/foc_math.c

### 2. Saturation compensation parity
`SAT_COMP_DISABLED`, `SAT_COMP_FACTOR`, `SAT_COMP_LAMBDA`, and `SAT_COMP_LAMBDA_AND_FACTOR` are all accepted and executed. Slow-loop inductance/flux compensation publishes fixed coefficients consumed by the hard ISR.

### 3. Observer selection is now real configuration
`foc_observer_type` is no longer an enum-only placeholder. The existing one-byte observer slot is runtime mutable and persistent. Ortega remains the conservative compiled default.

Compatibility detail: native VESC 6.00 defines observer IDs 0..3 only. MXV IDs 4..6 are later VESC extensions. Therefore this firmware continues to advertise VESC 6.00 and does not pretend that an old VESC-6 parameter UI natively understands MXV. For practical tuning from VESC Tool, the terminal now supports:

- `observer` — show current observer diagnostics
- `observer set 0..6` — choose observer type
- `observer sat 0..3` — choose saturation-compensation mode
- `observer speed 0..1` — choose corrected/observer FOC speed source

Writes are rejected while PWM is active or Detect-All is busy and pass through the canonical validated serializer/apply/transactional-flash path.

### 4. VESC6 FOC speed-source correction
A previous port assumption said FOC speed source did not exist in VESC 6.00. That was incorrect. Official VESC 6.00 serializes `foc_speed_soure` at the corresponding MCCONF byte. This port now round-trips byte 314 as:

- `FOC_SPEED_SRC_CORRECTED`
- `FOC_SPEED_SRC_OBSERVER`

The field is validated, applied to runtime, returned by GET MCCONF, and persisted by SET MCCONF/store.

Official VESC 6.00 references:
https://raw.githubusercontent.com/vedderb/bldc/6.00/datatypes.h
https://raw.githubusercontent.com/vedderb/bldc/6.00/confgenerator.c

### 5. Part-1 typed-configuration integration fix
Part 1 made sensorless valid in the VESC wire path, but `mc_interface_set_configuration()` still contained a typed-config path that could effectively force a non-encoder request back toward Hall semantics. Part 2 makes typed and wire paths consistent:

- LEFT: sensorless / Hall / ABI encoder
- RIGHT: sensorless / Hall only
- RIGHT physical encoder remains rejected
- Sensorless leaves physical pins in benign Hall-input configuration; the observer is the phase source.

### 6. Sampling-window instrumentation, without unsafe fake dynamic sampling
Stock hoverboard topology uses shared ADC1/ADC2 current acquisition for both local inverters. This build continues to enforce the hardware-qualified 10–90% modulation/current-sampling window.

Added per-motor diagnostics:

- `sampling_window_clamp_count`
- `sampling_margin_min_q15`

The values show how often the voltage request reaches the safe sampling boundary and the minimum observed margin.

`foc_sample_v0_v7` and `foc_sample_high_current` remain explicitly rejected. They are NOT silently ACKed because implementing them correctly requires timer/ADC trigger retiming and oscilloscope validation for both independently modulated motors. Full overmodulation therefore remains hardware-gated rather than being falsely advertised.

### 7. Watchdog/RTOS diagnosability hardening
The existing conservative IWDG timing remains unchanged so flash-page operations retain margin. Part 2 adds:

- exact unhealthy heartbeat mask
- missed-window counter per heartbeat owner
- FOC / motor-service / communication heartbeat diagnostics
- terminal exposure
- COMM_DIAG exposure

This makes a future watchdog tightening decision measurable rather than arbitrary.

### 8. COMM_DIAG revision 10
Custom communication diagnostics are bumped to revision 10 and append:

- watchdog unhealthy mask
- FOC miss count
- motor-service miss count
- communication miss count
- LEFT sampling clamp count
- RIGHT sampling clamp count
- LEFT minimum sampling margin Q15
- RIGHT minimum sampling margin Q15

`tools/debug.py` decodes revision 10 and its self-test covers the new payload.

### 9. Safety/invariants intentionally preserved
No changes were made to these fundamental contracts:

- LEFT PWM: TIM8
- RIGHT PWM: TIM1
- high-side PWM input: active HIGH
- low-side complementary PWM input: active LOW
- hard FOC: fixed-point, 16 kHz
- LEFT sensor policy: Hall / ABI / sensorless
- RIGHT physical sensor policy: Hall only; sensorless observer allowed
- HFI remains excluded
- physical CAN remains excluded; local Motor-2 forwarding is still protocol-only
- IMU, BMS, bm_if, NRF, LEDPWM, COMUSB, QMLUI, LispIF, NTC temperature, and LZO remain excluded
- transactional flash config and VESC6 wire sizes remain unchanged

## Deliberately NOT enabled without hardware qualification

These are not claimed complete in this ZIP:

1. Dynamic V0/V7 or high-current ADC sampling.
2. Full VESC overmodulation beyond the current safe sampling window.
3. Motor-phase startup audio. The non-blocking PA4 buzzer startup melody remains; motor excitation is not introduced just for sound.
4. VESC bootloader / in-tool firmware flashing. PlatformIO/ST-Link remains the update path.
5. VESC external CAN power-switch emulation. PA5 is this controller's own power-hold latch, not a remote VESC PSW node.
6. Migration of the complete MCCONF/APPCONF schema to VESC 7.00. Wire ABI remains intentionally VESC 6.00.

These omissions are intentional safety/compatibility boundaries, not silent TODO implementations.

## Validation state

All `tools/test_*.py` scripts pass after Part 2: **20/20 PASS**.

In addition to static/source regressions, `tools/test_part2_observer_runtime.py` compiles the actual fixed-point `src/motor/foc_math.c` on the host and executes **21 synthetic PMSM trajectories** (7 observers × forward/reverse/current cases). The final test verifies observer validity, flux/lambda bounds, and phase tracking; all 21 trajectories pass. Representative loaded-current phase errors are about 2.7–3.0° for the conservative Ortega path, ~0.03–0.05° for MXLEMMING, and ~0.005–0.022° for MXV in this synthetic test. These are host-model validation figures, not a substitute for real motor measurements.

Strict host `-Werror` syntax checks are included for portable changed control/config units. `tools/debug.py --self-test` also passes with COMM_DIAG revision 10.

The execution environment used for this revision does not contain PlatformIO or `arm-none-eabi-gcc`, so an STM32F103 target link/flash is NOT claimed. Hardware validation is still required for gate waveforms, ADC timing/current polarity, observer takeover under load, and ISR WCET for each observer type.

## Recommended hardware validation order

1. Build with PlatformIO for the target F103 environment.
2. Confirm linker RAM/Flash headroom.
3. Scope TIM8/TIM1 six gate signals with motor power limited; verify polarity/deadtime/MOE-off state.
4. Validate ADC offsets and current polarity at zero vector / low current.
5. LEFT Hall, LEFT ABI, RIGHT Hall regression.
6. Sensorless startup forward/reverse unloaded.
7. Ortega baseline loaded motor test.
8. MXLEMMING, Ortega-lambda, MXLEMMING-lambda, MXV, MXV-lambda, MXV-lambda-linear one at a time; record ISR max cycles, observer quality, phase error, current ripple, startup success rate.
9. Review `sampling_window_clamp_count` and `sampling_margin_min_q15` before any attempt to extend modulation.
10. Validate fault injection, watchdog owner misses, config power-cut rollback, and final dual-motor Detect-All.
