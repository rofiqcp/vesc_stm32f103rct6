# Batch 1 — Voltage/Modulation + Coherent Telemetry

Base: `vesc_stm32f103rct6_vesc6_foc_applied.zip`

This batch is intentionally limited to three high-priority fixes so each stage
can be reviewed and tested independently.

## Implemented

1. **VESC-style voltage circle uses MCCONF duty/overmodulation**
   - `vmax_coeff_q15 = min(l_max_duty * foc_overmod_factor, hardware duty window) / sqrt(3)`
   - coefficient is precomputed outside the 16-kHz hard ISR.
   - current PI saturation therefore no longer requests a voltage vector larger
     than the hoverboard PWM/sampling window can apply.

2. **SVPWM current-sampling window preserves the alpha/beta vector**
   - removed independent U/V/W 10..90% clipping from `hw.c`.
   - `foc_svm_q15()` now uniformly scales the centered phase excursions when
     needed, preserving vector angle while keeping all phases in the required
     10..90% sampling window.
   - `hw.c` retains only absolute 0..Q15 timer-domain guards.

3. **Atomic/coherent FOC telemetry snapshot**
   - one fixed-point snapshot is published at the end of each motor FOC pass.
   - seqlock publication/read avoids disabling the ADC/DMA FOC interrupt.
   - coherent snapshot includes Ia/Ib/Ic, Id/Iq, filtered Id/Iq, targets,
     Vd/Vq, Vbus, measured DC-link current, fast ERPM, phase duties, control
     phase, observer phase, encoder phase when available, Hall phase, ADC frame,
     and DWT cycle timestamp.
   - `telemetry.c` converts one coherent frame to public float telemetry.

## Files changed in Batch 1

- `src/datatypes.h`
- `src/hwconf/hw.c`
- `src/motor/foc_math.c`
- `src/motor/foc_math.h`
- `src/motor/mcpwm_foc.c`
- `src/telemetry.c`

No UART-DMA, watchdog, dead-time compensation, brake-transition, encoder/
observer source-switch, terminal/plot, or advanced PID changes are included in
this batch.

## Verification performed

- static VESC port verifier: PASS
- VESC6 MCCONF/APPCONF wire sizes/signatures: PASS (481 / 493)
- hard ADC/FOC no-float invariant: PASS
- strict host compile: `foc_math.c`, `mcpwm_foc.c`, `telemetry.c`: PASS
- SVPWM duty-window unit sweep: PASS
- dynamic voltage-circle coefficient test: PASS
- packet short/long/CRC recovery regression: PASS
- battery/regen/input-current limiter regression: PASS
- debug sampling NOW/START/fault/single regression: PASS

`hw.c` cannot be compiled as a complete host object with the minimal HAL stub
because that stub intentionally lacks the full STM32F1 GPIO/TIM/ADC HAL. Its
Batch-1 modification is limited to the PWM setter clamp ownership described
above. Target ARM/PlatformIO and physical-board testing are still required.

## Next recommended batch

**Batch 2 — Applied-voltage model and braking safety**

Recommended scope when the user says `lanjutkan`:

1. fixed-point dead-time applied-voltage compensation for the observer;
2. VESC-style brake zero-cross / direction-change guard;
3. VESC duty-mode behavior (normal ramp via current + dynamic modulation,
   dedicated PI only for duty down-ramp);
4. add targeted host regression tests for these three behaviors.

Keep USART DMA, watchdog, encoder/observer hybrid correction and protocol/API
cleanup for later batches so changes remain reviewable.
