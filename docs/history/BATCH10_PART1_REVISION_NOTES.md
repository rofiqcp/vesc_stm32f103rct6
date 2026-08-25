# Batch 10 Part 1/2 — Fast Field Weakening, VESC7 MTPA/FW Composition, Encoder Slip

Baseline: `vesc_stm32f103rct6_batch9_part3_powerstage_safety.zip`
Audit/port date: 2026-08-20

## Scope

This part intentionally changes only high-value motor-control/sensor-safety behavior. It does not change TIM1/TIM8 PWM timing, ADC1/ADC2 current acquisition, ADC3/DMA2 Vbus acquisition, PVD/BKIN, USART3 DMA, CMSIS-RTOS2 task topology, persistence layout, or the pinned VESC 6.00 481/493-byte wire ABI.

The explicitly excluded ecosystems remain excluded and are not introduced: CAN hardware, IMU, BMS/bm_if, NRF, LEDPWM, COM_USB, QML UI, lispif, NTC runtime, and LZO.

## 1. VESC7-style MTPA + field-weakening composition

Current upstream VESC computes MTPA in the fast current-control path, reduces Iq to keep the requested current-vector magnitude bounded, then combines MTPA Id and field-weakening Id using the request with the larger absolute magnitude rather than adding both negative Id values.

This port now follows that control intent:

- task-side MTPA computes `Id_mtpa`;
- requested Iq is reduced with `sqrt(Iq_cmd^2 - Id_mtpa^2)` so MTPA rotates rather than enlarges the requested current vector;
- the 16-kHz path chooses max-absolute between MTPA Id and `-I_fw`;
- q-axis FW compensation is applied using modulation/Vq sign semantics;
- the Id/Iq current circle is re-applied in fixed-point before the PI error is formed.

The former `Id_mtpa - I_fw` additive behavior is removed.

## 2. Fast fixed-point field weakening

Automatic FW ramp/target generation is moved from the 1-kHz motor-service layer into the 16-kHz hard FOC path, matching the stability-oriented VESC7 architecture while remaining appropriate for Cortex-M3:

- all configuration-dependent float divisions/scaling are precomputed task-side;
- the hard path contains integer/Q-format math only;
- a 0.01 fixed-point low-pass is applied to absolute duty before FW thresholding, matching upstream filtering intent;
- FW backoff uses filtered Iq versus the active effective Iq target and fast electrical-speed direction;
- the ramp uses an atomic signed-32-bit fractional Q31 accumulator, avoiding 64-bit shared state and preserving ramp resolution below one current-Q15 LSB per 16-kHz frame;
- manual FW override follows upstream direct-set behavior instead of passing through the automatic ramp;
- FW is disabled when `foc_fw_current_max < max(cc_min_current, 0.001 A)`;
- current-off hold remains task-owned; the ISR only raises a one-way hold request.

### Hoverboard-specific duty normalization

The board retains the proven 10..90% current-sampling modulation window. The physical duty magnitude seen by the ISR therefore tops out near 0.80 even when VESC-facing `l_max_duty` is higher. FW duty thresholds are normalized from the physical 0.80 usable span into VESC duty semantics so, for example, `foc_fw_duty_start = 0.90` still means 90% of available board modulation instead of becoming unreachable.

This does not relax the sampling window; dynamic sampling remains deferred to Part 2.

## 3. LEFT encoder-slip protection

Additional-fault bit 0 is implemented as `ENCODER_SLIP`, without moving the existing RPM-fault bits.

The check is deliberately limited to the LEFT encoder/ABI motor and requires:

- encoder mode active and synchronized;
- observer valid;
- PWM and command active;
- absolute electrical speed above 110% of `foc_openloop_rpm` so the observer is stable;
- coherent encoder/observer/ERPM snapshot.

The local snapshot stores raw observer phase, so the comparison first applies the same PWM/sample-delay observer compensation used by the high-speed control path. A wrapped electrical phase error above about 15 degrees for 500 consecutive 1-kHz checks raises the dedicated local fault, which maps to canonical `FAULT_CODE_ENCODER_SLIP`.

As with the other newer additional-fault bits, the pinned VESC6 serializer does not fake-persist a non-zero `l_additional_faults` value.

## 4. Files changed

- `src/datatypes.h`
- `src/motor/foc_math.c`
- `src/motor/mc_interface.c`
- `src/motor/mcconf_default.h`
- `src/motor/mcpwm_foc.c`
- `tools/test_batch10_part1.py` (new)
- `tools/test_batch10_part1_control.c` (new)

No ADC/PWM hardware source, RTOS task source, UART source, or excluded subsystem is changed.

## 5. Verification

Passed together in the audit environment:

- `tools/verify_vesc_port.py`
- Batch 2 through Batch 9 Part 3 regression suites
- `tools/test_batch10_part1.py`
- strict host syntax checks of `mc_interface.c`, `mcpwm_foc.c`, and `foc_math.c` with `-Wall -Wextra -Wshadow -Wdouble-promotion -Wformat=2 -Werror`
- numeric host tests for max-absolute composition, current-circle headroom, duty normalization, fractional fast ramp, and wrapped phase error
- `tools/debug.py --self-test`

The hard FW helper is explicitly checked for absence of floating-point/math-library operations.

`pio` and `arm-none-eabi-gcc` are not installed in this audit environment, so this is source/host verified, not an ARM target-build or physical-board validation claim.

## 6. Hardware commissioning order

1. Build with the real PlatformIO STM32F103 toolchain.
2. Leave field weakening disabled initially and verify normal low-current Hall/encoder FOC behavior is unchanged.
3. Enable encoder-slip bit only after observer-vs-encoder phase has been logged above the open-loop threshold; confirm normal phase error remains well below 15 degrees.
4. Test slip fault at low energy with an intentionally altered encoder phase/offset, not by mechanically damaging or disconnecting the running system.
5. Enable FW with a conservative current ceiling and verify ISR cycle/overrun telemetry remains healthy.
6. Log effective Id/Iq, FW current, duty, Vbus, and phase current during the FW threshold transition.
7. Increase load/speed only after the current-circle and Vbus/regen protections remain stable.

Part 2 should address the next non-hardware-invasive P1 items and leave dynamic ADC-window retiming as a separately validated step if it is included.
