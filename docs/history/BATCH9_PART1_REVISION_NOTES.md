# Batch 9 Part 1/3 — Thermal, RPM Faults, and Startup-Current Safety

Baseline: `vesc_stm32f103rct6_batch8_best_merged.zip`
Audit/port date: 2026-08-20

## Scope

Part 1 deliberately ports safety/control semantics that do not require retiming the proven motor-control hardware layer. TIM1/TIM8 complementary PWM, ADC1/ADC2 dual-regular simultaneous current sampling, DMA1 Channel 1 half-transfer FOC trigger, the 16-kHz fixed-point hard FOC ISR, USART3 DMA transport, IWDG, observer/PLL, and the pinned VESC 6.00 wire ABI remain unchanged.

This part implements three groups:

1. STM32 internal-temperature-backed board thermal telemetry, derating, and fault handling.
2. VESC-style optional additional RPM faults (over/under/absolute overspeed), with safe-default-disabled runtime policy on the pinned VESC6 ABI.
3. VESC-style `foc_start_curr_dec` / `foc_start_curr_dec_rpm` startup-current reduction using the existing VESC6 wire fields.

## 1. MCU / board temperature backend

- ADC1 regular rank 5 now samples `ADC_CHANNEL_TEMPSENSOR` with the maximum STM32F1 sampling time.
- The six current conversions in ranks 1-3 remain byte-for-byte unchanged.
- DMA half-transfer still occurs immediately after rank 3 and still invokes the hard 16-kHz FOC path before the slow diagnostic ranks.
- DCLINK remains on ADC1 rank 4 in Part 1. The planned ADC3/DMA2 DCLINK separation is intentionally deferred to Part 2/3.
- ADC1 internal temperature uses the STM32F1 TSVREFE path and a task-side conversion from ADC code to degrees Celsius.
- The conversion uses the STM32F1 typical V25 / average-slope relationship with nominal VDDA and therefore must be calibrated/validated on the actual hoverboard PCB before it is treated as an accurate absolute temperature.
- Runtime filtering occurs in the 1-kHz motor-service layer, never in the hard FOC ISR.

Important semantic limitation: this is a **MCU/board-temperature proxy**, not a MOSFET junction-temperature sensor. The VESC-facing FET-temperature telemetry is populated with this documented board proxy so thermal protection exists on stock hardware, but it must not be interpreted as measured MOSFET junction temperature.

## 2. VESC-style thermal current derating

The existing VESC6 fields are now runtime-owned rather than preserved-only:

- `l_temp_fet_start`
- `l_temp_fet_end`
- `l_temp_motor_start`
- `l_temp_motor_end`
- `l_temp_accel_dec`

Behavior:

- Below the start threshold, the normal current limit is available.
- Between start and end, allowed current is linearly reduced toward zero.
- At/end above the configured end threshold, a temperature fault is raised through the existing fault-stop pending-mask path.
- `l_temp_accel_dec` implements an acceleration-only earlier thermal cap while preserving braking authority, following the current VESC control intent.
- Motor-temperature derating becomes active only when a real/overridden motor-temperature value is available. The MCU/board temperature is not reused as a fake winding-temperature value.

## 3. Additional RPM faults

Runtime support was added for the current VESC semantic bits:

- bit 1: overspeed
- bit 2: underspeed
- bit 3: absolute overspeed

The checks use a dedicated slow filtered ERPM signal rather than the fast limiter signal. New local faults map to the matching standard VESC fault codes.

`l_additional_faults` is newer than the pinned VESC6 481-byte MCCONF image used by this project. Therefore:

- runtime default is `0` (disabled), matching the safe upstream default behavior;
- the VESC6 serializer refuses to silently persist a non-zero value;
- no MCCONF offsets, lengths, signatures, or legacy records are shifted.

This avoids pretending that a newer field exists in an older wire schema.

## 4. VESC-style startup-current reduction

The existing VESC6 fields are now active runtime controls:

- `foc_start_curr_dec`
- `foc_start_curr_dec_rpm`

At low absolute ERPM, acceleration current is capped from `foc_start_curr_dec * Imax` at zero speed and ramps linearly to full current at `foc_start_curr_dec_rpm`. A factor of `1.0` therefore leaves the old behavior unchanged. The calculation is task-side and is combined conservatively with the existing motor-current, ERPM, duty, battery/input-current, watt, MTPA, and field-weakening limits.

## 5. VESC6 ABI preservation

The project remains pinned to:

- MCCONF: 481 bytes
- APPCONF: 493 bytes

The existing VESC6 byte positions for thermal and startup-current fields are activated without changing the wire image. Newer `l_additional_faults` remains runtime-only and cannot be falsely persisted into VESC6.

## 6. Hard-path / timing preservation

Regression asserts:

- `src/motor/mcpwm_foc.c` is byte-identical to Batch 8 Best Merged.
- `platformio.ini` is unchanged.
- current-sense ADC ranks 1-3 are unchanged.
- ADC/DMA FOC trigger remains DMA half-transfer after rank 3.
- the long internal-temperature conversion is placed after the FOC half-transfer point.
- calculated full ADC1 six-rank scan at the existing 64-MHz / ADC-prescaler configuration is approximately 36.375 us, below the 62.5-us 16-kHz period.

## 7. Verification

Passed in the audit environment:

- `python tools/verify_vesc_port.py`
- Batch 2 regression
- Batch 3 regression
- Batch 4 regression
- Batch 5 regression
- Batch 6 regression
- Batch 7 regression
- Batch 8 regression
- `python tools/test_batch9_part1.py`
- Batch-8 host inductance estimator (`-Wall -Wextra -Werror`)
- Batch-9 Part-1 thermal/start-current numeric helper test (`-Wall -Wextra -Werror`)
- Batch-9 Part-1 VESC6 config codec round-trip (`-Wall -Wextra -Werror`)
- strict host syntax/warning check of modified `mc_interface.c`

The PlatformIO/ARM toolchain is not installed in this audit container, so a real `pio run` / `arm-none-eabi-gcc` target build was not claimed here.

## 8. Hardware commissioning requirements

Before high-current operation:

1. Build with the actual PlatformIO ARM toolchain.
2. Verify current calibration and all Batch-8 protection behavior unchanged.
3. Log raw MCU-temperature ADC and reported board temperature at room temperature and during controlled warming.
4. Calibrate/validate the board-temperature conversion for the actual PCB/VDDA.
5. Test thermal derating using deliberately low temporary thresholds before relying on high-temperature thresholds.
6. Verify braking authority is retained when `l_temp_accel_dec` reduces acceleration torque.
7. Exercise optional RPM fault bits at low-current/low-energy conditions before enabling them for normal operation.
8. Test `foc_start_curr_dec` with a conservative factor and verify steering holding/launch torque remains sufficient.

This remains source/host verified, not hardware-proven.

## Deferred to Part 2/3

- ADC3 + DMA2 dedicated DCLINK/Vbus sampling.
- Any current-sampling-window retiming.
- TIM1/TIM8 BKIN hardware support pending schematic signal verification.
- static low-side short-on-zero-duty behavior pending gate-driver/bootstrap validation.
- HFI, fake phase-voltage sensing, and protocol-version migration remain intentionally excluded.
