# PART 1 REVISION NOTES — Sensorless + VESC-like Sensor Auto Detect

Firmware identity: `vesc-f103-hoverboard-v27-part1`
Wire protocol/config ABI: **VESC 6.00** (intentionally unchanged)
Target: STM32F103RCT6 hoverboard controller, 48 KiB SRAM

## Scope completed in Part 1

This revision intentionally changes only the Part-1 motor-control/sensor-selection path. Dynamic ADC sample retiming, full modern overmodulation, VESC-7 wire-schema migration, additional observer families, and bootloader/firmware-update support remain deferred to Part 2.

### 1. Hardware contract retained

- LEFT inverter remains TIM8.
- RIGHT inverter remains TIM1.
- High-side timer output remains active HIGH (`TIM_OCPOLARITY_HIGH`).
- Complementary low-side timer output remains active LOW (`TIM_OCNPOLARITY_LOW`).
- Idle states remain high-side RESET and low-side SET, leaving both gate inputs in their OFF level when MOE is disabled.
- Hard 16-kHz FOC/ADC path remains fixed-point; Part-1 startup timing/ramp math runs in the existing 1-kHz task context.

### 2. Sensor mode policy completed

VESC FOC sensor mode validation now allows:

- LEFT: SENSORLESS / HALL / incremental ENCODER A/B.
- RIGHT: SENSORLESS / HALL.

RIGHT still rejects physical encoder mode. HFI modes remain explicitly rejected.

Sensorless is an observer control mode, not an added physical input. On RIGHT the only physical rotor sensor input remains Hall.

### 3. VESC-like sensor discovery

`COMM_DETECT_APPLY_ALL_FOC` no longer trusts the sensor selected before detection.

After electrical R/L/flux detection:

LEFT:
1. Try incremental A/B encoder detection.
2. If the encoder is physically absent (and there is no real motor/power-stage fault), try Hall detection.
3. If Hall is also absent, select SENSORLESS.

RIGHT:
1. Try Hall detection.
2. If Hall is physically absent (and there is no real motor/power-stage fault), select SENSORLESS.

A genuine motor/power-stage fault still aborts Detect-All. Dual local Motor-1/Motor-2 Detect-All keeps the previous atomic commit/rollback behavior.

The custom AUTO sensor-detect worker follows the same LEFT encoder -> Hall -> sensorless and RIGHT Hall -> sensorless policy. An explicit encoder request remains explicit and does not silently downgrade to another sensor.

### 4. Pure sensorless startup/handover

A 1-kHz sensorless startup state path was added without introducing float math into the 16-kHz hard FOC ISR.

Behavior:

- A stopped sensorless motor starts on a forced electrical phase/open-loop sequence.
- Startup uses a VESC-style 60 electrical-degree initial phase lead and seeds the observer a further 45 electrical degrees in the torque direction.
- Lock/ramp time does not advance until PWM/MOE is actually enabled and current-sense enable blanking has completed.
- Forced electrical speed ramps high enough to cross the configured sensorless observer handover threshold.
- Startup Q current includes configured open-loop Q boost and is bounded by configured `foc_sl_openloop_max_q`.
- Handover requires a valid observer, sufficient BEMF-derived speed, and phase coherence (<= 60 electrical degrees) between forced phase and observer phase.
- A failed lock does not claim success; torque is zeroed before forced phase is released and startup is re-armed.
- A direction reversal during forced startup re-arms alignment rather than continuing the old phase trajectory backwards/forwards with stale alignment.

### 5. Sensorless zero-speed safety

Without HFI or a physical rotor sensor, absolute rotor electrical position at standstill is not observable. Part 1 therefore intentionally does **not** invent a sensorless zero-speed position.

- Sensorless CURRENT / SPEED / DUTY can execute the open-loop -> observer startup path.
- Sensorless POSITION / BRAKE / HANDBRAKE only operate when the observer is already valid above the minimum observer-speed threshold.
- If that condition is not true, Id/Iq targets are forced to zero.

This is a safety/physics constraint, not a missing zero-speed-position algorithm.

### 6. Fault indication cleanup

`MOTOR_FAULT_COMMAND_TIMEOUT` is no longer included in the audible/visible VESC fault-code pulse selector because it maps to `FAULT_CODE_NONE` in the VESC compatibility layer. This prevents a non-VESC timeout state from being represented as a fake decimal fault-code pulse sequence.

Actual mapped faults continue to use the shared LED + buzzer pulse state machine.

### 7. VESC Tool compatibility retained

- VESC wire ABI remains 6.00.
- MCCONF/APPCONF wire sizes remain unchanged.
- USART3 remains the communication transport.
- Motor-1 / local forwarded Motor-2 behavior remains intact.
- Main SET duty/current/brake/current-rel/RPM/position/handbrake handlers are unchanged.
- Telemetry, config persistence, dual Detect-All rollback, ADC/DMA safety, power-stage latch, and 48-KiB SRAM policy remain under the existing regression suite.

### 8. Explicitly not added in Part 1

Still excluded exactly as requested:

- HFI
- physical CAN hardware
- IMU
- BMS / bm_if
- NRF
- LEDPWM
- COMUSB
- QMLUI
- LispIF
- NTC temperature backend
- LZO

Also deferred to Part 2:

- dynamic ADC sample-point retiming/current-sampling modes
- full modern VESC overmodulation behavior
- MXV/MXLEMMING/lambda-comp observer families
- VESC-7 configuration schema migration
- VESC bootloader/firmware update
- motor-phase startup audio

## Files changed by Part 1

Core firmware:

- `src/confgenerator.c`
- `src/comm/commands.c`
- `src/motor/foc_math.c`
- `src/motor/foc_math.h`
- `src/motor/mc_interface.c`
- `src/motor_tasks.c`

Validation:

- `tools/test_batch11_part1.py` (old selected-sensor Detect-All expectation updated)
- `tools/test_batch11_part2.py` (firmware identity expectation updated)
- `tools/test_part1_sensorless_autodetect.py` (new Part-1 regression)

## Mandatory hardware validation before high-power use

Host/static regression cannot prove inverter switching safety. Before applying a high-current load, perform these checks on the actual board:

1. Scope UH/UL, VH/VL, WH/WL and verify high-side active-HIGH, low-side active-LOW, correct deadtime, and no overlap/shoot-through.
2. Validate all current-sense polarities and offsets at zero current and known positive/negative phase current.
3. Confirm ADC sampling remains inside the valid current-sense window across the allowed duty range.
4. Test LEFT encoder auto-detect and forward/reverse startup unloaded first.
5. Test LEFT Hall auto-detect and forward/reverse startup unloaded first.
6. Disconnect physical sensor(s) and verify LEFT/RIGHT Detect-All intentionally falls back to SENSORLESS without hiding a real hardware fault.
7. Validate sensorless open-loop startup and observer handover forward/reverse at low DC-bus/current limit before increasing power.
8. Verify Motor-2 forwarded VESC Tool SET/GET and telemetry after sensor mode changes.
9. Inject a failed Motor-2 Detect-All and verify both runtimes return to the last committed MCCONF.
10. Measure FOC ISR DWT worst-case execution time while telemetry and both motors are active; maintain safe margin below the 62.5-us 16-kHz deadline.
11. Validate regen/over-voltage/current limit behavior under a controlled mechanical load.

## Build qualification note

This environment does not contain PlatformIO or `arm-none-eabi-gcc`, so a real STM32 target link/flash build cannot be claimed here. The included host/static suite and strict GCC checks pass; the project must still be compiled with the user's normal STM32 PlatformIO toolchain and then qualified on hardware.
