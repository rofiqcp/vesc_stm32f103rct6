# TAHAP 1 REVISION NOTES — v29 APP ADC + COMMAND ARBITRATION

## Scope
Base: `vesc_stm32f103rct6_PART2_v28_modern_observers_hardening.zip`

Tahap 1 intentionally focuses on:
- restore target-build correctness after the `encoder_deinit()` declaration regression,
- add production-oriented APP ADC on PA2/PA3,
- keep APP UART on USART3 PB10/PB11,
- add command-source arbitration,
- preserve the proven current-sense FOC timing boundary,
- extend real diagnostics/telemetry without dummy values,
- keep VESC 6.00 wire ABI.

Out of scope for this stage: dynamic V0/V7 sampling, full overmodulation, VESC7 schema migration, bootloader, HFI, physical CAN, IMU, BMS, bm_if, NRF, LEDPWM, COMUSB, QMLUI, LispIF, NTC temp, LZO.

## 1. Build regression fixed
`src/comm/commands.c` now includes `encoder/encoder.h`, so `encoder_deinit()` has its owning prototype. This directly addresses the user's PlatformIO failure:

```
error: implicit declaration of function 'encoder_deinit'
```

## 2. APP ADC hardware mapping
APP ADC uses the user-specified pins:
- PA2 / ADC channel 2 = APP ADC1 (default throttle)
- PA3 / ADC channel 3 = APP ADC2 (default brake)

The proven hard-current boundary is intentionally preserved:
- ADC1/ADC2 ranks 1..3 remain current-sense ranks.
- DMA half-transfer after rank 3 remains the FOC trigger.
- PA2/PA3 are placed at slow rank 6 after that boundary.
- No new ADC peripheral, DMA channel, ISR, or RTOS task is added.

APP ADC is serviced from the existing 1 kHz motor-service task to minimize SRAM usage.

## 3. APP ADC runtime
New modules:
- `src/applications/app_adc.c/.h`
- `src/applications/app_command.c/.h`

APP ADC pipeline:
raw -> volts -> optional filter -> range/rail validation -> normalize -> deadband -> throttle curve -> ramp -> arbitration -> motor-control API.

Implemented behavior:
- Duty
- Current
- Current + Brake (dual ADC)
- RPM/PID-speed command
- safe-start neutral dwell
- low/high range and rail fault detection
- implausible transition detection
- throttle inversion and ADC2 inversion
- linear / exponential-style throttle curves using VESC-compatible configuration fields
- positive/negative ramping
- brake priority over positive throttle
- single-motor or local dual-motor command using `multi_esc`

No APP ADC path writes TIM CCR/PWM registers directly.

## 4. Safe-start hardening
Safe-start neutral dwell is based on wall time (`now_ms`) rather than number of APP updates. Therefore a configured 500 ms dwell remains 500 ms even when the APP ADC update rate is changed.

UART preemption, application reconfiguration, invalid ADC, and output-disable conditions revoke ADC ownership and require neutral re-arm before ADC can regain torque control.

## 5. Command arbitration
Per-motor source ownership now distinguishes:
- NONE
- APP ADC
- UART / VESC Tool
- calibration
- motor detection
- internal control

UART explicit motor commands preempt ADC and are kept alive with command freshness / `COMM_ALIVE`. On timeout, stale UART torque is stopped and ADC must safely re-arm before regaining ownership.

Relevant UART setters are routed through arbitration before entering the central motor-control API.

## 6. VESC6 APPCONF
The existing 493-byte VESC6 APPCONF ABI is preserved.

Typed APP ADC fields are now decoded/encoded from the canonical VESC6 byte locations, including control mode, voltage mapping, filter, safe-start, inversion, throttle curve, ramps, `multi_esc`, traction-control fields, update rate, and UART baud.

Supported application selectors:
- APP_NONE
- APP_ADC
- APP_UART
- APP_ADC_UART

Unsupported application/control backends are rejected rather than falsely acknowledged.

Changing APPCONF while PWM or motor detection is active is rejected. A valid configuration change invalidates stale source leases and forces safe ADC re-arm.

## 7. `COMM_GET_DECODED_ADC`
Implemented with real PA2/PA3 data. The response contains canonical four values:
- decoded ADC1
- ADC1 voltage
- decoded ADC2
- ADC2 voltage

No fabricated zeros are returned before a real rank-6 ADC sample has become available.

## 8. Diagnostic safety bug fixed
A review found a serious stack-headroom bug in `vesc_comm_reply_diag()`:
- buffer was 128 bytes,
- diagnostic revision 10 payload could reach ~132 bytes.

The buffer is increased to 160 bytes and diagnostic revision is bumped to 11. Revision 11 appends real APP ADC and arbitration status while preserving backward parsing of older diagnostic revisions.

## 9. Existing hardware invariants preserved
Unchanged by this stage:
- LEFT PWM timer = TIM8
- RIGHT PWM timer = TIM1
- high-side output active HIGH
- low-side complementary output active LOW
- current ranks 1..3 and FOC DMA-half-transfer timing
- fixed-point 16 kHz FOC hard path
- LEFT Hall / Encoder A-B / Sensorless
- RIGHT Hall / Sensorless only
- USART3 TX PB10 / RX PB11
- virtual local Motor2 forwarding
- transactional flash configuration

## 10. Verification completed in this environment
- 23/23 Python regression scripts PASS.
- APP ADC real C runtime harness PASS.
- Real 493-byte APPCONF round-trip/rejection harness PASS.
- GCC strict-warning compile tests PASS for new portable application modules.
- GCC `-fanalyzer` checks PASS for APP ADC, command arbitration, app config, and config generator.
- ZIP integrity is checked before delivery.

## 11. Verification limitation
This environment does not provide PlatformIO or `arm-none-eabi-gcc`, therefore an actual STM32F103 target link cannot be claimed here.

The user should run:

```
pio run
```

on the target machine. The specific `encoder_deinit()` regression that blocked the previously supplied Part 2 source is corrected by including its owning header.

Because the earlier successful target build already used approximately 95.9% of SRAM, the post-patch target RAM/stack headroom must be checked carefully before hardware testing.

## 12. Required hardware tests before enabling loaded motor operation
1. Build/link and inspect RAM/Flash usage.
2. Verify PA2 and PA3 voltage/readback with motor PWM disabled.
3. Confirm current ADC rank1..3 waveform/timing is unchanged.
4. Validate APP ADC low/high rail faults.
5. Validate safe-start with throttle active during boot.
6. Validate neutral -> armed transition.
7. Validate throttle current, duty and RPM mapping at low limits.
8. Validate PA3 brake priority and regenerative-current sign.
9. Validate UART preemption and UART-timeout -> ADC neutral re-arm.
10. Validate LEFT/RIGHT independence and `multi_esc` behavior.
11. Scope TIM8/TIM1 complementary PWM and dead-time before higher-current tests.
