# STAGE 2 v30 — Sensorless / Detection / Safety Hardening

Base: `vesc_stm32f103rct6_STAGE1_v29_appadc_arbitration.zip`

## Scope
- True sensorless forced-angle -> observer blending.
- Sensorless acquisition validation before Detect-All fallback is committed.
- Separate physical sensor-mux status from logical FOC sensor source in telemetry.
- LEFT Encoder A/B steering position uses extended non-wrapped mechanical position.
- Flash first-boot virgin state is distinguished from corrupt nonblank configuration.
- Fault-manager heartbeat is included in watchdog health gating.
- Local SENSORLESS_OBSERVER fault after repeated acquisition failures.
- Diagnostic/config-status stack-buffer hardening.

## Sensorless
The 1-kHz startup state machine now keeps `phase_observer_override` active while it applies a bounded signed phase correction toward the observer. It only releases the override within ~1 electrical degree. The 16-kHz FOC ISR is unchanged and remains fixed-point.

Repeated actual startup lock timeouts increment `sensorless_start_failures`; after three failures the motor service raises `MOTOR_FAULT_SENSORLESS_OBSERVER`.

## Detect-All
LEFT remains Encoder A/B -> Hall -> Sensorless. RIGHT remains Hall -> Sensorless. Sensorless fallback now commands a bounded positive-current acquisition test and requires observer-valid, speed threshold, completed blend, and 150 ms stable acquisition before the runtime config may be committed. Failure returns detection error and the known-good MCCONF is restored by existing rollback logic.

## Steering position
For LEFT Encoder A/B, set-position and position PID now use the extended encoder mechanical coordinate without 0/360 shortest-path wrapping. Hall and sensorless modes retain circular VESC semantics.

## Configuration integrity
`conf_general` now reports `CONF_BOOT_VIRGIN`, `CONF_BOOT_VALID`, or `CONF_BOOT_CORRUPT`. Completely erased flash is a normal first boot. Nonblank flash with no valid transactional record latches `MOTOR_FAULT_FLASH_CONFIG` on both motors while communication remains available for repair.

## Watchdog
A fourth heartbeat owner, `TIMEOUT_HEARTBEAT_FAULT`, monitors the dedicated fault-stop thread. The fault thread uses a finite 50 ms wait so liveness remains observable even when no fault is pending.

## Telemetry
`motor_telemetry_t` now carries both physical `sensor_mode` and logical `foc_sensor_mode`. Custom extended/sensor diagnostics append the logical source and sensorless failure count without changing the VESC 6.00 standard wire ABI.

## Buffer hardening
The custom config-status payload previously used a 16-byte stack buffer although the serialized payload exceeded that size. It is now 32 bytes. COMM_DIAG has 192 bytes of headroom and revision 12 adds fault-heartbeat and config boot-state diagnostics.

## Validation
- 24/24 Python regression scripts PASS.
- Existing strict host syntax/warning checks PASS through the regression suite.
- Observer runtime synthetic tests PASS.
- APP ADC runtime + 493-byte APPCONF roundtrip tests PASS.
- VESC Tool dual-Motor source regressions PASS.
- Host `sizeof(MotorRuntime)` remains 2128 bytes before/after Stage 2.
- Host `sizeof(motor_telemetry_t)` remains 200 bytes before/after Stage 2.

## Build status
PlatformIO / arm-none-eabi-gcc are not installed in this execution environment, so STM32F103 target build is **REQUIRES USER BUILD TEST**. Run `pio run -t clean && pio run` on the target project before hardware testing.

## Hardware tests required
1. Gate polarity / dead-time / MOE scope check.
2. Current offset and current polarity.
3. LEFT Encoder A/B no-wrap position across 0/360 and reverse.
4. Sensorless open-loop -> blend -> observer handover forward/reverse.
5. Detect-All fallback with physical Hall/encoder disconnected.
6. Sensorless acquisition failure must not save MCCONF.
7. Corrupt one flash record CRC and verify FLASH_CONFIG fault/defaults.
8. Watchdog fault-task stall injection where practical.
