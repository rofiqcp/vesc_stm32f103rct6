# Batch 11 Part 2 — VESC Tool Command Completeness, Config Integrity, and Runtime Cleanup

Baseline: `vesc_stm32f103rct6_batch11_part1_core_detect_cal_telemetry.zip`
Firmware name: `vesc-f103-hoverboard-v26`
Wire compatibility: VESC 6.00 (`MCCONF=481`, `APPCONF=493`)

## Scope

Part 2 closes practical runtime/diagnostic gaps after the Part-1 core Detect-All/calibration/telemetry work. It deliberately does **not** add a bootloader, does not change the TIM1/TIM8 PWM topology, does not retime ADC1/ADC2/ADC3, and does not alter the 16-kHz fixed-point hard FOC schedule.

Excluded ecosystems remain absent: HFI, physical CAN hardware, IMU, BMS/bm_if, NRF, VESC LEDPWM, COM_USB, QML UI, lispif, NTC runtime, and LZO.

Implemented in Part 2:

1. VESC-style `COMM_SET_BATTERY_CUT` / `COMM_GET_BATTERY_CUT`, adapted atomically for the two local motors.
2. `COMM_FW_INFO` with truthful local metadata.
3. Safe board-level `COMM_SHUTDOWN` / restart behavior using the existing PA5 power-hold circuit.
4. Idle-only periodic integrity checking of the transactional Flash configuration record.
5. Canonical VESC flash-corruption fault mapping and clear/repair semantics.
6. Non-blocking buzzer/tone backend and removal of fake sample-audio behavior.
7. Correct classification of VESC `cc_*` fields that are BLDC-only upstream and therefore non-applicable to this FOC-only port.
8. Diagnostic revision 9 with configuration-integrity and power-latch state.

## 1. Battery-cut GET/SET

The VESC command payload is preserved:

- start voltage: signed 32-bit fixed scale 1e3
- end voltage: signed 32-bit fixed scale 1e3
- store flag
- forward-all flag

For this no-CAN dual controller, `forward-all` on the directly connected M1 is interpreted as **apply to both local motors**. A command already forwarded to virtual M2 applies only to M2 because there is no downstream bus.

When `store=true`, the operation is executed in the blocking worker and both local MCCONFs are written through one transactional configuration commit. Runtime configurations are restored on failure.

To protect the approximately 3-KiB blocking-task stack, the temporary wire images are not nested local arrays. Part 2 uses static worker-owned BSS scratch storage:

- two 481-byte backup images;
- one 481-byte work image.

This trades about 1443 bytes of deterministic BSS for much lower stack-overflow risk on the 48-KiB STM32F103RCT6 SRAM.

## 2. COMM_FW_INFO

`COMM_FW_INFO` now reports:

- firmware major 6;
- firmware minor 0;
- test version 0;
- empty NUL-terminated upstream git hash;
- empty NUL-terminated user git hash.

The hashes are intentionally empty because this generated port is not claiming a real upstream git commit identity. `COMM_FW_VERSION` remains the primary VESC Tool compatibility handshake.

## 3. Safe shutdown / restart

`COMM_SHUTDOWN` is implemented without adding a VESC bootloader.

Because both inverters share one power domain, shutdown checks **both** M1 and M2 speeds. Unless `force=1`, shutdown/restart is refused while either motor exceeds 100 ERPM magnitude.

Accepted shutdown performs:

1. latch `s_shutdown_latched` so new motor/detection commands cannot restart either bridge;
2. stop both motor controllers;
3. emergency-clear TIM1/TIM8 MOE;
4. stop the buzzer/tone output;
5. force status LED off;
6. wait briefly for outputs to settle;
7. restart with `NVIC_SystemReset()` when requested, otherwise drop PA5 power hold.

If external power prevents the logic rail from collapsing after PA5 is released, the shutdown latch continues to reject motor-driving and detection commands.

## 4. Transactional configuration integrity supervision

The existing Flash-emulated configuration record already carries a CRC and sequence number. Part 2 turns that into a low-overhead runtime integrity service instead of adding a separate firmware-image CRC/bootloader mechanism.

`conf_general_service_100hz()` performs a scrub every 100 invocations (1 Hz) only when:

- both bridges are idle;
- no PWM is active;
- neither motor owns an active command.

The scrub revalidates the transactional record and requires the newest valid sequence to match the last committed sequence. Falling back to an older valid page does not silently count as healthy.

On corruption:

- both bridges are emergency-disabled;
- both motors receive `MOTOR_FAULT_FLASH_CONFIG`;
- the public VESC fault is `FAULT_CODE_FLASH_CORRUPTION`;
- the configuration-valid state becomes false;
- `motor_clear_fault()` refuses to hide the fault while integrity is still bad.

A newly written and successfully verified replacement record clears the integrity latch, after which the user may clear the motor fault normally. Thus the fault is recoverable by a valid configuration repair rather than requiring a permanent reset-only state.

This service never executes in the 16-kHz hard FOC ISR and is intentionally deferred while driving.

## 5. Audio/status cleanup

The stock hoverboard adaptation has a simple PA4 buzzer/status-tone output, not VESC motor-phase sample-audio hardware. Previous compatibility functions could appear to support arbitrary audio samples while only toggling the simple buzzer.

Part 2 removes that false behavior:

- `mcpwm_foc_beep()` is non-blocking and schedules a real timed tone;
- `mcpwm_foc_play_tone()` uses the requested frequency;
- `mcpwm_foc_stop_audio()` stops the tone backend;
- arbitrary sample-table storage/playback explicitly returns unsupported instead of retaining fake state.

Tone generation is IRQ-counted through StatusIO and does not call `osDelay()` from the VESC-compatible motor API.

The Part-1 power-on melody and canonical LED+buzzer fault-code signaling remain intact.

## 6. Current-controller field semantics corrected

The pinned VESC6 wire image still contains:

- `cc_startup_boost_duty`;
- `cc_min_current`;
- `cc_gain`;
- `cc_ramp_step_max`.

Current upstream VESC uses `cc_startup_boost_duty`, `cc_gain`, and `cc_ramp_step_max` in the **trapezoidal BLDC** `mcpwm.c` current-to-duty controller. The FOC backend `mcpwm_foc.c` uses `cc_min_current` for release/current-off semantics and does not consume those three BLDC current-ramp fields.

Therefore this FOC-only STM32F103 port now makes the distinction explicit:

- `cc_min_current`: real FOC backend, writable;
- the other three fields: byte-preserved for VESC6 ABI, immutable/non-applicable to FOC;
- no fake FOC behavior is invented simply to make every wire field appear active.

## 7. Diagnostic revision 9

Custom communication diagnostics now append:

- configuration-integrity OK flag;
- number of completed integrity checks;
- number of integrity failures;
- PA5 power-hold state;
- shutdown-latched state.

The prior diagnostic prefix remains intact. `tools/debug.py` parses revision 9 and its self-test covers the appended fields.

Terminal command `integrity` reports the same persistent-state counters.

## 8. Preserved Part-1/core behavior

This batch does not change:

- VESC Tool discovery of direct M1/ID1 plus virtual M2/ID2;
- dual atomic Detect-All and fresh six-channel current calibration;
- LEFT Hall/ABI selection and RIGHT Hall-only validation;
- R/Ld/Lq/flux/current-PI/sensor detection;
- duty/current/brake/RPM/position/handbrake/current-relative command backends;
- standard and selective telemetry for both motors;
- TIM1/TIM8 complementary gate polarity;
- ADC1/ADC2 current sampling;
- ADC3/DMA2 Vbus sampling;
- 16-kHz fixed-point FOC;
- MTPA, VESC7 fast field weakening, observer, PLL, Hall rate limiting, encoder slip;
- PVD, optional BKIN, short-low-side backend;
- USART3 DMA and IWDG heartbeats.

## 9. Verification

All source/host regressions pass together after the final Part-2 changes:

- `tools/verify_vesc_port.py`
- Batch 2 through Batch 8
- Batch 9 Part 1/2/3
- Batch 10 Part 1/2
- VESC Tool dual-basic regression
- Batch 11 Part 1
- `tools/test_batch11_part2.py`
- `tools/debug.py --self-test`

Strict host GCC checks pass for:

- `src/motor/mc_interface.c`
- `src/motor/mcpwm_foc.c` (with the existing host-stub DMA symbol alias)
- `src/confgenerator.c`
- `src/terminal.c`

using `-Wall -Wextra -Wshadow -Wdouble-promotion -Wformat=2 -Werror`.

`commands.c`, `conf_general.c`, and `status_io.c` are covered by source/regression tests but are **not claimed** as full strict host translation-unit builds because the current host HAL/RTOS stubs do not define all CMSIS-RTOS2 and STM32 register symbols used by those modules.

PlatformIO / `arm-none-eabi-gcc` and physical hardware are not available in the audit environment, so a target ARM build and board commissioning are still mandatory before full-power use.

## 10. Commissioning checks after flashing

1. Verify `COMM_FW_VERSION` shows VESC 6.00 and firmware name `vesc-f103-hoverboard-v26`.
2. Scan and confirm direct M1 plus local forwarded ID2.
3. Run Part-1 passive dual-basic checker before spinning either motor.
4. Verify fresh current calibration and both motor Detect-All at low-energy/current limits.
5. Confirm battery-cut GET/SET for M1 and M2, then test store once at logic/low-power conditions.
6. Test shutdown/restart only with wheels unloaded; verify PA5 drops and both MOEs are already off first.
7. Deliberately corrupt a test configuration record only on a disposable/test build and confirm `FAULT_CODE_FLASH_CORRUPTION` behavior before relying on the integrity service.
8. Verify power-on melody, fault LED pulses, and buzzer pulses.
9. Only after the above, proceed to loaded duty/current/speed/position tests.
