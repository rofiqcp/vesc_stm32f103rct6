# Batch 11 Part 1/2 — Core VESC Tool Readiness, Dual Detect-All, Fresh Calibration, Telemetry and Fault Status

Baseline: `vesc_stm32f103rct6_batch10_part2_vesc_tool_dual_basic.zip`
Revision identity: `vesc-f103-hoverboard-v25`
VESC wire target: **6.00** (MCCONF 481 bytes / APPCONF 493 bytes)

## Scope

This part deliberately prioritizes the path required to make the stock STM32F103RCT6 hoverboard controller usable from VESC Tool before adding secondary commands or hardware-retiming optimizations.

No bootloader is added. Firmware update remains an external PlatformIO/ST-Link operation.

Excluded ecosystems remain excluded: physical CAN, HFI, IMU, BMS/bm_if, NRF, LEDPWM, COM_USB/USB, QML UI, lispif/Lisp, NTC runtime and LZO.

The hardware timing contract is unchanged:

- RIGHT inverter: TIM1.
- LEFT inverter: TIM8.
- TIM1/TIM8 complementary PWM, center aligned.
- high-side timer output active HIGH.
- complementary low-side timer output active LOW.
- ADC1/ADC2 dual current acquisition, DMA1 Channel1 HT FOC entry.
- ADC3/DMA2 Channel5 same-trigger Vbus acquisition.
- fixed-point hard FOC at 16 kHz.
- USART3 DMA VESC transport.

## 1. VESC Tool dual-controller model

The direct serial controller remains Motor-1 / LEFT / ID1. `COMM_PING_CAN` advertises one local forwarded node, Motor-2 / RIGHT / ID2. There is no physical CAN hardware.

The standard VESC Tool setup telemetry still reports `num_vescs = 2`, and standard per-motor telemetry contains the selected controller ID.

## 2. Full local dual-motor `COMM_DETECT_APPLY_ALL_FOC`

VESC Tool Board Setup invokes Detect-All on the directly connected controller with `detect_can=true`. The previous firmware rejected that mode because no physical CAN backend exists. Part 1 changes the meaning for this board:

`detect_can=true` on the direct controller means **detect Motor-1 plus the locally forwarded Motor-2**.

The transaction is:

1. Validate Detect-All input parameters.
2. Ignore normal motor commands on both motors.
3. Stop both inverters.
4. Force a fresh current-offset calibration.
5. Detect/apply FOC electrical parameters on LEFT.
6. Calibrate the selected LEFT physical sensor:
   - LEFT Encoder mode -> ABI encoder detection/alignment.
   - LEFT Hall mode -> Hall table detection.
7. Detect/apply FOC electrical parameters on RIGHT.
8. Calibrate RIGHT Hall table.
9. Validate/synchronize both runtime configurations.
10. Persist both configurations with one transactional flash commit.
11. Release command gating only after transaction completion.

The VESC Tool reply remains exactly `COMM_DETECT_APPLY_ALL_FOC + int16 result`.

### Failure policy

The previously committed LEFT and RIGHT MCCONF wire images are snapshotted before the dual commit. If LEFT detection, RIGHT detection, validation, configuration application or flash storage fails, both motor runtimes are restored to the last committed MCCONFs. A partially detected M1 must never remain active if M2 fails, and vice versa.

## 3. Fresh six-channel current calibration before Detect-All

Detect-All no longer accepts a stale `calibration_valid` result left over from boot. It requests a new calibration transaction and waits until that new transaction has actually entered the active state and completed successfully.

The existing calibration engine is retained because it already matches the stock hoverboard current-sense hardware:

- LEFT phase-current U/V and LEFT physical DC current.
- RIGHT phase-current U/V and RIGHT physical DC current.
- six undriven offset channels.
- LEFT driven 50/50/50 zero-vector calibration.
- RIGHT driven 50/50/50 zero-vector calibration.
- ADC-rail range checking.
- current-noise/variance diagnostics.
- driven-vs-undriven shift diagnostics.
- Vbus stability supervision.
- gross DC-current safety during driven calibration.

The driven offsets remain the offsets consumed by the current PI, as they are measured in the same switching condition as FOC current sampling.

## 4. Sensor ownership

Hardware policy is preserved at all layers:

### LEFT

- Hall, or
- incremental ABI encoder.

If Detect-All runs while LEFT is configured Encoder, encoder ratio/direction/electrical offset are detected and applied. If LEFT is configured Hall, a Hall electrical-angle table is detected and applied.

### RIGHT

RIGHT is Hall-only. Configuration validation rejects Encoder on RIGHT, and the hard FOC sensor setter also forces an invalid RIGHT-Encoder request back to Hall. This is intentionally redundant defense.

The internal sensorless observer remains available to the FOC engine for observer takeover/phase estimation. It is not opened as an unsupported persistent primary sensor selection.

## 5. Main VESC motor-control commands

The following standard commands remain routed per selected M1/M2 runtime and pass the common input/ESTOP/application-disable ownership gate:

- `COMM_SET_DUTY`
- `COMM_SET_CURRENT`
- `COMM_SET_CURRENT_BRAKE`
- `COMM_SET_RPM`
- `COMM_SET_POS`
- `COMM_SET_HANDBRAKE`
- `COMM_SET_CURRENT_REL`

The command path remains non-blocking. Detection/calibration commands are moved to the blocking worker and ordinary control commands are gated during a Detect-All transaction.

## 6. Individual VESC Tool calibration commands

The standard individual motor wizard paths remain available:

- `COMM_DETECT_MOTOR_R_L`
- `COMM_DETECT_MOTOR_FLUX_LINKAGE`
- `COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP`
- `COMM_DETECT_ENCODER`
- `COMM_DETECT_HALL_FOC`
- `COMM_DETECT_APPLY_ALL_FOC`

Stock hoverboard hardware does not independently ADC-measure all three phase voltages. Therefore zero-current passive flux measurement is not fabricated. Open-loop flux detection requires nonzero commanded drive current.

## 7. Standard VESC Tool telemetry

Both M1 and forwarded M2 support:

- `COMM_GET_VALUES`
- `COMM_GET_VALUES_SELECTIVE`
- `COMM_GET_VALUES_SETUP`
- `COMM_GET_VALUES_SETUP_SELECTIVE`

The pinned VESC6 mask bits 0..21 are populated. Relevant telemetry includes:

- motor current
- physical input/DC current
- Id / Iq
- duty
- ERPM
- Vbus
- amp-hours / charged amp-hours
- watt-hours / charged watt-hours
- tachometer / absolute tachometer
- canonical VESC fault code
- position
- controller ID
- Vd / Vq
- timeout state
- setup speed/distance/battery/odometer/uptime fields
- `num_vescs = 2`

Because NTC/FET/motor temperature hardware is excluded, the standard temperature slots explicitly return the unavailable sentinel rather than falsely exposing the STM32 internal board-temperature proxy as MOSFET or winding temperature.

## 8. `COMM_SET_APPCONF_NO_STORE`

Part 1 adds the standard no-store APPCONF write path. It shares validation with normal APPCONF, but passes `store=false`, so VESC Tool can temporarily apply application settings without creating a flash write. Motor-2 public ID2 validation remains enforced before normalization to the one shared physical application/communication layer.

## 9. Synchronized fault LED + buzzer

The status task now uses one canonical VESC fault number as the source for both the LED and buzzer. The two outputs therefore cannot drift into different fault-code tables.

The decimal VESC fault number is rendered as pulse groups:

- tens digit pulses at 1500 Hz,
- group pause,
- ones digit pulses at 2400 Hz,
- long repeat pause.

Digit zero is represented by ten pulses. `MOTOR_FAULT_ENCODER_SLIP` is included. A fault immediately preempts the startup melody.

The non-blocking startup melody remains 900 Hz -> 1350 Hz -> 1900 Hz.

## 10. Gate polarity contract

The firmware keeps:

- `OCPolarity = TIM_OCPOLARITY_HIGH`
- `OCNPolarity = TIM_OCNPOLARITY_LOW`
- high-side idle RESET/LOW -> off
- complementary low-side idle SET/HIGH -> off

This matches the EFeru stock hoverboard STM32F103 FOC setup. Actual gate-driver pins and phase nodes still require oscilloscope validation before full-power commissioning.

## 11. RTOS2 ownership

No extra RTOS thread was added for Detect-All. The existing blocking detection/config worker owns the complete transaction, while the 16-kHz FOC path remains in the ADC/DMA ISR. Ordinary motor input is gated for the duration. This avoids extra stack and context-switch overhead on STM32F103.

The existing task architecture remains:

- hard ADC/DMA FOC ISR: 16 kHz fixed point
- motor-service task: 1 kHz outer-control/limits/slow FOC work
- packet parser / USART DMA processing
- blocking detect/config worker
- fault-stop worker
- sample sender
- non-blocking status LED/buzzer task
- watchdog liveness supervision

## 12. Persistence

There is no true EEPROM peripheral. Configuration uses the existing transactional STM32F103 flash record with CRC. Dual Detect-All updates both MCCONFs in one `conf_general_store_all()` transaction. The VESC6 wire ABI remains 481/493 bytes.

## 13. Verification

Passed after this revision:

- `tools/verify_vesc_port.py`
- Batch 2 through Batch 8 regressions
- Batch 9 Part 1/2/3 regressions
- Batch 10 Part 1/2 regressions
- `tools/test_vesc_tool_dual_basic.py`
- new `tools/test_batch11_part1.py`
- `tools/debug.py --self-test`

The new Part-1 regression explicitly checks dual Detect-All ordering/rollback, fresh calibration ownership, six calibration channels, driven zero-vector stages, LEFT/RIGHT sensor policy, all main control commands, standard telemetry coverage, APPCONF no-store semantics, standard motor-wizard detection commands, fault signaling, MOSFET polarity, fixed-point ISR and subsystem exclusions.

`src/confgenerator.c` passes strict host `gcc -Wall -Wextra -Wshadow -Wdouble-promotion -Wformat=2 -Werror` syntax checking with the project host stubs. `commands.c` and RTOS/hardware translation units cannot be honestly strict-host-compiled with the intentionally incomplete CMSIS/STM32 stubs; no ARM build is claimed.

## 14. Physical validation still mandatory

This audit environment has no `pio` and no `arm-none-eabi-gcc`, and no physical hoverboard board is attached. Before calling the firmware hardware-proven:

1. Run the real PlatformIO ARM build.
2. Power logic only; confirm FW 6.00 / v25 and ID2 discovery.
3. Verify TIM1/TIM8 high-side/low-side gate polarity and dead-time with an oscilloscope.
4. Verify fresh current calibration completes with realistic six-channel offsets/noise.
5. Verify ADC3 Vbus against a DMM.
6. Run Detect-All at current-limited bus power.
7. Test LEFT Hall, then LEFT Encoder as separate configurations.
8. Test RIGHT Hall only.
9. Verify M1 and M2 telemetry in VESC Tool before enabling torque.
10. Test current, brake-current, duty, speed and position commands at low current first.

## Deferred to Part 2

Part 2 should close remaining core-completeness items without adding a bootloader: battery-cut GET/SET, clean shutdown/power-hold semantics if desired, remaining meaningful current-controller compatibility fields, firmware/config integrity supervision, audio compatibility stub cleanup, command-explicit unsupported behavior, and additional telemetry/diagnostic polish. Dynamic ADC-trigger/current-window retiming should remain a separate hardware-performance stage because it requires oscilloscope qualification.
