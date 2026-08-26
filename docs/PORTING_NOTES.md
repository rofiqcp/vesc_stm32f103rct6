# STM32F103 Hoverboard VESC FOC Port — Applied Notes

## Compatibility target

This firmware intentionally advertises and serializes the **VESC 6.00** wire ABI.
The canonical configuration payload sizes/signatures are locked to:

- `mc_configuration`: 481 bytes, signature `776184161`
- `app_configuration`: 493 bytes, signature `486554156`

Do not change only the firmware version bytes to a newer VESC release. A newer
advertised version is valid only after the complete matching MCCONF/APPCONF wire
schema and command semantics have been ported and round-trip tested.

## Hardware ownership

The proven hoverboard timing layer remains board-specific:

- TIM1 / TIM8 complementary three-phase PWM
- TIM1/TIM8 synchronization and TIM8-triggered dual ADC sampling
- ADC1 + ADC2 dual-regular simultaneous conversion
- DMA1 Channel 1 current frame
- fixed-point FOC work in the ADC/DMA hard ISR
- USART3 on PB10/PB11 for the VESC packet transport

Generic VESC ADC/PWM hardware code must not replace this layer unless the board
is revalidated electrically and with scope captures.

## Motor / sensor policy

- LEFT motor: Hall or ABI quadrature encoder only.
- RIGHT motor: Hall only.
- LEFT default: 4 pole-pairs, 1024 PPR quadrature encoder.
- STM32 TIM4 encoder mode TI12 counts four edges per PPR, so the runtime
  `m_encoder_counts` default is 4096 counts/mechanical revolution.
- Unsupported sensor modes (HFI, SPI encoders, resolver, etc.) remain part of
  protocol enums where required for ABI compatibility, but are rejected by the
  hardware configuration validator.

## Algorithms applied to the F103 backend

The control path now includes the applicable VESC-style pieces without moving
floating-point math into the 16 kHz hard ISR:

- fixed-point Clarke/Park/current PI/inverse Park/SVM
- fixed-point flux observer and phase/speed estimation
- configurable fixed-point current filtering
- dq cross-coupling and back-EMF decoupling
- MTPA d-axis target generation in the 1 kHz slow loop
- field-weakening ramp in the 1 kHz slow loop
- d-axis-priority voltage-vector saturation for field weakening
- PI anti-windup after voltage saturation
- VESC-style duty down-ramp PI behavior
- motor-current, input-current, watt, ERPM, duty and low-battery limiting
- high-bus-voltage regenerative-current taper before hard over-voltage trip

The slow-loop ordering is:

`torque request -> MTPA -> field weakening -> input/watt limit -> motor current limit -> d/q vector limit`

Float-derived coefficients are precomputed task-side into Q-format values before
the ISR consumes them.

## Fault compatibility

Board-internal fault reasons and VESC protocol faults are separate types.
`mc_fault_code` uses the VESC numbering; local ADC/DMA/ISR diagnostics are mapped
to an appropriate VESC fault before they are sent to VESC Tool. This prevents a
local enum value from being displayed as the wrong VESC fault.

## VESC Tool communication

The port keeps standard packet framing/CRC and VESC command IDs for the
implemented VESC 6.00 subset. `COMM_FORWARD_CAN` is deliberately retained only
as a **local motor-2 protocol selector** so VESC Tool can address the right
motor. No physical CAN driver/PHY is compiled.

Excluded physical/runtime subsystems include CAN hardware, IMU, BMS/bm_if, NRF,
LEDPWM, USB communication, QML UI, Lisp, LZO and NTC-temperature runtime.
Protocol/schema fields that are required to preserve the VESC wire layout are
not deleted; unsupported mutable fields are rejected rather than silently
accepted.

## Telemetry and sampling

Standard VESC values are sourced from coherent motor snapshots. The debug
sampler supports NOW, START, start/fault trigger modes (including NOSEND), last
samples and single-sample behavior, with circular pre-trigger history.

Available internal sample data includes phase currents, Id/Iq, Vd/Vq, duty U/V/W,
rotor electrical phase, ERPM, Vbus, Hall state and raw current ADC values.

The stock hoverboard board does **not** measure all three motor phase voltages
with ADC channels. Phase-voltage values in the standard sample stream are
therefore reconstructed/applied values from duty and Vbus, not claimed as
three independent phase-voltage ADC measurements.

## Task layout

The periodic slow control work is consolidated into a 1 kHz motor-service task:

- slow FOC update
- speed/position/duty outer controllers
- FW/MTPA/limit updates
- command timeout
- periodic communication/telemetry housekeeping

Event-driven blocking detection, sample transmission and fault handling remain
separate where blocking them into the 1 kHz service would harm determinism.
The ADC/DMA FOC loop remains an ISR.

## Fixed-point representation limit

The current speed estimators use signed Q16.16 ERPM state. This F103 port clamps
configuration to a 30,000 ERPM hardware envelope so fixed-point speed state
cannot wrap.

## Verification included

Run:

```text
python tools/verify_vesc_port.py
```

The verifier checks the VESC 6.00 wire sizes/signatures, FW-version alignment,
absence of floating-point constructs in `foc_one_motor_isr`, excluded physical
modules, sensor policy, local motor-2 forwarding, left encoder defaults and the
hoverboard TIM1/TIM8 dual-ADC timing contract.

Host-side regression tests used during this revision also covered:

- exact 481/493-byte configuration layout/signatures
- short and long VESC packet round-trip plus CRC/noise recovery
- low-battery, regenerative and input-current limit math
- debug sampling NOW/START/fault-trigger/single-sample behavior
- strict `gcc` warning-as-error compilation of the core portable translation units

These host tests do **not** replace an ARM/PlatformIO build or physical-board
commissioning.

## Commissioning order

1. Build with the STM32F103 PlatformIO/ARM toolchain.
2. Power logic only and confirm USART3 `COMM_FW_VERSION` / config round-trip.
3. Confirm ADC zero-offset calibration with PWM MOE disabled.
4. Verify current scaling/polarity with a current-limited supply.
5. Test LEFT Hall at low current.
6. Test RIGHT Hall at low current.
7. Test LEFT ABI encoder alignment/synchronization at low current.
8. Verify braking and bus-voltage regenerative taper.
9. Only then enable/test MTPA, decoupling and field weakening under load.

Do not treat a successful host compile as proof of safe full-power operation.

## Batch 4 communication and watchdog update

USART3 now uses DMA1 Channel3 circular RX with IDLE/HT/TC draining and DMA1
Channel2 queued TX. This removes RXNE/TXE interrupt-per-byte traffic while
keeping packet parsing in task context and ADC/FOC DMA1 Channel1 at the highest
IRQ priority.

The controller now also gates STM32F1 IWDG feeding on independent FOC,
motor-service and communication heartbeats. Reset flags are captured at boot
and exposed through custom communication diagnostics. See
`BATCH4_REVISION_NOTES.md` and `tools/test_batch4.py`.

## Batch 5 — canonical API/structure alignment

Batch 5 adds canonical VESC-facing type/API names without changing the B1–B4
hard-control or hardware timing layers. `setup_values`/`setup_stats`, full
`COMM_PACKET_ID` numbering, reduced `app.c/app.h`, typed `confgenerator_*`,
typed `conf_general_*`, canonical `buffer_*`, and standard UART app wrappers
are now present. The actual persistent/wire contract remains pinned to VESC
6.00 (481-byte MCCONF / 493-byte APPCONF). Unsupported subsystems and settings
remain rejected at apply time rather than being silently accepted.

## Batch 6 — VESC Tool ecosystem

Batch 6 adds terminal/sync terminal support, canonical plot and experiment-sample packet helpers, temporary MCCONF commands, app-output-disable, real setup statistics, persistent odometer support, and `COMM_MOTOR_ESTOP` input gating. Persistent storage record version is now `0x0013`; legacy `0x0012` config records remain readable. Odometer-only and component-specific saves compose their record from the last committed flash config, preventing temporary MCCONF from being persisted accidentally while avoiding an additional persistent-config SRAM shadow.

Hard FOC, TIM1/TIM8/ADC timing, B3 sensor/observer algorithms, B4 USART3 DMA, IWDG, and VESC packet framing are unchanged from Batch 5.

## Batch 7 — Controller semantics

Batch 7 aligns the reduced STM32F103 controller layer more closely with VESC controller behavior without changing the hard hardware/FOC architecture. Speed control now supports setpoint ramp, minimum-ERPM release, VESC6 braking permission, normalized 1/20-scaled PID, and typed runtime PLL/FAST/FASTER speed sources. Advanced position-control fields (`kd_proc`, gain reduction, angular divisor, offset) are active. `cc_min_current` and `current_off_delay` now control bridge release/hold semantics, including the field-weakening current-off hold. Public encoder/Hall correction helpers are aligned with the active 5% encoder hysteresis and Hall-to-observer transition behavior.

The firmware remains pinned to VESC6 wire ABI. `s_pid_speed_source` is a runtime-only typed extension because VESC6 does not contain that field; standard VESC6 config always decodes to PLL and serialization refuses to silently persist FAST/FASTER.

## Batch 8 — Motor identification and protection

Batch 8 removes the previous `Ld-Lq = 0` limitation in motor auto-detection. The existing 16-kHz current-loop capture now supports both d and q axes; the blocking detection worker electrically locks the rotor, measures `Ld`, then uses short alternating q-axis pulses to measure `Lq`. Operational HFI remains disabled and the TIM1/TIM8 + dual-ADC/DMA timing contract is unchanged. Detect-all stores average inductance plus `Lq-Ld`, feeding the MTPA and saliency-observer backends already implemented in earlier batches.

Protection is now two-layered. VESC6 `l_slow_abs_current` at its existing wire byte can filter/debounce the configurable absolute-current threshold, while the physical 25-A board ceiling remains immediate. Configured VIN thresholds are debounced for four PWM samples, while wider absolute under/over-voltage envelopes remain first-sample hard faults. All hard thresholds and filter coefficients are precomputed; the 16-kHz ISR stays fixed-point.

## Batch 9 Part 1/3 — thermal / RPM / startup-current safety

Batch 9 Part 1 activates applicable current-VESC safety semantics without changing the proven 16-kHz hard FOC or TIM1/TIM8 + ADC1/ADC2 DMA current-sampling architecture. ADC1 rank 5 now samples the STM32 internal temperature channel after the current-sense DMA half-transfer point; its filtered result is explicitly treated as an MCU/board-temperature proxy, not a MOSFET junction measurement. Existing VESC6 thermal fields now drive task-side current derating, hard end-temperature faults, and `l_temp_accel_dec` acceleration-only derating while preserving braking authority.

The existing VESC6 `foc_start_curr_dec` / `foc_start_curr_dec_rpm` fields now reduce acceleration current near standstill. Optional current-VESC additional RPM faults (over/under/absolute overspeed) are implemented against a slow ERPM signal, but remain runtime-default-disabled and cannot be silently serialized because `l_additional_faults` is not part of the pinned VESC6 481-byte MCCONF image. All B2-B8 regressions plus the new Part-1 regression pass in the host environment; ARM/PlatformIO build and real-board thermal calibration remain mandatory.

## Batch 9 Part 2 — dedicated ADC3 / DMA2 DCLINK path

PC2/DCLINK is now owned by ADC3 channel 12 and is triggered by the same TIM8 TRGO that starts the dual ADC1/ADC2 current scan. ADC3 transfers through a two-halfword circular DMA2 Channel 5 buffer. The current-control entry remains DMA1 Channel 1 half-transfer after unchanged ADC1/ADC2 ranks 1..3.

At the unchanged 10.667-MHz ADC clock, the ADC3 28.5-cycle DCLINK conversion completes nominally at about 3.844 us after TIM8_TRGO, before the rank-3 current HT at about 5.063 us. The FOC ISR therefore consumes a same-trigger-frame Vbus sample under nominal timing instead of relying on the previous/in-progress ADC1 rank-4 value.

ADC3 HT/TC IRQs are disabled. DMA2 transfer errors hard-disable both bridges, and the two-entry circular CNDTR pattern is supervised in the FOC ISR; three consecutive stale observations request `MOTOR_FAULT_ADC_DMA` for both motors. Part-1 board-temperature sampling remains ADC1 rank 5 after the hard current HT boundary. See `BATCH9_PART2_REVISION_NOTES.md` and `tools/test_batch9_part2.py`.

## Batch 9 Part 3 — power-stage safety

Part 3 adds a reset-latched STM32 PVD brownout shutdown, an optional TIM1/TIM8 BKIN backend, and a VESC-style static low-side zero-vector brake backend. PVD is enabled by default because it is internal to the MCU. BKIN remains disabled until external fault wiring/polarity is physically validated. `foc_short_ls_on_zero_duty` is implemented but defaults OFF and cannot be fake-persisted in the pinned VESC-6.00 MCCONF. PVD/BKIN faults map to canonical `FAULT_CODE_MCU_UNDER_VOLTAGE` / `FAULT_CODE_BRK` and cannot be cleared without MCU reset while the power-stage latch is active. TIM1/TIM8 timing, ADC1/ADC2 current acquisition, ADC3/DMA2 Vbus acquisition, 16-kHz fixed-point FOC and VESC6 481/493-byte wire ABI remain unchanged.

## Batch 10 Part 1 — VESC7 fast FW / MTPA composition / encoder slip

Batch 10 Part 1 updates the highest-value VESC7 control semantics without changing the STM32F103 hardware timing layer. MTPA now reduces Iq so it rotates rather than enlarges the current vector, and field weakening is composed with MTPA by selecting the larger-magnitude Id request instead of adding both negative Id values. Automatic FW ramp/backoff is moved into the 16-kHz fixed-point current path; configuration math remains task-side, duty is low-pass filtered, manual override is direct, and an atomic 32-bit fractional accumulator preserves slow-ramp resolution efficiently on Cortex-M3. Because this board intentionally constrains PWM to a 10..90% current-sampling window, FW duty thresholds are normalized to that usable modulation span rather than changing the proven ADC timing.

LEFT encoder/ABI operation also gains optional VESC-style encoder-slip protection. It compares a coherent, PWM-delay-compensated observer phase with encoder phase only above 110% of open-loop ERPM and faults after >15 electrical degrees persists for 500 ms. Additional-fault bit 0 remains runtime-only on the pinned VESC6 ABI; non-zero newer additional-fault settings are not fake-persisted.

The excluded ecosystems remain untouched: CAN hardware, IMU, BMS/bm_if, NRF, LEDPWM, COM_USB, QML UI, lispif, NTC runtime and LZO.

## Batch 10 Part 2 — adaptive/phase/input-current refinements

Part 2 adds a 1-kHz VESC-style online motor-resistance estimate (diagnostic only; it does not silently alter configured observer R), an integer Hall electrical-angle rate limiter before Hall-to-observer blending, a physical-DC-current-backed pre-limit starting at 90% of the input-current ceiling with 0.005 filtering, and a runtime FOC speed phase-source selector between corrected/control and compensated observer phase. The selected source drives PLL/FAST/FASTER while a corrected fast estimate remains available independently. V33 removes the old empty `offset_track_isr()` and its unused state; online undriven current-offset tracking is explicitly not claimed until a real, configurable, bench-tested backend is implemented. No ADC/PWM timing or excluded subsystem is changed, and VESC6 481/493-byte wire compatibility remains pinned.

## VESC Tool dual-controller basic discovery hardening

The single USART3 management link now presents the dual-motor MCU to VESC Tool
using the same user-facing model as an upstream dual-motor controller:

- the directly connected logical controller is Motor-1 / controller ID 1;
- `COMM_PING_CAN` on the root connection advertises exactly one additional ID: 2;
- VESC Tool can therefore list Motor-2 / ID 2 during its normal CAN/device scan;
- requests sent by VESC Tool to ID 2 through `COMM_FORWARD_CAN` execute locally
  against the right-motor runtime and replies remain ordinary inner command
  replies (not nested `COMM_FORWARD_CAN` packets);
- Motor-2 `COMM_FW_VERSION` uses the same firmware identity but a distinct UUID
  byte, matching upstream dual-motor backup/restore semantics;
- per-motor `COMM_GET_VALUES` reports controller ID 1 or 2 respectively;
- `COMM_GET_VALUES_SETUP` reports `num_vescs = 2`;
- `COMM_GET_MCCONF`/`COMM_SET_MCCONF` operate on the selected motor's independent
  motor configuration;
- Motor-2 `COMM_GET_APPCONF` exposes ID 2 while the single internal application
  configuration remains owned by ID 1. A forwarded Motor-2 APPCONF write is only
  accepted when the public wire image still requests fixed ID 2, preventing a
  false ACK for unsupported ID remapping.

This does not add a physical CAN driver or CAN PHY. `COMM_PING_CAN` is only the
standard VESC Tool discovery surface for the second local motor. IDs other than 2
remain unsupported for forwarding.

## Batch 11 Part 1 — VESC Tool core readiness / dual Detect-All / calibration / telemetry

Part 1 turns VESC Tool `detect_can=true` into an atomic local dual-motor Detect-All transaction: the direct LEFT/ID1 controller and local forwarded RIGHT/ID2 are freshly current-calibrated, electrically identified and sensor-calibrated in sequence, then both MCCONFs are committed in one transactional flash write. Any failure restores both runtimes to the last committed configurations. LEFT keeps selectable Hall/ABI behavior; RIGHT remains Hall-only at both configuration and hard-FOC layers. Standard duty/current/brake/RPM/position/handbrake/current-relative command routing and VESC6 GET_VALUES/SELECTIVE/SETUP telemetry are regression-locked for M1/M2. `COMM_SET_APPCONF_NO_STORE` is supported. Fault indication now derives both LED and buzzer pulse groups from the same canonical VESC fault number, including encoder slip, while retaining the non-blocking startup melody. PWM polarity remains high-side active HIGH / low-side complementary active LOW, matching the EFeru stock hoverboard FOC setup. No bootloader, excluded subsystem, ADC/PWM retiming or wire-ABI migration is introduced.

## Batch 11 Part 2 — command completeness / integrity / cleanup

Batch 11 Part 2 preserves the Part-1 dual Detect-All, calibration and telemetry architecture while adding practical VESC Tool/runtime completeness without a bootloader or PWM/ADC retiming. `COMM_SET_BATTERY_CUT` / `COMM_GET_BATTERY_CUT`, `COMM_FW_INFO`, and safe shared-domain `COMM_SHUTDOWN` are implemented. Direct battery-cut forward-all applies both local motor configurations atomically; the blocking worker uses static BSS wire scratch rather than large nested stack images. Shutdown checks both motor speeds, clears both MOEs before PA5 power hold is released, and remains input-latched if external power keeps the MCU alive.

The transactional Flash configuration record now has a 1-Hz idle-only CRC/sequence integrity scrub. Corruption maps to canonical `FAULT_CODE_FLASH_CORRUPTION`, hard-disables both bridges, and cannot be hidden by fault clear until a valid replacement configuration has repaired the integrity state. Custom diagnostic revision 9 and terminal `integrity` expose the resulting counters and power-latch state.

The simple PA4 buzzer backend is made explicit: timed tones are non-blocking; fake arbitrary sample-audio state is removed and unsupported sample playback reports unsupported. VESC6 `cc_startup_boost_duty`, `cc_gain`, and `cc_ramp_step_max` remain byte-preserved but immutable because upstream uses them in trapezoidal BLDC `mcpwm.c`, not in FOC. `cc_min_current` remains the real FOC control/release field.

Firmware name is `vesc-f103-hoverboard-v33-vesc-layout`; VESC wire ABI remains 6.00 / 481-byte MCCONF / 493-byte APPCONF. Excluded HFI/CAN-hardware/IMU/BMS/bm_if/NRF/LEDPWM/COM_USB/QML/lispif/NTC/LZO subsystems remain absent.

## F103 48-KiB RAM linker fix

The first complete ARM link exposed a real 6184-byte SRAM overflow that earlier compile failures had hidden. The target remains the physical STM32F103RCT6 48-KiB SRAM; the linker region is not enlarged. The debug/experiment capture buffer is reduced from 256 to 64 30-byte samples (saving 5760 bytes), and heap_4 is right-sized from 22 KiB to 18 KiB (saving 4096 bytes). A 2-KiB runtime free-heap reserve is required before the controller is advertised motor-ready. Standard VESC telemetry/control, dual Detect-All, FOC, sensor/calibration and persistence functionality are unchanged.
