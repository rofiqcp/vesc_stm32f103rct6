# Batch 2 — Dead-time, Brake Zero-Cross, and VESC Duty Control

Baseline: `vesc_stm32f103rct6_batch1_voltage_telem.zip`

This batch is intentionally limited to three motor-control areas. It does **not**
include UART DMA, watchdog, advanced encoder/observer work, terminal/plot, or a
VESC ABI-version migration.

## 1. Fixed-point dead-time applied-voltage model

- `foc_dt_us` is now an active runtime MCCONF value (validated 0..5 us) rather
  than a stored-but-ignored setting.
- Task-side precalculation converts `foc_dt_us * foc_f_zv` to Q15. With the
  board default 0.75 us and 32 kHz VESC `f_zv`, this is Q15 ~= 786.
- The hard 16 kHz FOC path remains integer/fixed-point.
- Timer dead-time/CCR timing is not modified. The existing hoverboard advanced
  timer dead-time remains the hardware source of truth.
- The observer voltage is reconstructed from the final U/V/W SVM duties first,
  so Batch-1 sampling-window vector scaling is included in the model.
- VESC-style phase-current-sign dead-time correction is then applied to this
  reconstructed alpha/beta voltage before it is fed to the flux observer.

## 2. VESC-style duty-mode behavior

- Dedicated duty PI is used only when actual duty must be reduced by more than
  0.01, avoiding dangerous hard voltage truncation.
- For normal/ramp-up duty control, the requested duty becomes the dynamic FOC
  modulation ceiling and the controller requests the allowed motor-current
  limit with the command sign.
- The dynamic ceiling is converted task-side into the fixed-point voltage-circle
  coefficient used by the ISR.
- Duty commands smaller than `l_min_duty` release duty drive instead of being
  promoted to a non-zero minimum duty.
- The duty PI I-term is normalized and initialized from actual Iq when leaving
  PI down-ramp control, reducing hand-off discontinuity.

## 3. Brake zero-cross / direction-change guard

- Brake mode tracks the signs of fast electrical speed and Vq as well as the
  near-zero duty region.
- When a zero-cross transition is detected and requested braking current has not
  yet been reached, control is routed through the zero-duty duty-controller
  path instead of immediately applying opposite active torque.
- Once near zero modulation, the hard FOC ISR can emit a centered zero vector.
- A one-tick minimum hold at the 1 kHz motor-service rate corresponds to 16 hard
  FOC samples on this board, exceeding VESC's 10-sample minimum guard.
- The guard compares current against the **runtime-limited** brake request so it
  cannot wait forever for an unreachable current command.

## Files changed from Batch 1

- `src/confgenerator.c`
- `src/datatypes.h`
- `src/motor/foc_math.c`
- `src/motor/foc_math.h`
- `src/motor/mc_interface.c`
- `src/motor/mcpwm_foc.c`
- `tools/test_batch2.py` (new)

## Explicitly unchanged in Batch 2

- TIM1/TIM8 synchronized PWM architecture
- ADC1/ADC2 dual simultaneous sampling and DMA1 Channel 1
- LEFT Hall/encoder-AB and RIGHT Hall sensor policy
- USART3 transport implementation
- RTOS task topology
- VESC 6.00 MCCONF/APPCONF wire schema
- local Motor-2 `COMM_FORWARD_CAN` routing

## Verification performed

- VESC6 MCCONF/APPCONF layout: 481 / 493 bytes
- VESC6 config signatures unchanged
- fixed-point/no-float hard FOC invariant
- strict host compilation of the changed core C files
- packet framing and CRC recovery regression
- low-battery / regen / input-current limiter regression
- debug sampling regression
- Batch-1 SVM sampling-window regression
- dynamic modulation-ceiling regression
- applied-voltage reconstruction + dead-time fixed-point math regression
- Batch-2 source/state-machine regression (`python tools/test_batch2.py`)

## Hardware commissioning note

These are source-level/host-side validations. The final control behavior still
requires staged bench commissioning on the STM32F103 hoverboard PCB: current
sensor zero calibration, low-current spin, braking around zero speed, then regen
and loaded duty transitions while monitoring current and Vbus.
