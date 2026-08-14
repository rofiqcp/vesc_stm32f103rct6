# V12 Calibration + TXT Debug Revision

## Why V11 failed on the reported board

Observed V11 result:

- calibration count reached 4096/4096
- ADC ISR count was already in the millions
- TIM1, TIM8 and TIM2 were running
- ADC1 and ADC2 were enabled
- DMA1 Channel1 was enabled
- offset means were approximately LU=2782, LV=2712, LDC=1946, RU=2788, RV=2770, RDC=2770

That proves the timer -> ADC dual-mode -> DMA1_CH1 -> FOC ISR chain is alive. The V11 failure came from the calibration acceptance policy, but V11 did not expose which channel failed spread/stddev validation.

V11 used a hard `stddev <= 12 ADC counts` and `peak-to-peak <= 200 counts` gate. This is stricter than the upstream VESC DC-cal flow, which primarily averages the measured current offsets. V12 therefore separates diagnostics into WARNING and HARD FAIL.

## V12 calibration policy

Mean offset is always the measured average of 4096 PWM-synchronous samples.

Hard fail only when a channel is clearly unsafe:

- mean outside 128..3967 ADC counts
- peak-to-peak spread > 800 counts
- population stddev > 80 counts

Diagnostic warning only:

- peak-to-peak spread > 160 counts
- population stddev > 16 counts

A warning does not inhibit PWM. A hard failure still raises `CURRENT_OFFSET` and keeps PWM disabled.

Channel bit order for all masks:

- bit 0 LEFT_U
- bit 1 LEFT_V
- bit 2 LEFT_DC
- bit 3 RIGHT_U
- bit 4 RIGHT_V
- bit 5 RIGHT_DC

## Important recovery fix

V11 left `MOTOR_FAULT_CURRENT_OFFSET` latched even after a later recalibration succeeded. V12 automatically clears **only** this recoverable calibration fault after a valid recalibration. Other electrical/sensor faults are never auto-cleared by this path.

## Detailed TXT output

Use the full passive diagnostic after flashing V12:

```bash
python3 debug_vesc_f103.py diagnose \
  --port /dev/ttyUSB0 \
  --baud 115200 \
  --timeout 8 \
  --out debug_v12.txt
```

The terminal only prints the TXT filename and result code. All details go to `debug_v12.txt`.

The report contains:

- FW_VERSION handshake identity
- USART3 RX/TX DMA communication counters
- calibration before and after a forced recalibration
- mean/min/max/peak-to-peak/stddev for all six current channels
- warning/failure bitmasks
- ADC ISR counter delta
- DMA CNDTR and raw six DMA words
- ADC1/ADC2 CR/SQR register snapshots
- TIM1/TIM8/TIM2 CR/ARR/CNT/BDTR/SMCR/CCR2 snapshots
- sensor state for both motors
- standard GET_VALUES and extended telemetry for LEFT and RIGHT
- final communication DMA counters

For calibration-only output:

```bash
python3 debug_vesc_f103.py calibrate \
  --port /dev/ttyUSB0 \
  --baud 115200 \
  --timeout 8 \
  --out calibration_v12.txt
```

## What stays unchanged

The V8-proven USART3 communication transport is still frozen byte-for-byte:

- USART3 PB11 RX
- DMA1 Channel3 circular RX
- packet parsing in task context
- TX queue
- DMA1 Channel2 TX

The V11 motor timing topology is also retained:

- STM32F103RCT6 at 64 MHz
- TIM1/TIM8 center-aligned PWM
- TIM2 CC2 sampling trigger/synchronization role
- ADC1 + ADC2 regular simultaneous
- DMA1 Channel1 circular
- current-loop FOC on DMA half-transfer
- no RTOS API in TIM2 or DMA1_CH1 hard ISR
