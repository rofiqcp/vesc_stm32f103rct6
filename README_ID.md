# V14 — Driven Current Offset Calibration Recovery

V14 keeps the **known-working V8 VESC Tool UART-DMA transport frozen** and keeps the V11 STM32F103 timing path (`TIM1/TIM8 -> TIM2 CC2 -> ADC1+ADC2 dual regular simultaneous -> DMA1_CH1 HT -> fixed-point FOC`). The main V14 change is current-sense calibration: final FOC offsets are measured under the same switching condition used by active FOC instead of being learned only while MOE is off.

## Why V14 exists

The V13 Hall-detect trace showed calibration valid and near-zero current before detection, then `ABS_OVER_CURRENT` was latched immediately after entering PREPARE. The later packet already showed MOE/PWM off and small current, so the actual fault was a short transient lost before telemetry could capture it.

V14 addresses that root cause with:

- **undriven baseline**: 4096 samples, used for rail/gross-safety diagnostics;
- **LEFT driven offset**: 50%/50%/50% zero-vector PWM, 64-event settle, 1000 samples at 1 kHz;
- **RIGHT driven offset**: same process independently;
- final phase/DC offsets used by the FOC PI are the **driven** averages;
- driven offsets are no longer slowly overwritten by off-state ADC readings;
- every normal PWM enable starts with 8 bounded FOC samples at exact 50% zero vector before current PI is allowed to act;
- first active-drive current fault is snapshotted before MOE is cleared.

This is aligned with VESC's separation between driven and undriven offsets. The low-side-shunt VESC calibration variant uses all phases at 50% modulation (zero SVM amplitude) and averages 1000 samples.

## Frozen VESC Tool communication

- USART3 PB10 TX / PB11 RX, 115200 8N1
- RX DMA1_CH3 circular
- parser/commands in `packet_process_thread`
- TX DMA1_CH2 queue
- LEFT local; RIGHT virtual `COMM_FORWARD_CAN` ID 2

`audit/V8_TRANSPORT_FROZEN.sha256` protects the four proven transport files.

## Current mapping / fixed point

Stock hoverboard baseline remains:

- current polarity: `offset - ADC`
- 50 ADC counts/A -> 0.020 A/count
- LEFT two shunts: phase A + phase B, reconstruct C
- RIGHT two shunts: phase B + phase C, reconstruct A
- Q15 current base: 64 A
- current gain stored Q16.16; PI integrators Q31

Host arithmetic test verifies 50 counts -> 0.998046875 A and 1250 counts -> 24.998046875 A. The fixed-point quantization is far below one ADC-count step and there is no float trig/divide in the normal fast current loop.

## ADC/PWM timing retained

- SYSCLK 64 MHz
- TIM1/TIM8 center-aligned 16 kHz
- TIM2 CC2 sampling event
- ADC1 master + ADC2 regular-simultaneous slave
- DMA1_CH1 circular, half-transfer after the three current ranks
- one motor full FOC update per V0/V7 event, each motor 16 kHz

## First test after flashing

Keep the wheels off the ground and use a current-limited supply. Do **not** run Hall detect first.

```bash
python3 debug_vesc_f103.py calibrate \
  --port /dev/ttyUSB0 \
  --baud 115200 \
  --timeout 8 \
  --out v14_calibration.txt
```

The TXT now includes undriven vs driven offset deltas and the first active-drive current-fault snapshot. Only after calibration reports `valid=True` should Hall detect be tested at 0.5 A.

See `V14_DRIVEN_OFFSET_CALIBRATION.md` and `V14_TEST_RESULTS.txt`.
