# Deep-audit revision: VESC 6.00 dual motor / RT data / current protection

References used for this revision:

- `vedderb/bldc` tag 6.00: `COMM_FORWARD_CAN`, `COMM_GET_VALUES(_SELECTIVE)`, MCCONF and motor-thread semantics.
- `Koxx3/SmartESC_STM32_v2`: USART3 DMA transport with circular RX and CNDTR polling in a task.
- `EFeru/hoverboard-firmware-hack-FOC`: hoverboard power-stage baseline (16 kHz PWM, 50 ADC-count/A, 15 A motor, 17 A DC link).

## Dual-motor contract

- Motor Left = local UART VESC, motor thread 1, controller ID 1.
- Motor Right = in-MCU virtual node `(2)`, motor thread 2, controller ID 2.
- `COMM_FORWARD_CAN,2,<inner>` selects motor thread 2, executes the inner command, replies on the same UART, then restores motor thread 1.
- GET_VALUES uses coherent FOC state plus VESC-6.00 bit mapping: 2/3/4/5/19/20/21 = I_motor/I_in/Id/Iq/Vd/Vq/status.
- Large GET_VALUES and MCCONF scratch lives in BSS; packet-process gets explicit stack headroom for the one-level forwarded dispatch.

## Current protection

- Current limits are not raised to hide sensorless-start faults.
- Sensorless open-loop Q current is capped by live derated direction-specific limits and the 15 A board envelope.
- Software-only current trips use consecutive-sample qualification to reject switching/enable spikes; hardware break/power-stage protection remains immediate.

## Buzzer / LED

- TIM3 autonomously advances the ~3.21 s startup melody before FreeRTOS starts.
- Fault tones pre-empt the startup melody and have bounded duration.
- Healthy idle LED toggles every ~500 ms; calibration/detection/running use transition-safe 1/2/3-pulse cues.
- `tools/debug.py buzzer-test --mode status|melody|beep|stop` verifies the buzzer path while both bridges are stopped.

## Recommended post-flash checks

```bash
python3 tools/debug.py handshake --port /dev/ttyUSB0 --baud 115200 --attempts 5 --timeout 0.7
python3 tools/debug.py motor2-forward --port /dev/ttyUSB0 --baud 115200
python3 tools/debug.py rt-data-abi --port /dev/ttyUSB0 --baud 115200
python3 tools/debug.py vesc-tool-dual-basic --port /dev/ttyUSB0 --baud 115200
python3 tools/debug.py comm-diag --port /dev/ttyUSB0 --baud 115200
python3 tools/debug.py buzzer-test --port /dev/ttyUSB0 --baud 115200 --mode melody
```

If motor-2 GET_VALUES fails, `vesc-tool-dual-basic` automatically requests COMM_DIAG-v16. In GDB, `vesc_snapshot` includes UART/DMA, virtual-CAN trace, packet-task stack headroom, current-fault snapshot and buzzer state.
