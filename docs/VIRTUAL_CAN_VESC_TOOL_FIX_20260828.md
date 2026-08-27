# VESC Tool dual-motor virtual-CAN fix — 2026-08-28

This revision keeps the proven USART3/FreeRTOS boot path and changes only the
remaining dual-motor/VESC Tool compatibility issues.

## Fixed

1. `reply_fw_version` no longer has an unused `motor_id_t id` parameter.
2. `COMM_GET_VALUES_SELECTIVE` now matches upstream VESC:
   - bit 19: Vd (`float32`, 1e3)
   - bit 20: Vq (`float32`, 1e3)
   - bit 21: status byte
3. `COMM_FORWARD_CAN` mirrors `HW_HAS_DUAL_MOTORS` upstream behavior for the
   local second motor: select motor-thread 2, recursively process the inner
   command, then always restore motor-thread 1.
4. Every unwrapped UART packet explicitly starts and ends in motor-thread 1.
   This prevents a forwarded request from leaking motor-2 identity into the
   next direct FW/config request (`Unknown (local)` / intermittent local ID).
5. Motor-2 `SET_APPCONF` ignores the forwarded public CAN ID before applying
   the shared application configuration, matching upstream dual-motor VESC.
6. Healthy-idle LED heartbeat now toggles every 500 ms. Calibration/detection/
   running burst modes are reset cleanly on every mode transition.

## Hardware test

After clean build/upload:

```bash
python3 tools/debug.py handshake --port /dev/ttyUSB0 --baud 115200 --attempts 5 --timeout 0.7
python3 tools/debug.py motor2-forward --port /dev/ttyUSB0 --baud 115200
python3 tools/debug.py vesc-tool-dual-basic --port /dev/ttyUSB0 --baud 115200
```

`vesc-tool-dual-basic` now also checks selective-value bits 19/20/21 and
repeated local <-> motor-2 route switching to catch context leakage before
opening VESC Tool.
