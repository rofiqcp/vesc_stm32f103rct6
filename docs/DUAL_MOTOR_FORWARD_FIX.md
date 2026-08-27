# VESC Tool local dual-motor forwarding fix

This STM32F103 board exposes the left bridge as the directly connected VESC and
exposes the right bridge as local controller ID 2. There is no physical CAN PHY
in this build. ID 2 therefore follows the same local-dual-motor semantics used
by upstream VESC when `HW_HAS_DUAL_MOTORS` is enabled.

## VESC Tool request path

When VESC Tool selects controller `(2)`, `COMM_FW_VERSION` is sent as:

```
COMM_FORWARD_CAN, 2, COMM_FW_VERSION
```

The packet task must:

1. Select VESC motor thread 2.
2. Process the inner command recursively on the same reply transport.
3. Restore the previous thread motor selection.

Replies remain ordinary inner VESC replies; they are not wrapped in
`COMM_FORWARD_CAN`.

## Firmware identity

Both local motor contexts report the same hardware name (`MOTOR_LEFT` in this
port, retained for compatibility with the already-proven local connection).
The second local motor uses the same 96-bit STM32 UUID with the last byte
incremented by one **only while motor thread 2 is selected**, matching upstream
VESC dual-motor backup/identity behavior.

The right motor APPCONF view still reports controller ID 2, while the persisted
single application configuration retains the base controller ID 1.

## Hardware test

After flashing, first verify the exact forwarded handshake without VESC Tool:

```bash
python3 tools/debug.py motor2-forward --port /dev/ttyUSB0 --baud 115200
```

Then verify both contexts and both configuration read paths:

```bash
python3 tools/debug.py vesc-tool-dual-basic --port /dev/ttyUSB0 --baud 115200
```

Only after both pass should VESC Tool be used to select motor `(2)`.
