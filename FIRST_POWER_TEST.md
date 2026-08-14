# V9 first controlled motor test

Communication is the proven V8 path. Do not start with high current.

1. Flash V9, power the board current-limited if possible.
2. Confirm PB2 heartbeat and the startup buzzer melody.
3. Confirm VESC Tool connects at 115200 and reads FW 6.00 / `HOVERBOARD_DUAL_FOC`.
4. With wheels lifted, confirm RT data Vbus and zero-current offsets look plausible.
5. For Hall mode, rotate each wheel by hand and verify six Hall states before torque.
6. For LEFT Encoder AB, configure the correct **quadrature CPR**, then run encoder detection/alignment. Do not use encoder torque until sync succeeds.
7. Test command order at low limits: CURRENT -> DUTY -> RPM -> POS. Position is meaningful only with a valid position source; LEFT AB needs the current boot's alignment.
8. Read Motor Config back in VESC Tool after Write, power-cycle, reconnect, and confirm the saved fields return.
9. Repeat for App Config. UART transport intentionally remains fixed at USART3/115200 DMA even if an APPCONF field requests another UART mode.
10. Verify RIGHT through virtual CAN ID 2 separately.

Recommended debug command:

```bash
python3 debug_vesc_f103.py motor-test --port /dev/ttyUSB0 --motor 0 --mode current --value 1.0 --seconds 2 --yes
```

Then low duty/RPM only after current and sensor direction are verified. Always scope complementary PWM/dead-time and confirm current polarity before increasing limits.
