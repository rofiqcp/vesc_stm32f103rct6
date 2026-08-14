# V13 — VESC Hall Detection + Hoverboard Current-Feedback Fix

V13 is a safety/correctness release built on V12. It keeps the proven V8 VESC Tool UART transport and V11 TIM2/ADC/DMA topology unchanged. The purpose is to fix the current spike/growl seen during Hall detection and the false over-current fault when a stopped motor is rotated by hand.

## Root causes corrected

1. **Current polarity was reversed for this hoverboard analog front-end.**
   The proven EFeru board reads motor current as `offset - ADC`, not `ADC - offset`. V13 uses the board polarity for LEFT phase A/B, RIGHT phase B/C, and DC-current channels.
2. **RIGHT shunt phase identity was wrong.**
   PC4/PC5 are RIGHT phase B/C. V13 reconstructs phase A as `-(Ib + Ic)`.
3. **Hall GPIO polarity was wrong.**
   Stock hoverboard Hall inputs are active-low. V13 reads them active-low and uses NOPULL, matching the proven board implementation.
4. **Hall detect timing was too aggressive.**
   V13 follows the upstream VESC Hall-detect transaction: phase override, timeout disabled, 1-second Id ramp, three forward and three reverse sweeps at one electrical degree per step and 5 ms dwell, then eight-state accumulation/table evaluation.
5. **Clean detect failure is not a latched motor fault.**
   Detection returns failure without inventing a `SENSOR_DETECT` electrical fault. A real over-current/voltage/ADC fault remains latched with its original cause.
6. **Back-driven motor did not need a software drive-current trip.**
   The software ABS-over-current gate is evaluated only while firmware PWM/MOE is actively driving. Rotating the wheel by hand with PWM off no longer generates a false 3-beep ABS-over-current fault.
7. **Hall source of truth moved into the fast control path.**
   Hall GPIO is polled at each Hall-mode FOC update; EXTI is only an optional early timestamp hint. Same-sector reads do not reset edge timing. Invalid 000/111 and non-neighbor transitions are fast-counted and can fault only while torque is commanded.

## Hall-detect sequence

```
blocking command / custom diagnostic
        |
        +-- acquire single detect owner
        +-- preserve sensor mode, PI gains and timeout
        +-- force electrical phase = 0
        +-- Id ramp 0 -> requested current over 1000 ms
        +-- 3 x forward 0..359 deg, 5 ms/deg
        +-- 3 x reverse 360..0 deg, 5 ms/deg
        +-- accumulate sin/cos per raw Hall code 0..7
        +-- table entry if >30 observations, otherwise 255
        +-- success only when exactly two states are unseen (and on this board they must be 0 and 7)
        +-- Id/Iq -> 0, PWM off, restore timeout/gains/config
        +-- return result; standard VESC command does not auto-write configuration
```

## Safe diagnostic current

The custom TXT diagnostic accepts an explicit current. Start with 0.5 A:

```bash
python3 debug_vesc_f103.py sensor-detect \
  --port /dev/ttyUSB0 --baud 115200 \
  --motor 0 --mode hall --current 0.5 \
  --timeout 25 --yes --out hall_v13.txt
```

This diagnostic current is intentionally independent of the standard VESC Tool request value. Standard `COMM_DETECT_HALL_FOC` still uses the current supplied by VESC Tool, clamped to the firmware safety limit.

## What V13 does not claim yet

- Full `COMM_DETECT_APPLY_ALL_FOC` R/L/flux measurement is not considered validated on this power stage and therefore must not report success.
- LEFT incremental AB has no index/absolute mechanical reference; session synchronization remains required after each boot.
- Vbus scaling must be verified against a DMM before high-current/regenerative tests.
- Hardware scope/DWT measurements are still required before raising current limits.
