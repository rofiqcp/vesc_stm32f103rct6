# V11 First Power Test — do this in order

Use a current-limited supply, keep wheels off the floor, and have a physical power cut reachable.

## 1. Flash / startup only

Expected:

- PB2 enters the normal heartbeat pattern.
- startup buzzer pattern sounds.
- no bridge should produce torque by itself.

## 2. Confirm V8 communication is still intact

```bash
python3 debug_vesc_f103.py handshake --port /dev/ttyUSB0 --baud 115200
```

Do not continue if handshake fails.

## 3. Prove TIM2 -> ADC -> DMA -> FOC ISR liveness

```bash
python3 debug_vesc_f103.py calibrate \
  --port /dev/ttyUSB0 --baud 115200 --timeout 5
```

Required observations:

- `tim1_running = true`
- `tim8_running = true`
- `tim2_running = true`
- `adc1_enabled = true`
- `adc2_enabled = true`
- `dma1_ch1_enabled = true`
- `adc_isr_count` increases between reads
- calibration `done = true`
- calibration `valid = true`
- calibration sample count reaches target

If `adc_isr_count` stays zero, stop. The fault is in the TIM2_CC2 / ADC multimode / DMA1_CH1 chain, not in Hall, encoder, PID or VESC commands.

## 4. Check telemetry before torque

```bash
python3 debug_vesc_f103.py status --motor 0 --port /dev/ttyUSB0 --baud 115200
python3 debug_vesc_f103.py status --motor 1 --port /dev/ttyUSB0 --baud 115200
```

Vbus must be plausible. Current values should remain near zero with PWM disabled.

## 5. Verify VESC Tool Motor Configuration

Open Motor Settings and read configuration. `GET_MCCONF` is intentionally available before motor boot finishes and returns the complete VESC6-size image. Do not start Auto Detect All yet.

## 6. Gate waveform check

Before a powered motor test, scope TIM8 LEFT and TIM1 RIGHT high/low outputs. Confirm approximately 16 kHz center-aligned complementary PWM, requested active polarity, dead-time, and no high/low overlap.

## 7. Lowest-risk motor command

LEFT, wheel lifted:

```bash
python3 debug_vesc_f103.py motor-test \
  --motor 0 --mode current --value 0.5 --seconds 1 \
  --port /dev/ttyUSB0 --baud 115200 --yes
```

Then only if current/phase is sane:

```bash
python3 debug_vesc_f103.py motor-test --motor 0 --mode duty --value 0.02 --seconds 1 --port /dev/ttyUSB0 --baud 115200 --yes
python3 debug_vesc_f103.py motor-test --motor 0 --mode rpm --value 300 --seconds 1 --port /dev/ttyUSB0 --baud 115200 --yes
```

## 8. Individual sensor detect

Hall LEFT:

```bash
python3 debug_vesc_f103.py sensor-detect --motor 0 --mode hall --port /dev/ttyUSB0 --baud 115200 --yes
```

Hall RIGHT:

```bash
python3 debug_vesc_f103.py sensor-detect --motor 1 --mode hall --port /dev/ttyUSB0 --baud 115200 --yes
```

LEFT Encoder AB only after the correct mechanical CPR is configured:

```bash
python3 debug_vesc_f103.py sensor-detect --motor 0 --mode encoder --port /dev/ttyUSB0 --baud 115200 --yes
```

PB6/PB7 are physically shared; never attempt LEFT Hall and TIM4 encoder simultaneously.

## 9. Auto Detect All limitation

V11 intentionally does not report successful full VESC Detect-All yet. The physical R/L/flux measurement stages still require board validation, so Detect-All returns a failure rather than storing fabricated parameters. Use individual Hall/encoder detect for this release.
