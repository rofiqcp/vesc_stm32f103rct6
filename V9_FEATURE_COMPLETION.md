# V9 — Full Control/Config/Encoder Completion + Fixed-Point FOC

## Baseline yang tidak diubah

Transport VESC Tool yang sudah terbukti pada V8 **dibekukan byte-for-byte**:

- USART3 PB10/PB11, 115200 8N1
- RX DMA1 Channel3 circular 1024 byte
- TX DMA1 Channel2 queued (6 frame)
- framing/CRC `vesc_packet.c`
- VESC 6.00 handshake envelope
- LEFT local controller ID 1
- RIGHT virtual CAN controller ID 2 melalui `COMM_FORWARD_CAN`

Hash referensi ada di `audit/V8_TRANSPORT_FROZEN.sha256`; `tests/host/audit_v9_features.sh` akan gagal bila empat file transport berubah.

## 1. Standard control commands

Command wire/scaling mengikuti VESC 6.00:

- `COMM_SET_DUTY`: int32 / 100000
- `COMM_SET_CURRENT`: int32 / 1000 A
- `COMM_SET_CURRENT_BRAKE`: int32 / 1000 A
- `COMM_SET_RPM`: int32 eRPM
- `COMM_SET_POS`: int32 / 1,000,000 degree
- `COMM_SET_HANDBRAKE`: int32 / 1000 A
- `COMM_SET_CURRENT_REL`: int32 / 100000

Pipeline runtime:

```text
VESC command
  -> motor_set_*()
  -> control mode + setpoint
  -> 1 kHz outer loop when needed
       DUTY -> duty PI -> Iq
       RPM  -> speed PID -> Iq
       POS  -> position PID -> Iq
       CURRENT -> Iq direct
  -> Id/Iq target snapshot
  -> 16 kHz ADC/DMA fixed-point current FOC
  -> inverse Park + SVM
  -> TIM8 LEFT / TIM1 RIGHT CCR
```

Every active control command refreshes the VESC command timeout.

## 2. LEFT encoder AB complete path

Hardware remains exactly the requested board mapping:

- LEFT Hall: PB5/PB6/PB7
- LEFT Encoder AB: PB6/PB7 = TIM4_CH1/TIM4_CH2, TI12 quadrature
- RIGHT: Hall PC10/PC11/PC12 only

LEFT Hall and Encoder AB are mutually exclusive. Reconfiguration always stops PWM first.

Encoder state includes:

- CPR/counts per mechanical revolution
- extended TIM4 revolution count
- direction and wrap
- first-speed-sample guard after reset/reconfigure
- encoder inversion
- electrical offset
- **fractional `foc_encoder_ratio`** preserved as float for configuration and converted to Q16.16 for the ISR
- physical motor pole-pairs stored independently from encoder electrical ratio
- session electrical zero and mechanical position zero
- synchronized/motion-proved flags

Because there is no index/Z, encoder FOC torque is inhibited until a controlled alignment for the current boot has succeeded. `COMM_DETECT_ENCODER` performs a low-current phase sweep, determines direction/ratio, returns to electrical phase zero, captures `session_zero_count`, and then marks the encoder synchronized. This avoids pretending that an incremental AB encoder has absolute electrical position after a power cycle.

## 3. Motor Configuration — full VESC 6.00 wire image

`COMM_GET_MCCONF`, `COMM_GET_MCCONF_DEFAULT`, and `COMM_SET_MCCONF` use an exact **481-byte VESC 6.00 MCCONF wire image** for each motor.

Both LEFT and RIGHT keep their complete 481-byte image. Every byte is retained for VESC Tool read/write and flash persistence. Hardware-relevant fields are also applied to runtime, including:

- motor/input/absolute current limits supported by the board
- min/max eRPM
- max duty
- current-loop Kp/Ki
- Hall table
- speed PID Kp/Ki/Kd + D filter
- position PID Kp/Ki/Kd + D filter
- direction inversion
- physical pole count/pole pairs
- LEFT encoder CPR, ratio, offset, inversion and sensor mode

Hardware limits are canonicalized back into the active wire image. UI-only/unsupported subsystem bytes remain untouched so VESC Tool round-trips them without silently destroying the user's configuration.

## 4. App Configuration — full VESC 6.00 wire image

`COMM_GET_APPCONF`, `COMM_GET_APPCONF_DEFAULT`, and `COMM_SET_APPCONF` use an exact **493-byte VESC 6.00 APPCONF wire image**.

All 493 bytes are retained and persisted. Runtime applies only the app fields that actually exist in this reduced hardware port (for example timeout/brake/app-ADC selection). The proven USART3 DMA transport cannot be disabled or re-bauded by APPCONF; those bytes still round-trip but V8 communication remains intentionally immutable.

The RIGHT node is virtual on the same MCU, so controller ID 2 is synthesized on a RIGHT GET. A RIGHT SET applies the same controller-level APPCONF while normalizing the stored physical controller ID to LEFT/local ID 1.

## 5. Flash-emulated EEPROM / persistent settings

STM32F103RCT6 has no true data EEPROM. V9 therefore stores the complete VESC wire configurations in flash emulation:

```text
0x0803E000 .. 0x0803FFFF = final 8 KiB
4 pages x 2 KiB
```

Each record contains:

- version/magic
- generation sequence
- CRC32
- LEFT MCCONF 481 bytes
- RIGHT MCCONF 481 bytes
- APPCONF 493 bytes

Write transaction:

1. stop both motors and clear MOE
2. prepare next record
3. erase the next rotation page
4. write header/payload/CRC except magic
5. write magic last as commit marker
6. read back and validate CRC/sequence
7. keep prior valid page available if the new write is interrupted

`stm32f103rc_v9.ld` limits application FLASH to 248 KiB and `platformio.ini` also limits upload size to 253952 bytes, so the last 8 KiB cannot be occupied by a valid application image.

## 6. Fixed-point FOC optimized for Cortex-M3

The hard current loop remains in the PWM-synchronous ADC DMA ISR. Slow configuration, telemetry and outer PID remain in RTOS tasks.

Fast path representations:

- phase currents: Q15
- Id/Iq: Q15
- Vd/Vq: Q15
- duty: Q15
- current Kp and Ki*dt: Q16.16
- current PI integral: **Q31 high-resolution accumulator** (Q15 plus 16 fractional bits)
- encoder electrical ratio: Q16.16
- encoder phase-per-count: fixed point
- inverse Vbus: cached Q30 reciprocal
- sine/cosine: 1024-point Q15 LUT in FLASH + 6-bit linear interpolation
- Clarke/Park/inverse Park: integer fixed point
- SVM: integer fixed point

The Q31 integrator is important at 16 kHz: small Ki increments are accumulated instead of being truncated to zero every 62.5 us.

The sine LUT is `const`, therefore it consumes FLASH instead of ~2 KiB SRAM. Exhaustive host testing of all 65536 electrical phase values limits sine/cosine interpolation error to <= 3 Q15 counts against the mathematical reference.

`-ffast-math` was removed. The hard ISR obtains speed from fixed-point arithmetic instead of depending on unsafe floating-point compiler assumptions; slow 1 kHz code can still use float where it improves clarity/config compatibility.

## 7. Safety/liveness preserved

V8 liveness relief remains in place so an over-budget dual-motor FOC update cannot permanently starve the VESC communication task. The short guard now also uses each motor's configured absolute-current trip limit independently.

PWM can only enable when calibration is valid, no fault is active, Vbus is in range, and (for LEFT encoder mode) the no-index encoder has been synchronized in the current boot.

## 8. Tests

Run:

```bash
./tests/host/run_v9.sh
```

It checks:

- exhaustive fixed-point sine/cosine precision
- SVM basic bounds
- VESC buffer endian/scaled/float32-auto semantics
- 481/493-byte config layouts
- unsupported config byte preservation
- runtime canonicalization
- physical pole-pairs vs fractional encoder-ratio separation
- control-command routing/scaling
- TIM4 LEFT encoder implementation
- four-page flash persistence architecture
- Q16.16/Q31 fixed-point FOC features
- V8 transport hash freeze
- Python framing/FW/virtual-CAN/sample self-test

Also run the V8 regression:

```bash
./tests/host/run_v8.sh
sha256sum -c audit/V8_TRANSPORT_FROZEN.sha256
```

## 9. Deliberate limitations

- Physical CAN hardware remains excluded; RIGHT is virtual CAN only.
- RIGHT encoder remains unsupported because the board only supplies RIGHT Hall inputs.
- Full physical R/L/flux-linkage detection is still not fabricated. Unsupported detect commands return a safe failure/unsupported result until a real measurement routine is validated.
- Incremental LEFT AB cannot retain absolute mechanical/electrical position through arbitrary power cycles without an external index/home/absolute reference. V9 uses controlled per-boot alignment instead.
- Host tests and source audits pass in this environment. ARM/PlatformIO cross-build cannot be claimed here because `pio`/`arm-none-eabi-gcc` are not installed in the runtime.
