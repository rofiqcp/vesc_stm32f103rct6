# First Power / VESC Connection Test — V8

## 1. Build

```bash
pio run -t clean
pio run
```

## 2. Flash with motors unloaded/current-limited

```bash
pio run -t upload
```

## 3. Observe PB2 and PA4 before connecting VESC Tool

Expected sequence:

- PB2 LED turns ON very early after reset.
- After the RTOS scheduler starts, PA4 buzzer plays 3 ascending notes.
- PB2 becomes a 1 Hz heartbeat (500 ms ON / 500 ms OFF).

Interpretation:

- no PB2 at all: wrong firmware/flash/reset/power/pin issue before normal boot;
- PB2 solid but no melody/heartbeat: firmware reached early GPIO but failed before scheduler/status threads;
- melody + heartbeat: MCU clock, RTOS and status subsystem are running.

## 4. Raw handshake first

Close VESC Tool so the serial port is free, then run:

```bash
python3 debug_vesc_f103.py handshake \
  --port /dev/ttyUSB0 \
  --baud 115200 \
  --attempts 10
```

Exact host request is:

```text
02 01 00 00 00 03
```

Expected V8 response payload begins with:

```text
00 06 00 48 4F 56 45 52 42 4F 41 52 44 5F 44 55 41 4C 5F 46 4F 43 00 ...
```

When a valid VESC frame is decoded, PB2 changes temporarily to the double-flash link pattern. This creates a useful split diagnosis:

- heartbeat stays normal during handshake attempts: RX DMA / PB11 / port / baud path did not produce a valid frame;
- PB2 double-flashes but PC receives no frame: RX + framing already work; focus on TX queue / DMA1_CH2 / PB10 / USB-UART RX;
- script prints PASS: physical UART + packet + CRC + FW_VERSION round trip works.

## 5. Only after raw handshake PASS, open VESC Tool

Use the same serial port at 115200 baud.

## 6. Verify virtual CAN RIGHT

```bash
python3 debug_vesc_f103.py can-scan --port /dev/ttyUSB0
```

Expected: virtual node 2 and forwarded RIGHT `COMM_FW_VERSION` reply.

## 7. Passive motor checks

```bash
python3 debug_vesc_f103.py calibrate --port /dev/ttyUSB0
python3 debug_vesc_f103.py status --port /dev/ttyUSB0
```

Then sensor detect/save/sample can be tested only after passive values are sane.
