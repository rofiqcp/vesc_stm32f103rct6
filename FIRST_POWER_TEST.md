# First Test V5

## A. Uji komunikasi tanpa power motor

1. Flash firmware:

```bash
pio run -t clean
pio run
pio run -t upload
```

2. Wiring serial:

```text
USB-UART TX -> PB11 / USART3_RX
USB-UART RX <- PB10 / USART3_TX
GND         -> GND
baud        = 115200 8N1
```

3. Jalankan:

```bash
python3 -m pip install pyserial
python3 debug_vesc_f103.py --self-test
python3 debug_vesc_f103.py handshake --port /dev/ttyUSB0 --baud 115200 --timeout 1.5
```

Jangan lanjut sebelum handshake memberi:

```text
PASS: framing + CRC + COMM_FW_VERSION reply valid
```

4. Setelah PASS:

```bash
python3 debug_vesc_f103.py comm-diag --port /dev/ttyUSB0 --baud 115200
python3 debug_vesc_f103.py info --port /dev/ttyUSB0 --baud 115200
python3 debug_vesc_f103.py test-all --port /dev/ttyUSB0 --baud 115200
```

## B. Baru uji ADC/current zero

Power stage tetap tidak diberi command motor.

```bash
python3 debug_vesc_f103.py calibrate --port /dev/ttyUSB0 --baud 115200
```

Pastikan calibration done/valid dan Id/Iq/Imotor dekat nol.

## C. Sensor

```bash
python3 debug_vesc_f103.py sensor-info --port /dev/ttyUSB0
```

Auto-detect hanya ketika roda bebas dan area aman.

## D. Motor aktif

Pastikan terlebih dahulu:

- current sensor gain benar,
- DC-link scale benar,
- phase U/V/W benar,
- high-side active HIGH,
- low-side active LOW,
- deadtime benar,
- current limit rendah,
- roda terangkat.

Kemudian gunakan `full-test --yes` dengan current kecil.

