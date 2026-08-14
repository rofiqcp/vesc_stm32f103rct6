# vesc_f103_dual_foc_rtos2_v5

Firmware dual-motor STM32F103RCT6 + STM32Cube HAL + CMSIS-RTOS2/FreeRTOS dengan fast-loop FOC di ISR ADC/DMA dan transport UART VESC yang dipisahkan dari loop motor.

## Perubahan utama V5

V5 mengubah transport USART3 agar mengikuti pola VESC UART interrupt/queue: RXNE untuk RX, TXE untuk TX, dan TC untuk akhir transmisi fisik. Jalur komunikasi sekarang dibuat:

```text
USB-UART
   |
USART3 PB11 RX
   |
RXNE interrupt
   |
software RX ring
   |
packet_process_thread
   |
VESC packet framing + CRC
   |
COMM_FW_VERSION / commands
   |
software TX ring
   |
TXE interrupt
   |
USART3 PB10 TX
   |
TC interrupt (physical TX complete)
```

UART VESC:

```text
USART3
TX = PB10
RX = PB11
115200 baud, 8N1
```

UART VESC **tidak memakai DMA RX maupun DMA TX**. DMA1 Channel1 tetap dipakai untuk ADC/FOC fast-loop.

Wiring USB-UART:

```text
USB-UART TX  -> PB11 (STM32 USART3 RX)
USB-UART RX  <- PB10 (STM32 USART3 TX)
USB-UART GND -> GND controller
```

TX dan RX **harus silang**.

## Thread V5

Semua fungsi thread yang diminta tersedia sebagai thread CMSIS-RTOS2 nyata. Nama fungsi dipertahankan sama dengan model VESC-style walaupun RTOS yang dipakai adalah CMSIS-RTOS2/FreeRTOS, bukan ChibiOS.

```text
adc_thread
packet_process_thread
blocking_thread
timer_thread
pid_thread
rpm_thread
sample_send_thread
fault_stop_thread
stat_thread
periodic_thread
led_thread
```

Pembagian pekerjaan:

```text
adc_thread             startup/current-zero calibration supervision
packet_process_thread  drain RX ring + VESC framing/CRC + non-blocking commands
blocking_thread        detection commands yang boleh lama

timer_thread           motor/FOC housekeeping 1 kHz
pid_thread             speed + position PID 1 kHz
rpm_thread             encoder/Hall RPM + position update 1 kHz
sample_send_thread     kirim captured FOC samples tanpa membebani ISR
fault_stop_thread      fault event + stop/recovery software
stat_thread            statistik 100 Hz
periodic_thread        rotor-position stream + telemetry snapshot 100 Hz
led_thread             LED status/fault
```

Fast FOC tetap **bukan RTOS thread**:

```text
PWM TIM1/TIM8
     |
ADC1+ADC2 simultaneous
     |
DMA1 Channel1 half-transfer IRQ
     |
FOC LEFT + RIGHT
  - offset current
  - Clarke
  - electrical rotor angle
  - Park
  - PI Id/Iq
  - Vd/Vq limiting
  - inverse Park
  - SVPWM
     |
TIM8 CCR1/2/3 LEFT
TIM1 CCR1/2/3 RIGHT
```

Tidak ada UART, `printf`, atau RTOS call di fast FOC ISR.

## Polaritas gate

```text
High-side UH/VH/WH : active HIGH
Low-side  UL/VL/WL : active LOW
```

TIM1/TIM8 menggunakan:

```c
OCPolarity   = TIM_OCPOLARITY_HIGH;
OCNPolarity  = TIM_OCNPOLARITY_LOW;
OCIdleState  = TIM_OCIDLESTATE_RESET;
OCNIdleState = TIM_OCNIDLESTATE_SET;
```

Dengan demikian kondisi OFF/dead-time adalah high-side LOW dan low-side HIGH.

## `COMM_FW_VERSION`

Reply V5 mengikuti layout paket firmware-version VESC 7.01:

```text
COMM_FW_VERSION
major = 7
minor = 1
HW name\0
12-byte STM32 UUID
pairing
FW test version
HW type
custom config count
phase filter flag
QML HW flag
QML APP flag
NRF flags
FW name\0
HW config CRC uint32
```

Handshake diproses langsung oleh `packet_process_thread`. Ia tidak menunggu current calibration, motor state, `periodic_thread`, atau telemetry.

## Blocking detection

Perintah berikut masuk queue depth-1 ke `blocking_thread` agar parser UART tetap hidup:

```text
COMM_DETECT_MOTOR_PARAM
COMM_DETECT_MOTOR_R_L
COMM_DETECT_MOTOR_FLUX_LINKAGE
COMM_DETECT_ENCODER
COMM_DETECT_HALL_FOC
COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP
COMM_DETECT_APPLY_ALL_FOC
```

Sensor Hall/encoder runtime detection yang sudah diimplementasikan tetap digunakan. **R/L/flux-linkage full VESC Detect All belum diimplementasikan sebagai measurement fisik final di V5**; command tersebut tidak dipalsukan dan mengembalikan failure/sentinel. Jangan menggunakan VESC Tool Detect All sebagai pengganti commissioning sensor V5 sampai modul R/L/flux ditambahkan.

## Runtime sensor LEFT

PB6/PB7 dipakai bersama oleh Hall kiri dan TIM4 encoder, sehingga V5 tetap melakukan runtime remux:

```text
LEFT HALL:
PB5/PB6/PB7 -> Hall input
TIM4 stop

LEFT ENCODER:
PB6/PB7 -> TIM4 CH1/CH2 encoder
Hall EXTI kiri disable
```

Mode dapat diubah saat runtime setelah motor dihentikan. Tidak ada lagi `#if LEFT_SENSOR_MODE` untuk jalur ISR Hall.

## Current zero calibration

Saat boot:

```text
PWM MOE OFF
 -> discard initial ADC samples
 -> collect 4096 zero-current samples
 -> calculate mean/noise/range
 -> set U/V/DC current offsets
 -> reset Id/Iq/Vd/Vq state
 -> only then motor can become ready
```

Zero calibration mengoreksi **offset**, bukan gain sensor. Nilai berikut tetap harus sesuai hardware PCB:

```c
LEFT_CURRENT_A_PER_COUNT
RIGHT_CURRENT_A_PER_COUNT
LEFT_DC_CURRENT_A_PER_COUNT
RIGHT_DC_CURRENT_A_PER_COUNT
DCLINK_V_PER_COUNT
```

Jika gain salah, nol bisa tepat tetapi skala Ampere/Volt tetap salah.

## Telemetry

`COMM_GET_VALUES` / selective menyediakan field VESC yang relevan:

```text
FET temp placeholder
motor temp placeholder
Imotor
Ibatt
Id
Iq
duty
ERPM
Vin
Ah / Ah charged
Wh / Wh charged
tachometer / tachometer_abs
fault
PID/rotor position
controller id
phase-temp placeholders
Vd
Vq
timeout status
```

Extended custom telemetry juga menyediakan raw/filtered current, current-zero offsets, Hall, encoder, sensor mode, rotor electrical angle, ISR cycles dan overrun count.

## Build

```bash
pio run -t clean
pio run
pio run -t upload
```

## Python debug

Install:

```bash
python3 -m pip install pyserial
```

Self-test parser:

```bash
python3 debug_vesc_f103.py --self-test
```

### Test pertama setelah upload: HANDSHAKE SAJA

Jangan hidupkan power motor dahulu.

```bash
python3 debug_vesc_f103.py handshake \
  --port /dev/ttyUSB0 \
  --baud 115200 \
  --timeout 1.5
```

Target:

```text
TX: ...
RX raw: ...
RX payload: 00 07 01 ...
PASS: framing + CRC + COMM_FW_VERSION reply valid
```

Kalau `RX raw: <no bytes>`:

```text
1. pastikan USB-UART TX -> PB11
2. pastikan USB-UART RX <- PB10
3. GND harus common
4. baud 115200 8N1
5. cek 3.3 V logic level
6. cek firmware benar-benar boot
```

Kalau ada raw bytes tetapi CRC/framing gagal, fokus ke baud/noise/wiring.

Baud scan:

```bash
python3 debug_vesc_f103.py baud-scan \
  --port /dev/ttyUSB0
```

Diagnostic transport setelah handshake:

```bash
python3 debug_vesc_f103.py comm-diag \
  --port /dev/ttyUSB0 \
  --baud 115200
```

Metrik mencakup:

```text
rx_bytes
rx_ring_overruns
rx_frames_ok
tx_bytes
uart_errors
tx_frames
tx_ring_overruns
tx_complete_count
blocking_busy_drops
```

Firmware info:

```bash
python3 debug_vesc_f103.py info --port /dev/ttyUSB0
```

Passive test:

```bash
python3 debug_vesc_f103.py test-all --port /dev/ttyUSB0
```

Current calibration:

```bash
python3 debug_vesc_f103.py calibrate --port /dev/ttyUSB0
```

Active commissioning baru setelah wiring/current scaling/sensor aman:

```bash
python3 debug_vesc_f103.py full-test \
  --port /dev/ttyUSB0 \
  --yes
```

## Urutan diagnosis VESC Tool

```text
Python handshake FAIL + 0 raw byte
    -> masalah fisik UART / pin / baud / boot

Python handshake raw ada, CRC FAIL
    -> framing / baud / noise

Python handshake PASS
    -> UART transport + COMM_FW_VERSION sudah bekerja

Python handshake PASS tetapi halaman konfigurasi VESC Tool tidak lengkap
    -> transport bukan masalah lagi; periksa compatibility command/config
       seperti MCCONF/APPCONF yang belum seluruhnya diimplementasikan.
```

