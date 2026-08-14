# V7 — Handshake-First VESC UART

Versi ini adalah revisi V6 untuk kasus VESC Tool masih gagal membaca firmware version. Baca `V7_HANDSHAKE_SCREENING.md` terlebih dahulu.

Tes pertama setelah flash:

```bash
python3 debug_vesc_f103.py handshake --port /dev/ttyUSB0 --baud 115200 --attempts 5
```

Target wire request adalah `02 01 00 00 00 03`. Firmware harus membalas `COMM_FW_VERSION` valid sebelum pengujian motor.

---

# vesc_f103_dual_foc_rtos2_v6

Port STM32F103RCT6 + STM32Cube HAL + CMSIS-RTOS2/FreeRTOS yang mempertahankan fast FOC di ADC/DMA ISR dan memetakan semantik komunikasi/worker VESC upstream ke hardware F103 ini.

## Topologi motor

```text
VESC Tool / USB-UART
        |
        v
USART3 PB10/PB11 @ 115200 8N1
        |
        +-- command normal ------------------------> MOTOR LEFT / local / ID 1
        |
        `-- COMM_FORWARD_CAN + target 2 + inner --> MOTOR RIGHT / virtual CAN / ID 2
                                                   (tanpa CAN peripheral fisik)
```

Reply motor RIGHT dari `COMM_FORWARD_CAN` dikirim sebagai **inner VESC reply normal**, bukan dibungkus lagi dengan `COMM_FORWARD_CAN`.

## UART

- USART3 TX PB10, RX PB11.
- RX = RXNE interrupt -> software RX ring -> `packet_process_thread`.
- TX = software TX ring -> TXE interrupt -> DR; TC menandai transmisi fisik selesai.
- Tidak ada UART RX/TX DMA.
- DMA1 Channel1 tetap khusus ADC/FOC fast loop.

## Packet / VESC Tool

V6 menangani payload sampai 512 byte, framing 2/3-byte length, CRC16, recovery setelah noise/bad frame, dan jalur request/reply berikut:

- `COMM_FW_VERSION`
- `COMM_GET_VALUES`
- `COMM_GET_VALUES_SELECTIVE`
- `COMM_GET_VALUES_SETUP`
- `COMM_GET_VALUES_SETUP_SELECTIVE`
- `COMM_SET_DUTY`
- `COMM_SET_CURRENT`
- `COMM_SET_CURRENT_BRAKE`
- `COMM_SET_RPM`
- `COMM_SET_POS`
- `COMM_SET_HANDBRAKE`
- `COMM_SET_CURRENT_REL`
- `COMM_ALIVE`
- `COMM_SET_DETECT` / rotor-position stream
- `COMM_GET_DECODED_ADC`
- `COMM_CUSTOM_APP_DATA` callback semantics
- `COMM_SAMPLE_PRINT`
- `COMM_DETECT_ENCODER`
- `COMM_DETECT_HALL_FOC`
- `COMM_FORWARD_CAN`
- `COMM_PING_CAN`

## Thread CMSIS-RTOS2

```text
adc_thread                 application ADC worker (applications/app_adc semantic)
packet_process_thread      drain RX ring + framing + non-blocking command dispatch
blocking_thread            Hall/encoder detect, ping CAN, blocking jobs
current_cal_thread         F103-specific startup current-zero supervisor

timer_thread               motor/interface housekeeping
pid_thread                 speed/position outer-loop controller
rpm_thread                 F103 Hall/ABI speed estimator service
sample_send_thread         sample packet sender
fault_stop_thread          deferred software fault handling
stat_thread                telemetry/stat accumulation
periodic_thread            telemetry + rotor-position periodic work
led_thread                 board status LED
timeout_thread             global command timeout for both motor contexts
```

Fast current FOC **bukan thread**. Ia tetap di `DMA1_Channel1_IRQHandler()`:

```text
ADC dual simultaneous -> DMA HT IRQ -> Clarke -> Park -> Id/Iq PI
-> Vd/Vq limit -> inverse Park -> SVPWM -> TIM8 LEFT + TIM1 RIGHT
```

## Persistence / EEPROM-equivalent

STM32F103RC tidak memiliki data EEPROM. V6 menggunakan page flash terakhir 2 KiB (`0x0803F800`) sebagai emulasi persistent configuration:

- firmware build dibatasi 260096 byte agar tidak masuk ke page config;
- record mempunyai version, sequence, CRC32;
- multiple append slots untuk wear-level;
- magic/commit ditulis paling akhir;
- record lama tetap valid jika power loss terjadi sebelum record baru committed;
- saat page penuh, page di-erase dan log dimulai lagi;
- kedua motor dihentikan dan gate dimatikan sebelum write;
- DMA1 Channel1 IRQ sementara dimatikan selama flash erase/program;
- current PI fixed-point gains dihitung ulang saat config diload.

Yang disimpan: timeout, sensor mode, pole pairs, encoder CPR/invert/electrical offset, Hall table/angle, current Kp/Ki, speed Kp/Ki/Kd kedua motor.

Gunakan:

```bash
python3 debug_vesc_f103.py config-status --port /dev/ttyUSB0
python3 debug_vesc_f103.py config-save   --port /dev/ttyUSB0
```

## Virtual CAN RIGHT

Test:

```bash
python3 debug_vesc_f103.py can-scan --port /dev/ttyUSB0
```

Target:

```text
COMM_PING_CAN
  -> [COMM_PING_CAN, 2]

COMM_FORWARD_CAN, 2, COMM_FW_VERSION
  -> COMM_FW_VERSION reply milik MOTOR RIGHT

COMM_FORWARD_CAN, 2, COMM_GET_VALUES
  -> COMM_GET_VALUES dengan controller_id = 2
```

## Sample

`COMM_SAMPLE_PRINT` output memakai satu packet per sample dengan layout VESC-style:

```text
COMM_SAMPLE_PRINT
index int16
current0 float32_auto
current1 float32_auto
current2 float32_auto
phase1 float32_auto
phase2 float32_auto
phase3 float32_auto
vzero float32_auto
current_fir float32_auto
switching_frequency float32_auto
status uint8
phase uint8
index int32
```

Test:

```bash
python3 debug_vesc_f103.py sample --motor 0 --count 64 --decimation 8 --port /dev/ttyUSB0
python3 debug_vesc_f103.py sample --motor 1 --count 64 --decimation 8 --port /dev/ttyUSB0
```

## Batas yang sengaja tidak diklaim 100% upstream

V6 **tidak mengklaim seluruh firmware `vedderb/bldc` telah dipindahkan byte-for-byte**. Hal berikut masih sengaja dibatasi:

1. Full `mc_configuration` / `app_configuration` confgenerator serialization untuk tombol Read/Write Motor/App Configuration VESC Tool belum dipindahkan. `COMM_SET/GET_MCCONF` dan `COMM_SET/GET_APPCONF` tidak di-ACK palsu. Persistence yang benar-benar bekerja saat ini adalah config-store V6 di atas.
2. R/L/flux-linkage Detect All belum memberikan measurement palsu; command tersebut mengembalikan failure/sentinel.
3. `COMM_SAMPLE_PRINT` packet layout sudah standar, tetapi trigger/fault prebuffer lengkap belum ada. Mode capture V6 saat ini immediate. PCB juga hanya mempunyai dua phase-current channels, sehingga channel ketiga dihitung/infer atau diset 0 pada raw sample.
4. HFI/sensorless-estimator upstream tidak dipindahkan; port ini memang berfokus Hall + ABI encoder sesuai hardware proyek.
5. Hardware IWDG/thread-monitor penuh upstream belum dipindahkan; global VESC-style command timeout sudah ada.
6. `rpm_thread` di V6 adalah service estimator Hall/ABI F103. Ia bukan klaim port literal thread BLDC six-step `mcpwm.c` upstream.

Modul yang memang dikeluarkan sesuai permintaan: physical CAN hardware, IMU, BMS, bm_if, NRF, LEDPWM, COMUSB, QMLUI, LispIF, NTC temperature subsystem, LZO.
