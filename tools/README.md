# Tools VESC STM32F103RCT6

Hanya ada dua program Python runtime:

- `vesc.py` — CLI teks untuk command standar VESC dan sekaligus PlatformIO pre-script FreeRTOS.
- `debug.py` — commissioning/debug lengkap untuk kedua motor.

Default koneksi: `/dev/ttyUSB0`, `115200` baud, USART3.

Instal dependensi:

```bash
python3 -m pip install -r tools/requirements.txt
```

## RT Data dan App Data

`stream-all` memakai `COMM_GET_VALUES_SELECTIVE` standar VESC pada 50 Hz dan
`COMM_GET_DECODED_ADC` standar VESC pada 20 Hz. Field RT cepat (I_motor, I_in, Id, Iq, duty, ERPM, Vin, fault, position,
controller ID, Vd, Vq, status) memakai wire format/scale `GET_VALUES_SELECTIVE`
VESC. Extended/raw diagnostics dipoll lebih lambat agar USART3 115200 tidak jenuh.

```bash
python3 tools/debug.py stream-all
python3 tools/debug.py stream-all --csv debug_50hz.csv
```

CLI teks GET_VALUES standar juga default 50 Hz:

```bash
python3 tools/vesc.py stream --motor left
python3 tools/vesc.py stream --motor right
```

## Hall commissioning LEFT + RIGHT

Roda harus bebas/terangkat. Perintah berikut melakukan Hall detect, apply,
persist, read-back MCCONF standar VESC, validasi status flash + tabel enam-state,
lalu test current **positif dan negatif** pada kedua motor. Motion check aktif
secara default; command gagal bila salah satu arah tidak melewati 50 ERPM:

```bash
python3 tools/debug.py hall-commission --motor both --yes
```

Default Hall detect current 1.5 A. Untuk test satu motor:

```bash
python3 tools/debug.py hall-commission --motor 0 --yes
python3 tools/debug.py hall-commission --motor 1 --yes
```

## Command standar VESC

```bash
python3 tools/vesc.py current 2.0 --motor left --seconds 3 --yes
python3 tools/vesc.py current 2.0 --motor right --seconds 3 --yes
python3 tools/vesc.py duty 0.05 --motor right --seconds 3 --yes
python3 tools/vesc.py rpm 300 --motor right --seconds 5 --yes
python3 tools/vesc.py position 30 --motor left --seconds 3 --yes
python3 tools/vesc.py shell
```

Motor RIGHT memakai forwarding lokal VESC controller ID 2 melalui
`COMM_FORWARD_CAN`, bukan protokol custom untuk command duty/current/RPM/position.

`SET_POS` pure sensorless dari zero-speed tidak memiliki referensi posisi rotor
absolut. Hall atau encoder harus menyediakan koordinat rotor/posisi yang sesuai,
atau observer sensorless harus sudah valid dari operasi bergerak.

## Acceptance rate nyata saat motor berjalan

Perintah ini mengirim `SET_RPM` 50 Hz, membaca RT Data 50 Hz, App ADC 20 Hz,
dan gagal bila rate aktual turun di bawah 45/18 Hz atau ada timeout/fault:

```bash
python3 tools/debug.py speed-test --motor 0 --erpm 300 --seconds 5 --yes --csv left_rate.csv
python3 tools/debug.py speed-test --motor 1 --erpm 300 --seconds 5 --yes --csv right_rate.csv
```

## Motor-2 / controller ID 2

Semua perintah berikut tetap melalui command standar VESC di dalam
`COMM_FORWARD_CAN` controller ID 2:

```bash
python3 tools/vesc.py duty 0.03 --motor right --seconds 3 --yes
python3 tools/vesc.py current 1.5 --motor right --seconds 3 --yes
python3 tools/vesc.py brake 1.0 --motor right --seconds 2 --yes
python3 tools/vesc.py handbrake 1.0 --motor right --seconds 2 --yes
python3 tools/vesc.py rpm 300 --motor right --seconds 5 --yes
python3 tools/vesc.py position 30 --motor right --seconds 3 --yes
```

Pada Hall, posisi mekanik bersifat incremental/coarse dari edge Hall; lakukan
Hall commissioning sebelum menguji position.

## Catatan Run25: telemetry, APP ADC, dan timing

- Default port: `/dev/ttyUSB0`, baud `115200`, transport USART3 PB10/PB11.
- `debug.py stream-all` mempoll `COMM_GET_VALUES_SELECTIVE` standar VESC pada 50 Hz dan `COMM_GET_DECODED_ADC` pada 20 Hz.
- `COMM_GET_DECODED_ADC` selalu mendapat balasan 17-byte standar, termasuk saat idle, kalibrasi, atau fault. Sebelum frame PA2/PA3 pertama tersedia nilainya nol; sesudah itu data berasal dari snapshot ADC fisik terbaru.
- APP yang memang tersedia pada board ini: `APP_NONE`, `APP_ADC`, `APP_UART`, `APP_ADC_UART`.
- Mode ADC analog tanpa tombol yang didukung: CURRENT, CURRENT_REV_CENTER, CURRENT_NOREV_BRAKE_CENTER, CURRENT_NOREV_BRAKE_ADC, DUTY, DUTY_REV_CENTER, PID, PID_REV_CENTER.
- Mode `*_BUTTON`, traction-control ADC, PPM, Nunchuk, NRF dan backend lain yang tidak memiliki hardware/modul pada build ini sengaja ditolak, bukan dipalsukan sebagai aktif.

## Run26: urutan uji sampai motor berputar

Setelah flash, roda harus bebas/terangkat. Power-on melody berjalan otomatis.
Kalibrasi manual memberi cue kalibrasi; transaksi konfigurasi yang benar-benar
tersimpan memberi dua beep pendek; saat running tidak ada beep status. Fault
VESC code N dibunyikan sebagai N beep pendek dan diulang setelah jeda.

1. Verifikasi kalibrasi dan ISR idle:

```bash
python3 tools/debug.py calibrate --timeout 8
python3 tools/debug.py stream-all --seconds 10 --csv run26_idle.csv
```

Target acceptance: `fault=0`, APP ADC tidak timeout, RT >=45 Hz, APP >=18 Hz,
dan `ISRmax < 4000`. Bila ISRmax tetap >=4000 jangan lanjutkan active test.

2. Sensorless LEFT dan RIGHT:

```bash
python3 tools/debug.py sensor-select --motor 0 --mode sensorless
python3 tools/debug.py sensor-select --motor 1 --mode sensorless
python3 tools/debug.py motor-test --motor 0 --mode current --value 1.0 --seconds 2 --yes
python3 tools/debug.py motor-test --motor 1 --mode current --value 1.0 --seconds 2 --yes
```

Jika rotor hanya bergetar tetapi tidak fault, naikkan bertahap ke 1.5 A lalu
2.0 A. Setelah current test memutar rotor, uji RPM:

```bash
python3 tools/debug.py speed-test --motor 0 --erpm 300 --seconds 5 --yes --csv left_run26.csv
python3 tools/debug.py speed-test --motor 1 --erpm 300 --seconds 5 --yes --csv right_run26.csv
```

3. Hall, bila ingin mode Hall:

```bash
python3 tools/debug.py hall-commission --motor both --yes
```

Perintah ini melakukan detect, apply, persist/readback, lalu current positif dan
negatif. Save yang berhasil menghasilkan dua beep pendek.

4. Uji cue save EEPROM secara aman saat motor berhenti:

```bash
python3 tools/debug.py config-save
```

Harus terdengar dua beep pendek setelah status save sukses.

## Run29 readiness / motor-not-ready diagnosis

Setelah upload, gunakan urutan ini sebelum motor test:

```bash
python3 tools/debug.py startup-check
python3 tools/debug.py calibrate --timeout 8
python3 tools/debug.py startup-check
```

`startup-check` menampilkan `motor_ready`, calibration, full/drive sampling flags, boot snapshot, fault kedua motor, PWM/MOE, dan status power-on melody. `sensor-select` sekarang memisahkan ACK pemilihan mode (`SELECT RESULT`) dari `detect_success(last)`, sehingga sensorless tidak lagi terlihat gagal hanya karena belum ada Hall/encoder detect.

Untuk audit source tanpa serial/hardware:

```bash
python3 tools/debug.py --self-test
python3 tools/audit_run29.py
```

## Run30 active drive acceptance

Run30 menambahkan:

```bash
python3 tools/debug.py brake-test --motor 0 --erpm 300 --current 1.0 --both-directions --yes
python3 tools/debug.py position-test --motor 0 --step 5 --seconds 2 --yes
python3 tools/debug.py drive-acceptance --yes
```

`drive-acceptance` menjalankan sensorless current, Hall autodetect/persist, duty, current, current-relative, RPM, real pre-spin brake, Hall-resolution-aware position, handbrake, dan EEPROM save. Gunakan hanya saat kedua roda terangkat dan emergency power cut tersedia.
