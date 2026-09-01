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
