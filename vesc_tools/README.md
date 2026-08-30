# VESC BLDC Protocol Test Suite — tanpa library eksternal

Suite ini mengimplementasikan sendiri protokol serial VESC: CRC16, framing,
buffer big-endian, `COMM_FORWARD_CAN`, parser telemetri, serta command kontrol.
Runtime hanya membutuhkan Python 3.10+ di Linux; tidak perlu `pip install`.

## Peringatan keselamatan

1. Lepaskan propeller/chain/beban, atau angkat roda dari lantai.
2. Atur batas arus, ERPM, tegangan, suhu, dan duty yang benar di VESC Tool dahulu.
3. Mulai tes dengan nilai sangat kecil, misalnya duty `0.02` selama `0.3` detik.
4. Skrip gerak menolak berjalan tanpa `--arm`, dibatasi maksimum 30 detik, dan
   selalu mencoba mengirim `SET_DUTY=0` ketika selesai atau saat Ctrl+C.
5. Batas pada skrip bukan pengganti konfigurasi proteksi VESC. Nilai besar tetap
   dapat merusak motor, ESC, baterai, atau mencederai orang.
6. Skrip contoh juga memberi batas konservatif: duty/arus relatif ±0.25, arus
   absolut ±10 A, dan ERPM ±20.000. Turunkan lagi sesuai perangkat Anda.

## Tahap 0 — self-test tanpa hardware

```bash
cd vesc_bldc_protocol_suite
python3 selftest_protocol.py
```

Hasil yang diharapkan:

```text
OK: CRC, framing 2/3/4-byte, resync, dan buffer codec lulus.
```

## Tahap 1 — cek port dan hak akses

```bash
python3 01_list_ports.py
ls -l /dev/ttyACM0
```

Port umum adalah `/dev/ttyACM0` atau `/dev/ttyUSB0`. Bila muncul `Permission
denied`, tambahkan user ke grup serial distro (umumnya `dialout`), lalu logout dan
login kembali:

```bash
sudo usermod -aG dialout "$USER"
```

Jangan jalankan seluruh program sebagai root. Tutup VESC Tool lebih dahulu agar
port tidak sedang dipakai aplikasi lain.

## Tahap 2 — koneksi awal

```bash
python3 02_test_connection.py --port /dev/ttyACM0
```

Default: 115200 baud, timeout 1 detik, satu retry. Opsi yang sama berlaku pada
semua skrip:

```bash
--baud 115200 --timeout 1.0 --retries 1
```

## Tahap 3 — cari motor/VESC pada CAN bus

```bash
python3 03_scan_can_ids.py --port /dev/ttyACM0 --identify
```

`PING_CAN` memindai ID 0–254 di firmware, kemudian `--identify` membaca firmware
setiap node yang menjawab. Untuk mengakses satu node, tambahkan `--can-id`:

```bash
python3 04_get_firmware.py --port /dev/ttyACM0 --can-id 10
```

CAN termination, baud CAN, dan status CAN pada semua VESC harus cocok. VESC yang
terhubung USB adalah target `local` dan tidak ikut muncul sebagai node CAN remote.

## Tahap 4 — semua data GET

```bash
# Identitas firmware/hardware/UUID
python3 04_get_firmware.py --port /dev/ttyACM0

# COMM_GET_VALUES lengkap
python3 05_get_values_once.py --port /dev/ttyACM0

# Versi selective (bit 0..21), berguna untuk mengurangi bandwidth
python3 05b_get_values_selective.py --port /dev/ttyACM0 --mask 0x003FFFFF

# Setup/dashboard values: speed, battery, distance, odometer, uptime, dll.
python3 06_get_setup_values.py --port /dev/ttyACM0

# Input aplikasi
python3 09_get_inputs.py --port /dev/ttyACM0 --source all

# IMU, bila hardware/firmware menyediakannya
python3 10_get_imu.py --port /dev/ttyACM0 --mask 0xFFFF

# Terminal command; contoh output fault history
python3 11_terminal_command.py --port /dev/ttyACM0 faults
```

Semua perintah di atas juga menerima `--can-id 10`. Firmware lama/custom mungkin
belum memiliki field tambahan; parser `GET_VALUES` berhenti dengan aman saat field
opsional tidak ada. Command yang tidak didukung akan timeout dengan pesan jelas.

## Tahap 5 — telemetri realtime

Satu VESC, keluaran JSON Lines:

```bash
python3 07_monitor_realtime.py --port /dev/ttyACM0 --hz 10
python3 07_monitor_realtime.py --port /dev/ttyACM0 --can-id 10 --hz 10
```

Semua node CAN secara bergilir, CAN ID dicari otomatis:

```bash
python3 07b_monitor_all_can.py --port /dev/ttyACM0 --hz 5
```

Atau tentukan ID agar tidak scan ulang:

```bash
python3 07b_monitor_all_can.py --port /dev/ttyACM0 --can-ids 1,2,10 --hz 5
```

Log CSV 60 detik:

```bash
python3 08_log_csv.py --port /dev/ttyACM0 --can-id 10 \
  --hz 10 --duration 60 --output motor10.csv
```

`--duration 0` berarti merekam hingga Ctrl+C.

## Tahap 6 — SET/control satu per satu

Mulai dari stop dan duty rendah:

```bash
python3 12_stop_motor.py --port /dev/ttyACM0 --can-id 10
python3 13_set_duty.py --port /dev/ttyACM0 --can-id 10 \
  0.02 --duration 0.3 --hz 20 --arm
```

Command kontrol lain (contoh nilai sengaja rendah):

```bash
python3 14_set_current.py --port /dev/ttyACM0 0.5 --duration 0.3 --arm
python3 15_set_brake_current.py --port /dev/ttyACM0 0.5 --duration 0.3 --arm
python3 16_set_rpm.py --port /dev/ttyACM0 500 --duration 0.5 --arm
python3 17_set_position.py --port /dev/ttyACM0 5 --duration 0.5 --arm
python3 18_set_handbrake.py --port /dev/ttyACM0 0.5 --duration 0.3 --arm
python3 19_set_servo.py --port /dev/ttyACM0 0.5 --duration 0.5 --arm
python3 20_set_current_relative.py --port /dev/ttyACM0 0.01 --duration 0.3 --arm
python3 21_set_display_mode.py --port /dev/ttyACM0 1 --duration 0.5 --arm
python3 22_send_alive.py --port /dev/ttyACM0
```

Sisipkan `--can-id N` untuk node CAN. Posisi hanya bekerja bila sensor dan mode
kontrol motor sudah dikonfigurasi sesuai. Nilai RPM adalah **electrical RPM**,
bukan mechanical RPM. Konversi umum: `mechanical RPM = ERPM / pole_pairs`.

## Struktur implementasi

```text
vesc_protocol/
  crc.py           CRC16 bitwise poly 0x1021
  packet.py        frame/pemisahan stream dan validasi CRC
  buffer.py        integer big-endian, scaled float, float32-auto
  serial_linux.py  serial 8N1 via os/select/termios
  client.py        request/reply, timeout, retry, forward CAN
  parsers.py       firmware, values, setup, input, IMU, fault
  safety.py        arm, batas durasi, stop di finally/Ctrl+C
```

Referensi teknis resmi ada di `PROTOCOL_SOURCES.md`. Implementasi cocok dengan
struktur protokol `vedderb/bldc` master saat paket ini dibuat. Jika firmware F103
custom Anda mengubah nomor command atau susunan field, kirimkan output raw/commit
firmware tersebut agar `ids.py` dan parser dapat disesuaikan secara tepat.

Suite ini sengaja tidak menulis `MCCONF`, `APPCONF`, firmware, bootloader, atau
flash. Format konfigurasi tersebut dihasilkan per versi firmware, bersifat
persisten, dan salah payload dapat membuat controller tidak aman atau tidak bisa
boot. Semua GET telemetri dan SET kontrol runtime utama dipisahkan di atas.
