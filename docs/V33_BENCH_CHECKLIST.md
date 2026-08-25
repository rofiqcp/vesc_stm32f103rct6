# Checklist bench V33

## 1. Persiapan aman

- Roda kiri dan kanan terangkat serta bebas menyentuh benda.
- Suplai DC memakai current limit konservatif dan fuse.
- Tombol/pemutus daya dapat dijangkau langsung.
- Jangan pasang beban manusia.
- Jangan memakai `--force` pada speed test sebelum sensor motor tersebut lulus.

## 2. Build target

1. Build dengan PlatformIO/ARM GCC yang sama dengan produksi.
2. Catat commit/ZIP, versi compiler, dan map file.
3. Pastikan target tetap STM32F103RCT6 dan linker RAM 49152 byte.
4. Rekam output RAM/flash V33. Jangan memakai angka V32 sebagai hasil V33.
5. Pastikan tidak ada warning sumber proyek dan tidak ada overflow stack/heap.

## 3. Passive boot

```sh
python3 tools/debug.py info --port /dev/ttyUSB0 --baud 115200
python3 tools/debug.py comm-diag --port /dev/ttyUSB0 --baud 115200
python3 tools/debug.py diagnose --port /dev/ttyUSB0 --baud 115200 --timeout 10
```

Kriteria:

- Nama firmware `vesc-f103-hoverboard-v33-vesc-layout`.
- UART, ISR, ADC1/2, DMA1 CH1, ADC3, dan DMA2 CH5 hidup.
- Sampling contract flags nol.
- Tidak ada stale Vbus, watchdog miss, power-stage latch, atau fault flash.

## 4. Kalibrasi revision 17

```sh
python3 tools/debug.py calibrate --port /dev/ttyUSB0 --baud 115200 \
  --timeout 10 --out calibration_v33.txt
```

Wajib lulus:

- `done=True`, `valid=True`, `count=target=6096`.
- `fail_range_mask=0`, `fail_noise_mask=0`, `moe_fail_mask=0`.
- `moe_confirmed_mask=0x03`.
- LEFT dan RIGHT mempunyai request, confirm, dan first-sample ADC nonzero.
- Request→confirm kecil dan konsisten dengan dua preload event; tidak mendekati
  timeout 128 event.
- Setiap kanal mempunyai paling sedikit 990 inlier (`1000-outlier_count`).
- Tidak ada offset mean dekat 0/4095.
- Residual Id/Iq zero-current berada dalam batas commissioning yang dipilih.

Jika outlier >10 atau clean stddev >80, **berhenti**. Jangan menaikkan limit.

## 5. Scope MOE/PWM/ADC

Periksa terpisah untuk LEFT TIM8 dan RIGHT TIM1:

- Semua CCR = 50% sebelum MOE naik.
- High-side dan complementary low-side mengikuti polaritas board.
- Tidak ada overlap gate; dead-time benar.
- ADC trigger TIM8 TRGO berada di jendela tenang, bukan di ringing edge.
- Setelah stop/fault, MOE kedua timer rendah dan seluruh gate benar-benar OFF.

## 6. Sensor

Jalankan deteksi dengan arus rendah yang sesuai motor. LEFT dapat ABI/Hall/
sensorless; RIGHT hanya Hall/sensorless. Verifikasi arah, pole pairs, tabel Hall,
offset encoder, dan tidak ada phase jump saat transisi observer.

## 7. Speed test dua motor

```sh
python3 tools/debug.py speed-test --port /dev/ttyUSB0 --baud 115200 \
  --motor 0 --erpm 300 --seconds 5 --csv speed_left_300.csv --yes

python3 tools/debug.py speed-test --port /dev/ttyUSB0 --baud 115200 \
  --motor 1 --erpm 300 --seconds 5 --csv speed_right_300.csv --yes
```

Kriteria tiap motor:

- Command rate dan RT data mendekati 50 Hz; default minimum 45 Hz.
- APP data mendekati 20 Hz; default minimum 18 Hz.
- Timeout = 0, fault = 0, dan tidak ada slot terlewat yang berulang.
- ERPM mengikuti target tanpa Id/Iq atau duty tidak wajar.
- Motor yang tidak diuji tetap berhenti.

Naikkan ERPM/beban bertahap hanya setelah hasil 300 ERPM bersih.

## 8. Release gate

Firmware baru dapat dinyatakan siap produksi setelah tersedia:

- binary/map ARM V33;
- laporan kalibrasi revision 17 kiri+kanan;
- CSV speed test kiri+kanan;
- bukti scope PWM/MOE/ADC;
- soak test fault/timeout/power-cycle;
- hasil temperatur yang aman sesuai strategi hardware aktual.
