# VESC STM32F103RCT6 Dual Hoverboard FOC — V33

Port FOC dua motor untuk mainboard hoverboard STM32F103RCT6. Struktur sumber
mengikuti kepemilikan modul VESC sejauh masuk akal untuk target 48 KiB RAM:

- `src/motor/`: antarmuka motor, FOC, matematika, task, dan debug sampler.
- `src/hwconf/`: pemetaan/polaritas PCB, ADC, PWM, MOE, status, dan power latch.
- `src/applications/`: APP ADC PA2/PA3, UART, arbitrasi, dan default aplikasi.
- `src/comm/`: framing serta subset perintah VESC Tool yang benar-benar memiliki backend.
- `src/datatypes.h`: tipe serta ABI konfigurasi pusat; tidak ada header tipe alias.

Firmware mengiklankan nama `vesc-f103-hoverboard-v33-vesc-layout` dan sengaja
mempertahankan wire ABI VESC 6.00: MCCONF 481 byte dan APPCONF 493 byte.

## Validasi host

```sh
python3 tools/debug.py --self-test
python3 tools/test_v33_refactor_calibration.py
python3 tools/test_stage3_v32_compile_contract.py
python3 tools/test_stage3_v32_static_analysis.py
```

Seluruh regresi dapat dijalankan dengan:

```sh
for test_file in $(find tools -maxdepth 1 -type f -name 'test_*.py' | sort); do
  python3 "$test_file" || exit $?
done
```

## Uji hardware

Jalankan kalibrasi terlebih dahulu. Tes aktif hanya boleh dilakukan saat roda
terangkat, suplai dibatasi arus, dan penghentian darurat siap.

```sh
python3 tools/debug.py calibrate --port /dev/ttyUSB0 --baud 115200 --timeout 10

python3 tools/debug.py speed-test --port /dev/ttyUSB0 --baud 115200 \
  --motor 0 --erpm 300 --seconds 5 --csv speed_left.csv --yes

python3 tools/debug.py speed-test --port /dev/ttyUSB0 --baud 115200 \
  --motor 1 --erpm 300 --seconds 5 --csv speed_right.csv --yes
```

`speed-test` menyegarkan perintah dan meminta RT data pada 50 Hz, meminta APP
ADC pada 20 Hz secara independen, lalu mengukur rate aktual, timeout, slot yang
terlewat, jitter, dan fault. Opsi `--force` tidak boleh dipakai kecuali risiko
sensor yang belum lulus deteksi sudah dipahami.

Lihat `docs/V33_AUDIT_REPORT.md` dan `docs/V33_BENCH_CHECKLIST.md` sebelum flash.
