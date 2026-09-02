# Audit Checkpoint Run30 — Hoverboard Real-Mode Cross-Check + VESC Drive Acceptance

Tanggal: 2026-09-01

## Tujuan

Run30 melanjutkan Run29 dengan membandingkan jalur motor terhadap firmware referensi `hoverboard-firmware-hack-FOC_VESC_DUAL.zip` yang dinyatakan sudah berjalan nyata pada hardware. Fokus Run30 bukan mengganti API VESC dengan mode proprietary hoverboard, tetapi memastikan primitive motor, ADC, FOC, Hall detect, dan enable bridge yang dipakai VESC tetap konsisten dengan jalur hardware yang sudah terbukti.

## Hasil cross-check firmware hoverboard referensi

Suite host bawaan referensi dijalankan penuh dan menghasilkan `ALL_FINAL_HOST_CHECKS_PASS`. Log lengkap disimpan di `docs/HOVERBOARD_REFERENCE_HOST_VALIDATION_RUN30.txt`.

Perilaku mode referensi yang dipakai sebagai pembanding:

- Mode 1 / voltage-PWM: menghasilkan modulasi PWM/SVPWM dan membuktikan jalur bridge.
- Mode 2 / speed: loop speed membentuk target torsi dan melepaskan bridge setelah target nol tercapai.
- Mode 3 / torque: Id=0 dan Iq menjadi pembentuk torsi.
- Mode 4 / Id open-loop sensorless: sudut listrik dipaksa, Id diberikan, Iq=0. Sensor Hall tidak digunakan untuk menentukan sudut saat eksitasi.

Run30 mempertahankan semantik command VESC asli. `COMM_SET_DUTY` tetap VESC duty control, bukan menyalin mentah mode-1 proprietary. Namun jalur inverter, SVPWM, current sensing, dan phase actuation memakai hardware primitive yang sudah dicocokkan dengan referensi.

## ADC dan current calibration

Mapping dual ADC Run29/Run30 cocok dengan firmware hoverboard referensi:

- rank 1: ADC1 PC1 = RIGHT DC, ADC2 PC0 = LEFT DC
- rank 2: ADC1 PA0 = LEFT phase A, ADC2 PC3 = LEFT phase B
- rank 3: ADC1 PC4 = RIGHT phase B, ADC2 PC5 = RIGHT phase C

Polaritas current juga sama: `offset - raw`.

Offset tetap diambil pada 50% zero-vector dengan bridge aktif. Ini disengaja karena board hoverboard dapat mengalami common-mode shift besar ketika bridge OFF vs aktif. Karena itu nilai phase-current yang terlihat besar setelah bridge kembali OFF tidak boleh dipakai sendiri untuk menyimpulkan offset FOC aktif salah. Acceptance utama tetap residual Id/Iq pada kondisi active-zero-vector, ADC/DMA validity, dan fault/current-trip.

## Hall autodetect

Hall detect Run30 mengikuti prinsip yang sama dengan mode-4 referensi:

1. bridge di-enable secara aman;
2. sudut listrik dipaksa (`detect_force_angle`);
3. Id dinaikkan bertahap, Iq=0;
4. fase listrik disweep maju dan mundur;
5. Hall hanya disampling sebagai hasil pengamatan;
6. enam state harus valid, Gray-code topology harus benar;
7. hasil diterapkan ke MCCONF dan dipersist.

Dengan demikian Hall tidak dipakai sebagai phase feedback untuk proses detect itu sendiri.

## Command VESC yang wajib diuji aktif

Run30 memastikan handler berikut melewati readiness boundary yang sama sebelum mencapai motor API:

- `COMM_SET_DUTY` -> `motor_set_duty()`
- `COMM_SET_CURRENT` -> `motor_set_current()`
- `COMM_SET_CURRENT_BRAKE` -> `motor_set_brake_current()`
- `COMM_SET_RPM` -> `motor_set_speed()`
- `COMM_SET_POS` -> `motor_set_position()`
- `COMM_SET_HANDBRAKE` -> `motor_set_handbrake()`
- `COMM_SET_CURRENT_REL` -> `motor_set_current_rel()`

`tools/debug.py drive-acceptance --yes` sekarang membuktikan jalur tersebut secara aktif, bukan hanya melihat ACK command.

## Brake acceptance

`brake-test` tidak menguji brake saat roda diam. Tool melakukan pre-spin dengan `SET_RPM`, memastikan ERPM minimum tercapai, lalu beralih langsung ke `SET_CURRENT_BRAKE`. Acceptance memerlukan:

- command trace `SET_CURRENT_BRAKE` diterima;
- braking Iq target/actual muncul;
- |ERPM| turun secara terukur;
- tidak ada fault.

Full drive acceptance melakukan brake dua arah.

## Position acceptance

SET_POS diuji setelah Hall commissioning. Untuk Hall, tool menghitung resolusi mekanik kira-kira `60 / pole_pairs` derajat per Hall edge dan membesarkan step test menjadi minimal 1.25 edge. Hal ini mencegah false-fail ketika target lebih kecil daripada resolusi Hall.

Pure sensorless SET_POS dari keadaan diam tetap ditolak oleh precheck kecuali dipaksa, karena tidak ada absolute rotor-position reference. Ini sesuai batas fisik, bukan kekurangan protocol.

## Buzzer dan LED Run30

Perilaku final:

- power-on melody: tetap aktif seperti Run29;
- normal running: **3 pulse LED saja**;
- normal running tidak menghasilkan 3 beep buzzer;
- detect/running menunda cue audio non-fault;
- fault tone tetap memiliki fungsi safety diagnostics;
- setiap transaksi EEPROM/flash yang benar-benar sukses mengantrikan **tepat 5 beep** setelah kondisi audio aman/idle.

Notifikasi EEPROM dipindahkan ke empat wrapper transaksi flash terendah di `conf_general.c`, sehingga semua jalur persistence memakai satu sumber kebenaran. High-level notification yang duplikat di `commands.c`/`confgenerator.c` dihapus.

Counter pending digunakan agar beberapa save berurutan tidak terkoalesensi menjadi satu cue.

Jalur persistence yang tercakup antara lain MCCONF, APPCONF, detect/commit runtime, custom config-save, dual-store, dan auxiliary/odometer persistence.

## Debug acceptance baru

Command utama:

```bash
python3 tools/debug.py drive-acceptance --yes
```

Urutannya:

1. STOP + clear fault kedua motor
2. current calibration
3. startup/readiness validation
4. sensorless select + SET_CURRENT +/- pada LEFT dan RIGHT
5. Hall autodetect + apply + persist + current +/- pada LEFT dan RIGHT
6. Hall-mode SET_DUTY +/-
7. SET_CURRENT +/-
8. SET_CURRENT_REL +/-
9. SET_RPM +/-
10. pre-spin + SET_CURRENT_BRAKE dua arah
11. SET_POS maju dan balik dengan Hall-resolution-aware step
12. SET_HANDBRAKE / Id lock
13. explicit EEPROM save + sequence increment
14. final fault + Hall MCCONF/readback check

Command terpisah juga tersedia:

```bash
python3 tools/debug.py brake-test --motor 0 --erpm 300 --current 1.0 --both-directions --yes
python3 tools/debug.py brake-test --motor 1 --erpm 300 --current 1.0 --both-directions --yes
python3 tools/debug.py position-test --motor 0 --step 5 --seconds 2 --yes
python3 tools/debug.py position-test --motor 1 --step 5 --seconds 2 --yes
```

`full-test --yes` sekarang menjadi compatibility alias ke acceptance Run30 yang lebih lengkap.

## Verifikasi offline yang dijalankan

- firmware referensi hoverboard: `ALL_FINAL_HOST_CHECKS_PASS`
- `python3 tools/debug.py --self-test`: PASS
- `python3 tools/audit_run29.py`: 25/25 PASS
- `python3 tools/audit_run30.py`: 41/41 PASS
- structural delimiter balance untuk file C yang disentuh: PASS
- Python bytecode compile: PASS

Cross-build STM32 belum dijalankan di environment audit ini karena `pio` dan toolchain ARM PlatformIO tidak tersedia. Build/upload terakhir dari sisi hardware user pada basis Run28/Run29 sudah SUCCESS. Karena itu status Run30 adalah: source/audit/acceptance logic siap untuk cross-build dan pengujian board, tetapi klaim gerakan fisik akhir tetap harus didasarkan pada hasil `drive-acceptance` yang dijalankan pada board user.
