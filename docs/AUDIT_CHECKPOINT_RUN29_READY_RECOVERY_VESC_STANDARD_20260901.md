# Audit Checkpoint Run29 — Motor Ready Recovery, Power-On Melody, dan Standar VESC

Tanggal: 2026-09-01

## Pemicu dari hasil hardware

Build/upload Run28 berhasil (`RAM 84.2%`, `Flash 63.2%`). Recalibration manual kemudian `RESULT CODE: 0`, tetapi dua command motor standar tetap ditolak:

- `COMM_SET_CURRENT`, motor LEFT: `result=4`;
- `COMM_SET_CURRENT`, motor RIGHT: `result=4`.

Pada firmware ini `result=4` adalah `CONTROL_CMD_RESULT_MOTOR_NOT_READY`. Artinya masalah terjadi **sebelum** `motor_set_current()`/FOC menerima perintah. Ini berbeda dari sensorless gagal start atau Hall invalid.

## Akar masalah Run28

Saat boot, `main.c` membaca `motor_hw_sampling_contract_flags()`. Run25 menambah audit register yang lebih ketat (sample time, ADC clock, DMA HT/TC, ADC3, EXTSEL/AFIO route, rank-6). Run28 menjadikan **semua** bit audit tersebut sebagai latch `s_motor_ready=false`.

Masalah kedua: setelah DMA dan current calibration kemudian terbukti sehat, tidak ada jalur yang menaikkan `s_motor_ready` kembali. Karena itu manual `calibrate` bisa valid tetapi command motor tetap `MOTOR_NOT_READY` sampai reset berikutnya.

## Perbaikan Run29

### 1. Readiness dipisahkan dari audit-only

Semua bit sampling tetap direkam untuk diagnosis. Gate motor menggunakan subset hard-critical:

- timer/PWM topology;
- ADC1/ADC2 sequence dan dual mode;
- DMA1/DMA2 transfer structure;
- ADC clock;
- DMA1 HT IRQ sebagai current ISR source.

Audit sample-time/detail trigger tambahan tetap tampil di `COMM_DIAG`, tetapi satu mismatch audit-only tidak lagi sendirian membuat motor terkunci permanen bila current sampling nyata dan kalibrasi sudah sehat.

### 2. Safe runtime recovery

`vesc_comm_try_recover_motor_ready()` ditambahkan. Recovery hanya boleh terjadi bila:

1. tidak ada shutdown latch;
2. tidak ada power-stage hardware latch;
3. current calibration `VALID`;
4. sampling drive-critical valid;
5. tidak ada fault lain.

Hanya `MOTOR_FAULT_ADC_DMA` **historis akibat boot readiness check** yang boleh dibersihkan otomatis pada jalur ini. Fault overcurrent, voltage, thermal, BREAK, encoder, Hall, observer, dan fault lain tidak dibypass.

Recovery diprobe setiap 100 ms oleh packet task dan sekali lagi pada boundary command motor. Jadi setelah manual calibration berhasil, command berikutnya tidak membutuhkan power-cycle hanya untuk mengubah `motor_ready`.

### 3. COMM_DIAG revision 18

Diagnostic sekarang mengekspor:

- `motor_ready`;
- `config_ready_runtime`;
- `ready_shutdown_latched`;
- `cal_done_runtime` / `cal_valid_runtime`;
- `sampling_drive_flags`;
- snapshot `sampling_boot_flags`;
- `motor_ready_recovery_count`.

Payload command VESC standar tidak diubah.

### 4. Power-on melody

Power-on melody TIM3 sekitar 3.21 s tetap dimulai sebelum scheduler. Run29 tidak lagi memotong melody hanya karena motor mulai atau karena fault runtime muncul selama melody. Fault tetap langsung mematikan bridge melalui jalur keselamatan; **yang ditunda hanya audio fault-code** sampai melody selesai. Setelah itu pola fault dapat berbunyi normal.

`COMM_SHUTDOWN` dan explicit diagnostic stop tetap boleh menghentikan buzzer karena memang merupakan tindakan eksplisit.

## Audit terhadap VESC master

`vesc(2).zip` dipakai sebagai acuan semantic, bukan disalin mentah ke STM32F103.

- `COMM_SET_CURRENT` upstream langsung menuju `mc_interface_set_current()` setelah parsing. Port F103 mempertahankan payload/scale VESC dan menambah readiness safety layer lokal.
- Hall detection upstream memaksa electrical phase, meramp `Id`, sweep forward/reverse, lalu membaca Hall sebagai **hasil pengukuran**. Hall tidak dipakai sebagai sumber phase selama detect.
- Startup sensorless upstream memetakan open-loop RPM dari current + `foc_sl_openloop_boost_q`, memakai `foc_openloop_rpm_low` sebagai fraksi, lalu pindah ke observer. Run28/29 telah mengikuti semantics ini dengan forced-phase + handover blend.

## Audit terhadap firmware hoverboard V10 yang diketahui berjalan

Referensi V10 memperlihatkan mode:

1. duty/PWM;
2. speed;
3. FOC torque/Iq;
4. open-loop `Id` tanpa Hall sebagai phase source.

Prinsip mode 4 tersebut dipertahankan pada Hall detect Run29: phase dipaksa oleh software, `Iq=0`, `Id` dinaikkan untuk lock/sweep, sedangkan Hall hanya disampling untuk membuat table.

## Makna output `sensor-select`

Field `success` pada CUSTOM_SENSOR_INFO adalah **hasil detect terakhir**, bukan ACK sensor selection. Karena itu output lama seperti:

`mode=3 request=3 success=False foc_sensor_mode=0`

sebenarnya berarti SENSORLESS sudah terpilih (`mode/request=3`, VESC `FOC_SENSOR_MODE_SENSORLESS=0`) dan `success=False` hanya berarti belum ada detect sensored yang sukses. `tools/debug.py` Run29 menampilkan `SELECT RESULT: PASS/FAIL` terpisah dari `detect_success(last)`.

## Urutan acceptance setelah flash

```bash
pio run -t upload
python3 tools/debug.py startup-check
python3 tools/debug.py calibrate --timeout 8
python3 tools/debug.py startup-check

python3 tools/debug.py sensor-select --motor 0 --mode sensorless
python3 tools/debug.py motor-test --motor 0 --mode current --value 1.0 --seconds 3 --yes

python3 tools/debug.py sensor-select --motor 1 --mode sensorless
python3 tools/debug.py motor-test --motor 1 --mode current --value 1.0 --seconds 3 --yes
```

Untuk Hall, mulai dari current rendah dan roda bebas dari beban mekanik:

```bash
python3 tools/debug.py hall-commission --motor 0 --yes
python3 tools/debug.py hall-commission --motor 1 --yes
```

Jika motor tidak bergerak setelah command sudah `ACCEPTED`, kirim output:

```bash
python3 tools/debug.py startup-check
python3 tools/debug.py comm-diag
python3 tools/debug.py sensor-info
python3 tools/debug.py diagnose --timeout 8
```

Run29 membedakan command reject dari command accepted-but-PWM/observer/startup tidak berhasil.

## Verifikasi yang dilakukan pada paket Run29

- `gcc -std=c11 -Wall -Wextra -Werror -fsyntax-only -DFOC_MATH_UNIT_TEST -Isrc src/motor/foc_math.c`: PASS.
- `python3 -m py_compile tools/debug.py tools/vesc.py tools/audit_run29.py`: PASS.
- `python3 tools/debug.py --self-test`: PASS.
- `python3 tools/audit_run29.py`: harus PASS seluruh invariant source.

## Batas verifikasi di lingkungan pembuatan paket

Toolchain `pio`/`arm-none-eabi-gcc` tidak tersedia di lingkungan pembuatan ZIP ini, sehingga **full STM32 cross-build dan hardware spin tidak diklaim sudah dijalankan di sini**. Build Run28 yang Anda kirim sendiri sudah terbukti SUCCESS; patch Run29 dibuat kecil dan diverifikasi dengan host syntax/self-test/static audit. Acceptance final tetap dilakukan dengan command di atas pada board STM32F103RCT6.
