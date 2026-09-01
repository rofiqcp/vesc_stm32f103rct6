# Audit Checkpoint Run20 — Build Fix Sensorless

Tanggal: 2026-08-31

## Acceptance gate utama

Run ini memprioritaskan build log pengguna dari checkpoint Run19:

- `src/motor/mc_interface.c:2822`: `sensorless` undeclared di `mc_interface_set_configuration()`.
- `src/motor/mcpwm_foc.c:3393`: parameter `current_a` pada `detect_ramp_id()` tidak digunakan.

Sesuai kontrak pengguna, `pio run` tidak dijalankan pada checkpoint ini.

## Perubahan source

### `src/motor/mc_interface.c`

Menambahkan deklarasi lokal:

`const bool sensorless = c->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS;`

Deklarasi ditempatkan di `mc_interface_set_configuration()` sebelum policy mux sensor digunakan. Dengan demikian konfigurasi LEFT dan RIGHT dapat memilih `SENSOR_MODE_NO_SENSOR` saat `foc_sensor_mode` adalah sensorless tanpa mengandalkan variabel yang hanya hidup di fungsi validator.

### `src/motor/mcpwm_foc.c`

`detect_ramp_id()` sekarang menerima hanya `(MotorRuntime *m, uint32_t ramp_ms)`. Parameter `current_a` dihapus karena helper ini memang menjalankan ramp open-loop duty `SENSOR_DETECT_DUTY`, bukan current PI. Empat call-site disesuaikan.

`lock_current` pada pengukuran induktansi **tidak dihapus** karena tetap digunakan setelah ramp untuk menjaga rotor dengan target Id pada pulse Ld/Lq.

## Regression sensorless

`tools/audit_run19_dual_sensorless.py` kembali PASS setelah fix. Kontrak yang tetap dipertahankan meliputi:

- LEFT memakai TIM8 dan RIGHT memakai TIM1.
- sensorless LEFT/RIGHT memakai `SENSOR_MODE_NO_SENSOR` dan tidak memakai Hall/encoder sebagai sumber fase.
- forced-openloop memakai state per motor dan `foc_openloop_rpm`/`foc_openloop_rpm_low`.
- arah startup memperhitungkan `m_invert_direction`.
- transisi forced-openloop ke observer memakai phase coherence.
- FOC normal masuk `foc_svm_q15()` lalu `motor_hw_set_pwm_q15()` dan CCR1/2/3.

## Hasil test statis/host

PASS:

- Run20 build feedback audit.
- Run19 dual sensorless audit.
- Run18 telemetry contract.
- Run17 ISR/boot calibration.
- Run16 VESC 6.00 ABI.
- Run15 host C syntax portable units.
- Run14 operator integrity.
- Run13 ADC margin.
- Run12 clean format/dead static audit.
- Run11 full static firmware audit.
- RTOS audit.
- Communication audit.
- VESC protocol self-test.
- pytest: 95/95.
- Python compileall.
- shell `bash -n`.

Audit dokumentasi Run10 lengkap yang berat kembali melewati timeout harness pada run ini. Karena itu gate tersebut tidak diklaim PASS pada Run20. Perubahan fungsi/parameter Run20 sendiri memiliki komentar Bahasa Indonesia dan Run12 format audit lulus setelah perubahan.

## Yang belum terverifikasi

- Build PlatformIO final setelah fix ini: pengguna yang akan menjalankan sesuai instruksi `jangan pio run`.
- RAM/Flash aktual Run20.
- ISR FOC aktual pada hardware tetap <4000 cycle.
- Start/reverse/handover observer sensorless fisik LEFT dan RIGHT.
- RT Data 50 Hz pada hardware.
- Fault, Detect-All, EEPROM brownout, LED/buzzer pada hardware.

Tidak ada klaim hardware PASS pada item di atas.
