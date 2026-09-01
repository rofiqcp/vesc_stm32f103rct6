# Audit Checkpoint Run19 — Dual Sensorless FOC/SVPWM

Tanggal: 2026-08-31

## Basis

Checkpoint kerja berasal dari Run18 fisik yang telah dipulihkan. Run19 tidak menjalankan `pio run` sesuai instruksi pengguna.

## Fokus Run19

Audit difokuskan pada kemampuan motor LEFT dan RIGHT berjalan dalam mode FOC `SENSORLESS` tanpa Hall/encoder sebagai sumber fase. Jalur yang diaudit mencakup pemilihan mode dari boot/default, SET MCCONF VESC Tool, custom sensor selection, Detect-All, startup forced-openloop, handover observer, FOC dq, inverse Park, SVPWM, penulisan CCR, arah motor inverted, dan pemulihan sensor setelah proses deteksi.

## Bug/risiko yang ditemukan dan diperbaiki

1. **Arah forced-openloop belum mengikuti `m_invert_direction`.**
   - Sebelumnya arah startup memakai tanda command eksternal, sedangkan Iq setelah handover baru dibalik oleh `m_invert_direction`.
   - Pada motor yang orientasinya dibalik, startup dapat memaksa rotor ke arah fisik yang berbeda dari arah Iq setelah handover.
   - Run19 membalik `direction_hint` sebelum state machine sensorless jika `m_invert_direction` aktif, sehingga arah forced-openloop dan FOC setelah handover berada pada koordinat fisik yang sama.

2. **Mode FOC SENSORLESS masih membiarkan mux Hall fisik aktif.**
   - Fase FOC memang sudah berasal dari observer, tetapi EXTI Hall yang tidak digunakan masih dapat menerima noise.
   - Ditambahkan runtime mode internal `SENSOR_MODE_NO_SENSOR` yang terpisah dari enum ABI VESC.
   - LEFT: TIM4 encoder dihentikan, EXTI Hall dimask, PB5/PB6/PB7 menjadi input pasif.
   - RIGHT: EXTI Hall dimask dan pin Hall menjadi input pasif.
   - ISR Hall tetap hanya memproses motor yang physical runtime mode-nya `SENSOR_MODE_HALL`.

3. **SET MCCONF masih memaksa physical mux ke Hall untuk sensorless.**
   - Jalur `confgenerator.c` sebelumnya dapat membatalkan true-no-sensor setelah VESC Tool mengirim MCCONF.
   - Sekarang LEFT/RIGHT menurunkan physical mux dari `foc_sensor_mode`: encoder LEFT -> ABI, Hall -> Hall EXTI, SENSORLESS -> NO_SENSOR.
   - Wire field `m_sensor_port_mode` tetap dipertahankan sesuai VESC 6.00 untuk round-trip ABI.

4. **Detect-All sensorless fallback masih mengaktifkan Hall input.**
   - `apply_sensorless_result()` sekarang mematikan encoder/Hall fisik dan memilih NO_SENSOR pada kedua motor.

5. **Validasi Detect-All memakai threshold hybrid yang salah.**
   - Sebelumnya validasi observer memakai `foc_sl_erpm` (default 2500 ERPM), sedangkan pure-sensorless startup memakai `foc_openloop_rpm`/`foc_openloop_rpm_low` (default openloop 900 ERPM).
   - Ini dapat membuat startup/handover observer berhasil tetapi Detect-All tetap menyatakan gagal.
   - Run19 menjadikan state machine startup sebagai pemilik threshold handover; worker Detect-All hanya memerlukan observer valid, forced override selesai, openloop selesai, dan kecepatan minimum 50 ERPM selama window stabil.

6. **Dead enum runtime lama.**
   - `SENSOR_MODE_FORCED_OPENLOOP` tidak memiliki pengguna nyata dan dihapus/diganti oleh `SENSOR_MODE_NO_SENSOR` yang benar-benar dipakai untuk mux fisik.

7. **Deteksi non-Hall melakukan call Hall yang tidak perlu.**
   - `detect_begin()` sekarang hanya memanggil `motor_hall_edge_isr()` jika mode fisik memang Hall.

## Jalur sensorless LEFT dan RIGHT setelah Run19

1. Command current/speed/duty diterima pada koordinat VESC eksternal.
2. Untuk SENSORLESS, service 1 kHz membentuk `direction_hint` dan mengoreksinya dengan `m_invert_direction` agar menjadi arah fisik.
3. `foc_sensorless_startup_1khz()` menjalankan lock/ramp berdasarkan `foc_openloop_rpm` dan `foc_openloop_rpm_low` (0..1).
4. `phase_observer_override_u16` menjadi sumber fase selama forced-openloop.
5. ISR FOC 16 kHz tetap menjalankan observer dari tegangan/arus motor pada masing-masing `MotorRuntime`.
6. Current PI menghasilkan Vd/Vq, inverse Park menghasilkan alpha/beta.
7. `foc_svm_q15()` menghasilkan duty U/V/W.
8. `motor_hw_set_pwm_q15()` menulis CCR1/CCR2/CCR3 secara koheren ke timer motor yang bersangkutan:
   - LEFT -> TIM8
   - RIGHT -> TIM1
9. Setelah observer valid dan fase koheren, forced phase diblend menuju observer sampai error <= sekitar 1 derajat listrik, lalu override dilepas.
10. Fase selanjutnya berasal dari `observer_phase_compensated_u16()` tanpa Hall/encoder.

State `openloop_started`, `openloop_erpm_now`, `phase_observer_override`, phase override dan failure counter berada di `MotorRuntime`, sehingga LEFT dan RIGHT tidak berbagi state startup.

## Catatan command sensorless

Current, speed, dan duty dapat melakukan forced startup dari diam secara statis berdasarkan jalur di atas. Brake/handbrake/position pada pure sensorless tidak mengarang posisi rotor saat diam; jalur tersebut baru aman bila observer sudah valid dari kondisi rotor yang sebelumnya bergerak. Ini merupakan batas observabilitas sensorless, bukan fallback ke Hall/encoder.

## File source yang berubah terhadap Run18

- `src/datatypes.h`
- `src/hwconf/hw.c`
- `src/motor/mc_interface.c`
- `src/motor/mcpwm_foc.c`
- `src/confgenerator.c`
- `src/comm/commands.c`

## File audit/tool yang berubah/ditambahkan

- `tools/audit_run19_dual_sensorless.py` (baru)
- `tools/run_static_acceptance.py`
- `docs/AUDIT_CHECKPOINT_RUN19_DUAL_SENSORLESS_CLEAN_20260831.md` (baru)

## Hasil acceptance statis/host

PASS yang benar-benar dijalankan setelah perubahan logic Run19:

- Run19 dual sensorless contract
- Run18 telemetry contract
- Run17 ISR/boot calibration
- Run16 VESC 6.00 ABI
- Run15 portable host C syntax
- Run14 operator integrity
- Run13 ADC margin
- Run12 clean formatting
- Run8 contract
- Run9 contract
- Run11 full static contract
- RTOS audit
- communication audit
- VESC protocol CRC/framing/resync self-test
- pytest: 95/95
- Python compileall
- shell `bash -n`

Run12 clean-format juga melaporkan:

- tab C/H: 0
- trailing whitespace C/H: 0
- one-line block/control terlarang: 0
- multiple statement padat: 0
- `#if 0`: 0
- obvious unreferenced static function: 0

Audit dokumentasi legacy Run10 dicoba kembali tetapi melewati timeout harness pada environment ini. Tidak ada fungsi C baru pada Run19; seluruh fungsi C yang diedit tetap mempunyai komentar `// Fungsi ...` Bahasa Indonesia di atas definisinya. Run8/Run9 function-comment gate tetap PASS. Timeout tool dokumentasi tidak diklaim sebagai PASS.

## Hal yang belum terverifikasi tanpa build/hardware

- ukuran RAM/Flash aktual Run19;
- cycle ISR aktual Run19 dan margin terhadap 4000 cycle;
- motor LEFT benar-benar start/reverse/handover observer pada hardware;
- motor RIGHT benar-benar start/reverse/handover observer pada hardware;
- tuning `foc_openloop_rpm`, current boost, R/L/flux dan observer untuk motor fisik pengguna;
- RT Data 50 Hz fisik saat kedua motor berjalan;
- fault/noise/brownout/thermal behavior di bawah beban nyata.

Karena `pio run` tidak dijalankan, tidak ada klaim PASS untuk build, Flash/RAM, atau hardware.
