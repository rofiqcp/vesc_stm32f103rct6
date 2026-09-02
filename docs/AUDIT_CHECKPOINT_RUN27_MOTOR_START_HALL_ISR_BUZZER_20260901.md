# AUDIT CHECKPOINT RUN27 — MOTOR START, HALL ISR, BUZZER

Tanggal: 2026-09-01

## Acceptance gate dari hardware Run26

Log pengguna menunjukkan:
- current calibration 2000 sampel valid dan offset phase/DC masuk akal;
- RT dedicated mencapai ~50 Hz dan App Data 20 Hz;
- SET_CURRENT/SET_RPM sebelumnya tidak membuktikan gerakan: ERPM, duty, Id/Iq tetap nol;
- Hall detect pernah mencapai isr_max_cycles sekitar 5831 dan FOC_ISR_OVERRUN;
- Hall raw sendiri sudah dapat terbaca valid.

Run27 memprioritaskan masalah di atas. Tidak ada `pio run` yang dijalankan.

## Perubahan firmware

1. Sensorless startup dari zero-speed
   - forced-openloop sekarang langsung menerbitkan alignment Iq = |command| + foc_sl_openloop_boost_q, dibatasi live current limit;
   - alignment Iq tetap dipertahankan selama MOE/pwm blanking handshake;
   - menghilangkan kondisi startup yang terlihat aktif tetapi target arus tetap nol.

2. Hall/forced-angle detection ISR
   - saat `detect_force_angle && detect.busy`, ISR tetap menjalankan current acquisition, Clarke/Park, current PI, inverse Park, SVPWM dan proteksi;
   - observer update, encoder source update, PLL, fast speed estimator, field-weakening fast target dan dq-decoupling dilewati karena tidak diperlukan pada forced-angle sweep;
   - targetnya mengurangi Hall detect worst-case cycle dan mencegah FOC_ISR_OVERRUN yang terlihat pada Run26.

3. Current Ki temperature compensation
   - konversi float `Ki * dt` dipindahkan ke task 1 kHz;
   - hard ISR hanya membaca `current_ki_dt_q16` fixed-point.

4. Low-side current sampling window
   - sampling margin disamakan dengan reference V5: 110 timer ticks dari tepi ARR=2000;
   - rentang CCR efektif sekitar 5.5% sampai 94.5%;
   - enam current channels tetap berada pada tiga dual-rank pertama dan DMA HT tetap menjadi entry FOC agar temperature/APP ADC tidak menunda hard loop.

5. Buzzer
   - power-on melody tetap aktif;
   - calibration cue tetap aktif;
   - EEPROM/config save sukses = tepat 2 beep pendek;
   - active fault code VESC N = N beep pendek per burst;
   - detect/running tidak memiliki status beep;
   - bila running/detect dimulai saat cue non-fault masih aktif, tone dihentikan dan cue ditunda sampai idle;
   - fault tetap memiliki prioritas tertinggi dan dapat memotong startup melody.

## Perubahan tools/debug.py

- motor-test membaca COMM_DIAG setelah command pertama dan mensyaratkan:
  - `last_control_result == ACCEPTED`;
  - command ID sesuai;
  - motor target sesuai.
- untuk current/RPM/duty, PASS memerlukan target/PWM aktif dan motion >= 20 ERPM.
- speed-test tidak lagi PASS hanya karena RT 50/App20 sehat; PASS juga memerlukan motion aktual.

## File berubah dari Run26

- `src/applications/appconf_default.h`
- `src/hwconf/hw_status.c`
- `src/motor/foc_math.c`
- `src/motor/mc_interface.c`
- `src/motor/mcpwm_foc.c`
- `tools/debug.py`
- `docs/AUDIT_CHECKPOINT_RUN27_MOTOR_START_HALL_ISR_BUZZER_20260901.md`

## Host/static tests

- `python3 tools/debug.py --self-test` — PASS
- `python3 -m py_compile tools/debug.py tools/vesc.py` — PASS
- portable `foc_math.c` compile `-Wall -Wextra -Werror` — PASS
- Run27 key contracts — PASS
- `#if 0` — 0
- broken shift operator patterns — 0
- obvious unreferenced static functions — 0
- C/H tabs — 0
- C/H trailing whitespace — 0
- Python project files — exactly `tools/vesc.py` and `tools/debug.py`
- `vesc_tools/` — absent

## Belum boleh diklaim PASS

Karena `pio run` dan hardware test Run27 tidak dijalankan di sini, berikut masih memerlukan log pengguna:
- build/link dan RAM/Flash Run27;
- active sensorless LEFT/RIGHT benar-benar berputar;
- Hall detect LEFT/RIGHT berhasil mendapatkan enam-state table;
- Hall current +/- dan RPM benar-benar bergerak;
- isr_total_max_cycles pada active sensorless/Hall < 4000;
- fault-code buzzer fisik dan power-on melody fisik;
- EEPROM save beep fisik.
