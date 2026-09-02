# Audit Checkpoint Run28 - Compile Scope Fix

Tanggal: 2026-09-01

## Feedback build pengguna
Compiler menemukan `dir` dan `iq_hint_a` tidak terdefinisi di `foc_encoder_ab_startup_1khz()`.

## Akar masalah
Blok alignment torque yang ditujukan untuk `foc_sensorless_startup_1khz()` ikut tersisip ke jalur startup encoder. Fungsi encoder tidak menerima parameter direction/torsi sensorless sehingga source gagal compile.

## Perbaikan
- Mengembalikan wait-path encoder ke target FOC nol selama MOE/current-sense blanking belum siap.
- Mempertahankan alignment Iq sensorless di `foc_sensorless_startup_1khz()` tempat `direction_hint` dan `iq_hint_a` memang berada pada scope yang valid.
- Tidak mengubah ABI VESC, ADC mapping, calibration, RT50/App20, Hall detect, buzzer, atau persistence pada patch build-fix ini.

## Verifikasi tanpa pio run
- `gcc -std=c11 -Wall -Wextra -Werror -fsyntax-only -DFOC_MATH_UNIT_TEST ... src/motor/foc_math.c`: PASS.
- `python3 tools/debug.py --self-test`: PASS.
- `python3 -m py_compile tools/debug.py tools/vesc.py`: PASS.
- Scope guard: `iq_hint_a`/`wait_limit = dir` tidak muncul pada badan startup encoder dan tetap ada pada startup sensorless: PASS.
- Tidak ada tab/trailing whitespace pada source C/H: PASS.
- `#if 0`: tidak ditemukan.
- Python project tetap hanya `tools/debug.py` dan `tools/vesc.py`.

## Belum diverifikasi
Build PlatformIO penuh, penggunaan Flash/RAM, motor hardware, Hall detect hardware, dan ISR cycle hardware harus diverifikasi pengguna setelah flash. `pio run` tidak dijalankan sesuai instruksi.
