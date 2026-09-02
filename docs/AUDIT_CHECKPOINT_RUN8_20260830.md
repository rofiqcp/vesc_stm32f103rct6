# Audit Checkpoint Run 8 — 2026-08-30

Checkpoint ini melanjutkan Run 7 tanpa menjalankan `pio run` sesuai instruksi pengguna.

## Fokus perbaikan

1. **Terminal VESC angka kosong**
   - Semua output float pada `terminal.c` dipindahkan ke formatter fixed-point ringan.
   - Tidak menambahkan `_printf_float`, sehingga tidak menambah library printf-float besar ke Flash.
   - `currents`, `foc`, `observer`, `appadc`, `timing`, `config`, dan `stats` sekarang mengirim angka sebagai `%s` hasil formatter integer/fixed-point.
   - Ditambahkan command `adc` untuk raw ADC current, offset phase/DC, skala 20 mA/count, progress kalibrasi, stage, dan validitas.

2. **MCCONF VESC firmware 6.00**
   - Layout dibandingkan dengan `vedderb/bldc` tag `6.00/confgenerator.c`.
   - MCCONF tetap 481 byte dan APPCONF 493 byte.
   - Offset penting: `foc_sensor_mode=152`, `foc_hfi_start_samples=265`.
   - Full wire image tetap menjadi source-of-truth SET/GET/persistence agar field yang tidak dieksekusi F103 tidak hilang saat round-trip.
   - Factory MCCONF sekarang menerbitkan sensor mode berdasarkan FOC mode runtime/default yang sebenarnya, termasuk SENSORLESS, bukan selalu Hall.
   - Hardware-limit safety tetap dipertahankan; nilai yang benar-benar berada di luar limit power-stage masih boleh diklem seperti perilaku VESC upstream.

3. **ADC/current calibration dan mapping referensi FULL_MODES**
   - Boot offset calibration mengikuti referensi yang terbukti bergerak: 2000 sampel, seed 2000 count, update `(raw + offset) / 2`.
   - Polaritas current adalah `offset - raw`.
   - Phase dan DC current scale adalah 0.0200 A/count (`A2BIT_CONV=50` ekuivalen).
   - Mapping DMA fast ranks:
     - word0 LOW RIGHT DC PC1, HIGH LEFT DC PC0
     - word1 LOW LEFT A PA0, HIGH LEFT B PC3
     - word2 LOW RIGHT B PC4, HIGH RIGHT C PC5
   - Third phase direkonstruksi dengan jumlah tiga fase = 0.

4. **PWM / CCR**
   - LEFT TIM8: U=CCR1, V=CCR2, W=CCR3.
   - RIGHT TIM1: U=CCR1, V=CCR2, W=CCR3.
   - Triplet CCR memakai preload dan ditulis koheren.

5. **Sensorless FOC/SVPWM**
   - SENSORLESS tetap enum wire `0` dan diterima pada LEFT maupun RIGHT.
   - HFI tidak digunakan sebagai backend sensorless target ini.
   - Low-speed sensorless memakai forced/open-loop phase lalu handover ke flux observer/PLL saat observer valid dan threshold ERPM tercapai.
   - Hall GPIO boleh tetap menjadi input fisik, tetapi Hall phase/RPM hanya dipakai ketika `foc_sensor_mode == HALL`.

6. **Proteksi arus dan dead code**
   - Blok proteksi phase-current yang sebelumnya dibungkus `#if 0` diaktifkan kembali.
   - Startup blanking tetap memakai guard DC current untuk mencegah fault palsu akibat switching transient pertama.
   - Tidak ada lagi `#if 0` pada source proyek.
   - Duplicate include dibersihkan.
   - Heuristik static-function tidak menemukan static function tanpa reference.

7. **Komentar fungsi**
   - Source milik proyek (`src/`, kecuali `freertos_vendor`) diaudit otomatis.
   - 910/910 definisi fungsi terdeteksi memiliki komentar `// Fungsi ...` Bahasa Indonesia tepat sebelum definisi.
   - Third-party FreeRTOS vendor sengaja tidak dimodifikasi untuk menjaga source upstream tetap bersih.

## Verifikasi host-side

- `python tools/rtos_audit.py` — PASS
- `python tools/comm_audit.py` — PASS
- `python tools/debug.py --self-test` — PASS
- `python vesc_tools/selftest_protocol.py` — PASS
- `python -m pytest vesc_tools/vesc_protocol/tests -q` — 95/95 PASS
- `python tools/audit_run8_contract.py` — PASS seluruh kontrak Run 8

`python -m pytest -q` tanpa scope mencoba mengoleksi `tools/vesc_tool_test.py`, yang memang membutuhkan `pyserial` dan hardware serial; karena `pyserial` tidak tersedia pada environment host ini, test hardware tersebut tidak dijadikan acceptance host-side. Protocol unit tests tetap 95/95 PASS.

## Belum diverifikasi hardware

Tanpa build/flash pengguna, checkpoint ini **belum** mengklaim PASS untuk:
- RAM/Flash linker aktual;
- ISR cycle aktual setelah perubahan;
- sensorless start/handover pada motor nyata;
- VESC Tool 50 Hz tanpa dropout pada USB-UART nyata;
- tidak munculnya dialog `Parameters truncated` untuk nilai yang memang melampaui limit hardware;
- Hall/encoder detection di motor nyata;
- brown-out tepat saat flash erase/program;
- LED/buzzer fisik dan seluruh mode duty/current/RPM/position/brake/handbrake di bawah beban.

## File utama yang berubah

- `src/terminal.c`
- `src/confgenerator.c`
- `src/confgenerator.h`
- `src/motor/mcpwm_foc.c`
- `src/motor/mc_interface.c`
- `src/comm/commands.c`
- komentar `// Fungsi ...` di source proyek
- `tools/add_function_comments_id.py`
- `tools/audit_run8_contract.py`
- dokumen ini
