# Run31 Final Package — 2026-09-02

Checkpoint ini adalah hasil finalisasi Run31 setelah fault hardware Run30 (`FAULT_CODE_DRV=3`) dan popup VESC Tool `HFI Start Samples 2 truncated`.

Perubahan yang menjadi bagian paket ini:

- default build hardware `stm32f103rc` menggunakan `-O3`; build debug bersimbol juga tetap `-O3`, bukan `-Og`;
- current sampling mengikuti mapping hoverboard V13: ADC1+ADC2 dual regular simultaneous, DMA1 Channel1, enam word dengan boundary HT setelah tiga rank current;
- ADC3/DMA2 tidak lagi menjadi jalur VBUS/safety aktif;
- trigger motor menggunakan TIM8_TRGO dan audit `TRIGGER_ROUTE` hanya memeriksa route yang memang aktif;
- konfigurasi HFI legacy `foc_hfi_start_samples < 5` dinormalisasi menjadi 5, sehingga image lama bernilai 2 tidak lagi dikirim kembali ke VESC Tool;
- native fault diagnostic membedakan `ADC_DMA` dengan `FOC_ISR_OVERRUN`, walaupun keduanya tetap dipetakan ke VESC `FAULT_CODE_DRV` pada wire protocol;
- Detect-All mencakup R, L, Ld-Lq, flux linkage, sensor discovery, apply dan persistence;
- Hall autodetect memakai forced electrical phase + Id, Iq=0, forward/reverse sweep;
- command acceptance mencakup duty, current, current-relative, RPM, brake, position, handbrake dan EEPROM save;
- power-on melody tetap aktif; running normal hanya 3 pulse LED tanpa buzzer; setiap transaksi EEPROM/flash sukses menghasilkan 5 beep.

Verifikasi offline dari source yang sama dengan ZIP:

- `python3 tools/debug.py --self-test` — PASS
- `python3 tools/audit_run29.py` — 25/25 PASS
- `python3 tools/audit_run30.py` — 41/41 PASS
- `python3 tools/audit_run31.py` — 71/71 PASS
- Python compile — PASS
- portable utility C GCC `-Werror` — PASS
- firmware hoverboard V13 reference `tools/run_all_checks.py` — `ALL_FINAL_HOST_CHECKS_PASS`

Batas verifikasi: environment packaging tidak memiliki `pio` atau `arm-none-eabi-gcc`, sehingga cross-build STM32 dan pembuktian motor fisik dilakukan pada board pengguna. Gunakan `docs/RUN31_HARDWARE_ACCEPTANCE_COMMANDS.txt` dan jangan melewati tahap yang fault.
