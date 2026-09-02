# Audit Checkpoint Run 9 — STM32F103RCT6 VESC Port

Tanggal: 2026-08-30

## Ruang lingkup

Run ini melanjutkan checkpoint Run 8 tanpa menjalankan `pio run`. Fokus audit adalah jalur real-time yang sudah terbukti pada `hoverboard-firmware-hack-FOC_v3_DQ_SIGN_FIX(1).zip`, kompatibilitas VESC firmware 6.00, false `Parameters truncated`, sensorless FOC/SVPWM, dan konsistensi telemetri.

## 1. Referensi hoverboard yang dipertahankan

Jalur current-sense kritis tetap mengikuti referensi yang sudah terbukti bergerak:

- TIM8 TRGO memicu ADC current sampling.
- ADC1 + ADC2 regular simultaneous.
- DMA1 Channel 1, circular, IRQ priority 0.
- Rank 1: ADC1 PC1 = RIGHT DC, ADC2 PC0 = LEFT DC.
- Rank 2: ADC1 PA0 = LEFT phase A, ADC2 PC3 = LEFT phase B.
- Rank 3: ADC1 PC4 = RIGHT phase B, ADC2 PC5 = RIGHT phase C.
- LEFT phase C direkonstruksi `-(A+B)`; RIGHT phase A direkonstruksi `-(B+C)`.
- Polaritas arus `offset - raw`.
- Skala phase/DC current 0.0200 A/count (`A2BIT_CONV=50`).
- Boot current-offset calibration: seed 2000, 2000 frame, update `(raw + offset) / 2`.
- CCR PWM U/V/W tetap CCR1/CCR2/CCR3 dengan preload/UDIS update koheren.

Manual VESC Tool recalibration tetap memiliki pipeline undriven/driven yang lebih aman untuk validasi VESC, tetapi boot path yang timing-critical mengikuti referensi hoverboard di atas.

## 2. ISR cycle accounting diperbaiki

Run 8 masih memiliki beberapa early-return ISR (boot calibration, calibration/fault path) yang dapat keluar sebelum statistik DWT ditutup. Run 9 memusatkan penutupan timing pada `foc_isr_finish_timing()` dan memanggilnya pada seluruh jalur keluar penting.

Definisi diagnostik:

- >85% slot: `near_deadline` saja.
- >4000 cycle: baru `isr_overruns`.
- Tidak ada pembagian floating point di hard ISR untuk mengubah cycle ke detik.

Log referensi yang diberikan pengguna:

| Log | Maksimum cycle | Pemakaian budget 4000 |
|---|---:|---:|
| hover_20260830_020447 | 3359 | 83.98% |
| mode1 | 3179 | 79.48% |
| mode4 | 1409 | 35.23% |
| mode3 | 3519 | 87.98% |

Worst reference = 3519 cycle, headroom = 481 cycle. Ini bukan pengukuran Run 9; Run 9 harus diukur setelah build/flash pengguna.

## 3. False Parameters Truncated — foc_openloop_rpm_low

Ditemukan kesalahan semantik nyata pada checkpoint sebelumnya. Pada VESC 6.00, `foc_openloop_rpm_low` adalah **fraksi 0..1** dari `foc_openloop_rpm` pada current rendah, bukan ERPM absolut.

Perbaikan Run 9:

- default `foc_openloop_rpm_low` dari nilai ERPM salah menjadi `0.0f`;
- decode/validation dibatasi 0..1;
- offset wire tetap VESC 6.00 byte 205, float16 scale 1000;
- `foc_openloop_rpm` tetap byte 201;
- HFI Start Samples tetap byte 265 dan dipertahankan byte-for-byte walaupun backend HFI tidak dijalankan pada F103 ini;
- magic offset 201/205/211..221 diganti macro bernama agar tidak mudah drift.

Full MCCONF 481 byte dan APPCONF 493 byte tetap disimpan sebagai source-of-truth wire image untuk SET/GET/default/persistence. Field UI yang tidak memiliki backend runtime tidak dibuang dari image.

## 4. Sensorless FOC/SVPWM

Pure sensorless tidak memakai Hall/encoder sebagai sumber phase. Jalurnya:

1. forced/open-loop electrical phase;
2. observer flux/BEMF berjalan;
3. open-loop target mengikuti `foc_openloop_rpm` dan `foc_openloop_rpm_low` 0..1;
4. phase-coherence check;
5. blend menuju observer;
6. observer menjadi phase source normal.

Bug sebelumnya yang memaksa target sensorless ke `foc_sl_erpm * 1.10` dihapus. `foc_sl_erpm` digunakan sebagai threshold hybrid sensored/observer, bukan target forced startup pure sensorless.

Sensor policy tetap:

- LEFT: SENSORLESS / HALL / ENCODER.
- RIGHT: SENSORLESS / HALL; encoder ditolak.

Hardware sensorless Run 9 belum boleh dinyatakan PASS sebelum pengujian motor nyata.

## 5. Telemetri current disamakan dengan VESC 6.00

VESC 6.00 mendefinisikan motor current sebagai `SIGN(Vq * Iq) * |I_dq|`, sedangkan directional current adalah Iq. Checkpoint sebelumnya masih menggunakan tanda current input atau tanda Iq pada beberapa jalur.

Run 9 menyamakan:

- `current_motor` RT Data: `SIGN(Vq * Iq_filter) * sqrt(Id_filter^2 + Iq_filter^2)`;
- `current_in`: tetap berasal dari shunt DC;
- directional current: Iq;
- Id/Iq/Vd/Vq/current motor/current input diambil dari satu coherent FOC seqlock snapshot untuk RT Data.

GET_VALUES tetap mengikuti urutan VESC 6.00. Full non-selective payload = 74 byte; frame serial pendek ~79 byte, waktu line-rate pada 115200 sekitar 6.86 ms, sehingga secara bandwidth murni masih masuk interval 20 ms (50 Hz). Paket periodik low-priority menggunakan drop/retry saat UART sibuk agar tidak menahan reply VESC Tool.

## 6. Config, command, detect, EEPROM/thread

Static contract memverifikasi adanya jalur:

- SET_DUTY
- SET_CURRENT
- SET_CURRENT_BRAKE
- SET_RPM
- SET_POS
- SET_HANDBRAKE
- Detect R/L
- Flux linkage
- Hall detect
- Encoder detect LEFT-only
- Detect-All FOC

Persistence masih memakai record CRC + transactional store guard sehingga timer autosave dan blocking configuration worker tidak melakukan erase/program bersamaan.

LED dan buzzer backend tetap ada. Kondisi fisik output tetap perlu tes hardware.

## 7. Clean code

Perubahan clean-code:

- helper konfigurasi `A()` / `F()` sudah menjadi `append_float32_auto_field()` / `append_float16_field()`;
- offset FOC open-loop diberi macro bernama;
- tidak ada `#if 0` pada project-owned source;
- heuristic audit tidak menemukan static function yang didefinisikan tetapi tidak pernah direferensikan;
- 913/913 definisi fungsi project-owned memiliki komentar `// Fungsi ...` Bahasa Indonesia di atas fungsi;
- third-party FreeRTOS vendor tidak dimodifikasi hanya untuk komentar.

## 8. Host checks (tanpa pio run)

- `tools/audit_run8_contract.py`: PASS
- `tools/audit_run9_contract.py`: PASS
- `tools/rtos_audit.py`: PASS
- `tools/comm_audit.py`: PASS
- `tools/debug.py --self-test`: PASS
- `vesc_tools/selftest_protocol.py`: PASS
- `pytest vesc_tools/vesc_protocol/tests`: 95/95 PASS

## 9. File utama berubah dari Run 8

- `src/telemetry.c`
- `src/confgenerator.c`
- `src/confgenerator.h`
- `src/motor/mc_interface.c`
- `src/motor/foc_math.c`
- `src/motor/mcpwm_foc.c`
- `src/applications/appconf_default.h`
- `tools/audit_run9_contract.py`
- `docs/AUDIT_CHECKPOINT_RUN9_20260830.md`

## 10. Belum diverifikasi hardware/build

Karena sesuai instruksi tidak menjalankan `pio run`, Run 9 **tidak mengklaim**:

- RAM/Flash aktual hasil linker;
- ISR max-cycle aktual Run 9;
- RT Data fisik 50 Hz tanpa drop pada USB-UART nyata;
- sensorless/Hall/encoder-left motor nyata;
- rotor sign/offset/position setelah beban mekanik;
- brown-out tepat saat flash erase/program;
- LED/buzzer fisik;
- seluruh duty/current/speed/position/brake/handbrake di bawah beban;
- tidak ada fault hardware dalam semua kondisi.

Gunakan hasil build/flash dan log pengguna sebagai acceptance gate run berikutnya.
