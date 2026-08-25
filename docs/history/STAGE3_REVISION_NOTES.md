# STAGE 3 / PART 3 — Production Hardening, Sampling Contract, WCET/RAM/RTOS, UART Cleanup

## 1. Temuan

Stage2 sudah memiliki FOC fixed-point, sensor Hall/Encoder/Sensorless, APP ADC PA2/PA3, USART3 VESC transport, auto-detection, persistence, fault, dan watchdog. Bagian yang masih perlu ditutup untuk Part 3 adalah pembuktian runtime terhadap kontrak ADC/PWM, headroom waktu ISR dan SRAM/stack, UART TX yang masih dapat menunggu ketika queue penuh, serta beberapa API kompatibilitas VESC yang tidak pernah digunakan.

Topologi STM32F103 hoverboard ini mempunyai satu frame ADC1/ADC2 dual-regular yang koheren untuk kedua motor. Rank 1–3 adalah fast-current sample dan DMA half-transfer menjalankan FOC 16 kHz. Karena itu mode VESC modern V0/V7 alternating sample atau high-current re-sampling TIDAK boleh diaktifkan hanya karena field MCCONF tersedia. Implementasi Part 3 mempertahankan rejection terhadap mode tersebut sampai oscilloscope/current-probe validation membuktikan topologi baru aman.

## 2. Root Cause / Prinsip Desain

- SRAM target sangat ketat, sehingga task/buffer baru harus dihindari.
- `app_uartcomm_write_raw()` sebelumnya dapat menunggu queue sampai ratusan ms. Ini tidak sesuai target communication task non-blocking.
- Pengukuran per-motor ISR saja tidak cukup untuk membuktikan deadline: dua motor diproses pada satu coherent ADC frame, sehingga perlu mengukur total dual-motor ISR dan period jitter.
- Konfigurasi register TIM/ADC/DMA dapat secara teoritis berubah/terkorupsi tanpa terdeteksi sampai motor sudah dinyatakan ready. Diperlukan runtime hardware sampling contract sebelum ready.
- Beberapa audio/sample compatibility API tidak mempunyai caller pada firmware ini dan hanya mengembalikan unsupported. Karena startup/fault melody memang menggunakan buzzer StatusIO, API yatim tersebut dihapus sebagai dead code.

## 3. Referensi VESC / FreeRTOS yang Diadopsi Secara Selektif

- VESC modern memiliki beberapa FOC control/current sample modes dan overmodulation. Behavior tersebut digunakan sebagai referensi, tetapi tidak dipaksakan ke shared-ADC hoverboard topology tanpa pembuktian hardware.
- FreeRTOS `uxTaskGetStackHighWaterMark()` serta `xPortGetMinimumEverFreeHeapSize()` digunakan untuk mengukur headroom runtime dan menghindari tuning stack berdasarkan tebakan.
- FOC hard path tetap fixed-point dan tidak diberi logging/RTOS call baru.

## 4. Perubahan yang Dilakukan

### 4.1 Runtime ADC/PWM Sampling Contract
File: `src/hwconf/hw.c`, `src/hwconf/hw.h`, `src/main.c`

Ditambahkan pemeriksaan sebelum controller dinyatakan motor-ready:
- TIM1 center-aligned + ARR expected.
- TIM8 center-aligned + ARR expected.
- TIM8 TRGO = UPDATE.
- TIM8 repetition counter = 1.
- ADC1 regular sequence length = 6.
- ADC2 regular sequence length = 6.
- DMA1 Channel1 = memory increment + circular + very-high priority.

Jika kontrak gagal:
- power stage emergency off,
- kedua motor mendapat `MOTOR_FAULT_ADC_DMA`,
- motor-ready tidak dipublikasikan,
- USART management tetap dapat hidup untuk diagnostic/recovery.

### 4.2 FOC ISR WCET + Period Jitter
File: `src/motor/mcpwm_foc.c/.h`

Ditambahkan:
- total dual-motor ISR maximum cycles,
- near-deadline counter (>85% dari slot),
- min/max ISR entry period,
- tetap mempertahankan per-motor `isr_max_cycles` yang sudah ada.

Clock firmware aktual = HSI/2 × 16 = 64 MHz. Pada FOC 16 kHz, satu slot = 4000 cycles = 62.5 us. Tidak ada float/logging/packet parsing yang ditambahkan ke ISR.

### 4.3 RTOS Stack + Heap Watermark
File: `src/motor_tasks.c/.h`, `src/comm/commands.c/.h`, `src/terminal.c`

Ditambahkan runtime telemetry:
- current free FreeRTOS heap,
- minimum-ever free heap,
- stack high-water/free bytes untuk motor service,
- sample sender,
- fault thread,
- StatusIO,
- VESC packet thread,
- blocking-worker thread.

Terminal baru/ditingkatkan:
- `timing`
- `resources`

### 4.4 UART TX Final Hardening + SRAM Reduction
File: `src/applications/app_uartcomm.c/.h`

Perubahan:
- TX queue depth 6 → 4.
- queue-full tidak lagi menunggu dengan `osDelay()` sampai 250 ms.
- mutex acquire menjadi immediate/non-blocking.
- full/busy menghasilkan `false` backpressure dan counter diagnostic.
- ditambahkan TX queue high-water mark.
- ditambahkan TX queue busy-drop count.

Satu `tx_slot_t` = 2 byte length + 520-byte frame. Pengurangan dua slot menghemat sekitar 1044 byte static SRAM sebelum alignment/linker effects.

### 4.5 COMM_DIAG Revision 13
File: `src/comm/commands.c`, `tools/debug.py`

Buffer diperbesar 224 byte dan revision naik menjadi 13. Field tambahan:
- sampling contract flags,
- total ISR max cycles,
- near-deadline count,
- min/max ISR period,
- heap current/min-ever,
- enam stack free-watermark,
- TX queue high-water,
- TX busy-drop.

### 4.6 Dead-Code Cleanup
Dihapus karena tidak mempunyai caller dan tidak relevan dengan target buzzer-melody firmware:
- `mcpwm_foc_set_audio_sample_table()`
- `mcpwm_foc_get_audio_sample_table()`
- `mcpwm_foc_play_audio_samples()`
- VESC motor-audio wrapper yang tidak digunakan
- unused ADC injected/timer-sample compatibility hook.

StatusIO buzzer startup melody, LED/buzzer fault code, dan power-stage behavior tidak diubah.

### 4.7 Firmware Identity
`vesc-f103-hoverboard-v31-stage3-production`

Wire protocol tetap VESC 6.00 untuk menjaga kompatibilitas schema yang sudah diuji.

## 5. Fitur yang Sengaja TIDAK Diaktifkan

Part 3 TIDAK memalsukan dukungan:
- V0/V7 alternating current sample,
- V0/V7 interpolated sample,
- high-current resampling,
- full VESC overmodulation di luar current-sense window yang sudah aman.

`foc_sample_v0_v7` dan `foc_sample_high_current` tetap ditolak oleh config validation. Ini keputusan safety, bukan fitur yang terlewat.

## 6. Verification

### VERIFIED BY SOURCE AUDIT
- High-side active HIGH / complementary low active LOW tetap.
- TIM8 LEFT / TIM1 RIGHT tetap.
- fast-current ADC ranks tidak diubah.
- unsupported dynamic sample modes tetap ditolak.
- UART TX tidak memiliki queue-full delay.
- excluded subsystems tidak kembali.
- dead audio/injected-sample compatibility stubs yang tidak dipakai sudah dihapus.

### VERIFIED BY SOFTWARE TEST
- 25/25 project regression scripts PASS.
- Stage3 production-hardening test PASS.
- debug parser COMM_DIAG v13 self-test PASS.
- Python tools compileall PASS.
- tidak ditemukan TODO/FIXME/HACK pada `src/`.

### VERIFIED BY BUILD
Belum dapat diklaim di environment audit ini karena `pio` dan `arm-none-eabi-gcc` tidak tersedia. Build target harus dilakukan pada PC pengguna.

### REQUIRES HARDWARE TEST
- Gate polarity/dead-time dengan oscilloscope.
- ADC current sample position terhadap switching edges.
- `timing`: dual ISR WCET/jitter pada kedua motor aktif.
- `resources`: stack/heap watermark setelah stress VESC Tool + dual motor + APP ADC.
- UART TX drop/HWM pada telemetry stress.
- current waveform dan FOC stability pada high duty.

## 7. Acceptance Gate Hardware

Rekomendasi konservatif sebelum membuka fitur sampling/overmodulation lanjutan:
- `sampling_contract_flags = 0`.
- `dual ISR max < 4000 cycles`; target normal < 3400 cycles (85% slot).
- near-deadline counter tidak terus bertambah pada operasi normal.
- ISR entry period berkisar dekat 4000 cycles tanpa outlier berulang.
- setiap critical task masih memiliki stack reserve nyata; target development >256 byte bila memungkinkan, minimum jangan mendekati nol.
- minimum-ever free heap tetap mempunyai reserve; target >2 KiB sesuai readiness reserve firmware.
- TX queue busy/full drops = 0 pada pemakaian VESC Tool normal.

## 8. Remaining Issues / Hardware-Dependent Backlog

- Dynamic V0/V7/high-current sampling hanya boleh dirancang setelah scope trace membuktikan titik sample yang valid untuk kedua inverter.
- Full overmodulation baru boleh dinaikkan setelah current reconstruction tetap valid pada high duty.
- Nilai stack dapat dikecilkan hanya setelah watermark hardware cukup lama, bukan berdasarkan host audit.
- ARM target build/size Stage3 harus dibuktikan pada PlatformIO pengguna.

## 9. Next Step

1. `pio run -t clean && pio run`.
2. Catat RAM dan Flash.
3. Flash dengan supply/current limit aman.
4. Jalankan terminal `timing`, `resources`, `appadc`, `integrity` saat idle dan saat kedua motor berjalan.
5. Lakukan scope test PWM/ADC sebelum mempertimbangkan dynamic sampling/overmodulation.
