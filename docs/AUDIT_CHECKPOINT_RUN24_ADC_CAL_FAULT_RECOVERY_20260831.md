# Audit Checkpoint Run24 — ADC, Calibration, Telemetry, Buzzer, Fault Recovery

Tanggal: 2026-08-31

## Acceptance gate dari log hardware pengguna

Hall detect sebelumnya gagal dengan `ABS_OVER_CURRENT` karena raw phase current ketika bridge aktif berada sekitar 2047/2065 count, tetapi offset runtime yang dipakai sudah bergeser ke sekitar 2817/2833 count. Dengan polaritas board `offset - raw`, selisih ini menghasilkan arus semu besar dan phase hasil rekonstruksi ikut melampaui ambang fault. Run24 menghapus sumber offset bridge-OFF tersebut dari pipeline kalibrasi.

## Jalur ADC/current yang dikunci

Tiga dual-rank pertama mengikuti pemetaan firmware hoverboard referensi terbaru:

1. rank 1: ADC1 PC1 / CH11 = RIGHT DC, ADC2 PC0 / CH10 = LEFT DC;
2. rank 2: ADC1 PA0 / CH0 = LEFT A, ADC2 PC3 / CH13 = LEFT B;
3. rank 3: ADC1 PC4 / CH14 = RIGHT B, ADC2 PC5 / CH15 = RIGHT C.

Auxiliary juga disusun mengikuti urutan reference:

4. rank 4: ADC1 PC2 / CH12 = VBAT diagnostic, ADC2 PA2 / CH2 = APP ADC1;
5. rank 5: ADC1 internal temperature = board-temperature proxy, ADC2 PA3 / CH3 = APP ADC2;
6. rank 6: filler cepat eksplisit dan tidak digunakan oleh FOC maupun APP ADC.

Polarity current tetap `offset - raw`, dengan 50 count/A atau 0.020 A/count.

### Perbedaan yang disengaja terhadap reference

Reference sederhana menggunakan lima rank, ADC clock /4, dan DMA transfer-complete di akhir sequence. Port VESC dual-motor ini **tidak menyalin bagian tersebut secara buta** karena ISR dual FOC hardware pengguna pernah membutuhkan lebih dari 2300 CPU cycle. Menunggu rank temperature sebelum memulai FOC pada ADC /6 akan memperkecil deadline secara berbahaya.

Run24 menggunakan ADC /6 dan DMA half-transfer setelah rank 3. Dengan demikian enam kanal current sudah coherent sebelum FOC dimulai, sementara PC2/PA2/PA3/temperature selesai sesudah boundary tersebut. Ini mempertahankan urutan sensor reference tetapi memberi margin ISR yang jauh lebih aman. ADC3 tetap membaca PC2/DCLINK secara independen pada TIM8 TRGO yang sama untuk freshness Vbus.

## Kalibrasi Run24

Boot calibration dan recalibration manual sekarang menggunakan **satu algoritma yang sama**:

1. tunggu Vbus berada pada range valid dan stabil selama 2 s;
2. set LEFT dan RIGHT ke 50%/50%/50% zero-vector;
3. tunggu MOE kedua bridge benar-benar terkonfirmasi;
4. warm-up 64 frame ADC;
5. ambil 2000 frame kontinu untuk keenam kanal current;
6. simpan min dan max masing-masing kanal;
7. offset final = `(min + max) / 2`;
8. validasi rail/noise;
9. matikan PWM dan tandai calibration DONE/VALID.

Tidak ada lagi sumber offset “undriven” yang kemudian dipakai saat analog front-end berada pada common-mode PWM aktif. Field diagnostik `undriven/driven` lama tetap dipertahankan untuk ABI debug, tetapi keduanya berisi midpoint yang sama. Diagnostic revision menjadi 19 dan target progress menjadi 2000.

## APP ADC dan temperature

Bug sebelumnya: fungsi capture PA2/PA3 sudah ada, tetapi tidak pernah dipanggil dari jalur DMA ISR, sehingga APP ADC dapat terus terlihat nol. Run24 memanggil capture auxiliary pada DMA HT. Karena rank 4/5 frame saat ini belum selesai pada titik HT, yang dilatch adalah rank 4/5 frame PWM sebelumnya yang sudah lengkap dan stabil:

- PA2 = ADC2 rank-4 high16;
- PA3 = ADC2 rank-5 high16;
- temperature = ADC1 rank-5 low16.

Tidak ada interrupt ADC auxiliary tambahan.

## ISR/timing

`DMA1_Channel1_IRQHandler()` mengambil DWT cycle pada instruksi awal handler, sehingga jalur normal maupun transfer-error tercatat end-to-end. Pada HT:

1. clear HT flag;
2. latch beberapa word auxiliary dari frame sebelumnya;
3. jalankan `foc_adc_dma_isr_timed()` menggunakan tiga word current frame sekarang;
4. semua early return menutup statistik timing.

Tidak ada printf, UART, flash, atau RTOS call blocking di hard FOC ISR.

## Power-on melody dan fault buzzer

Power-on melody sekitar 3.21 s **tetap aktif** dan dimulai sebelum FreeRTOS. Saat calibration/detect/running/idle tidak ada cue buzzer tambahan; status tersebut hanya memakai LED.

Jika fault runtime aktif, startup melody diputus dan fault dengan prioritas tertinggi menghasilkan pola buzzer berdasarkan kode fault VESC (digit puluhan lalu satuan). Command buzzer diagnostik tidak diizinkan memulai melody/beep normal; action stop tetap tersedia untuk emergency diagnostic.

## Fault recovery timer

`MotorRuntime` sekarang menyimpan:

- `last_fault` untuk histori diagnosis;
- `fault_recovery_ticks` sebagai counter 1 ms.

Fault software yang recoverable baru di-clear setelah **1000 ms kondisi sehat kontinu**, motor stopped, command inactive, dan tidak ada hardware power-stage latch. Contoh: over-current setelah current kembali aman, bus over/undervoltage setelah kembali dalam range, Hall invalid setelah Hall valid stabil, sensor-detect selesai, sensorless observer kembali ke stopped state, dan current-offset setelah kalibrasi valid.

Fault hardware/critical seperti MCU undervoltage PVD, BREAK/BKIN dan flash corruption tetap tidak di-clear secara buta. Fault ADC-DMA/FOC-overrun/thermal/encoder-slip juga tidak di-auto-clear tanpa bukti sehat khusus. Jika kondisi recoverable kembali buruk selama hold, counter kembali 0.

Terminal `faults` menampilkan active VESC fault, last fault, dan `recover_ms/1000`.

## Telemetry standar VESC

Handler standar tetap request-response dan tidak bergantung pada state motor:

- `COMM_GET_VALUES`;
- `COMM_GET_VALUES_SELECTIVE`;
- `COMM_GET_VALUES_SETUP`;
- `COMM_GET_VALUES_SETUP_SELECTIVE`;
- `COMM_GET_DECODED_ADC`;
- konfigurasi/firmware/terminal standar terkait.

Saat idle, calibration selesai, running, atau fault, request GET_VALUES tetap dibalas selama USART3 transport hidup. Fault field melaporkan active VESC fault; setelah recovery valid menjadi NONE, sedangkan `last_fault` tetap tersedia pada terminal debug.

`tools/debug.py stream-all` default:

- RT Data `COMM_GET_VALUES_SELECTIVE`: 50 Hz;
- App Data `COMM_GET_DECODED_ADC`: 20 Hz;
- raw/internal diagnostics dipoll lebih lambat agar USART3 115200 tidak jenuh.

Motor RIGHT/controller ID 2 tetap menggunakan forwarding VESC `COMM_FORWARD_CAN` dan payload standard yang sama.

## File source yang berubah terhadap Run23

- `src/applications/appconf_default.h`
- `src/comm/commands.c`
- `src/datatypes.h`
- `src/hwconf/hw.c`
- `src/hwconf/hw_status.c`
- `src/motor/mc_interface.c`
- `src/motor/mcpwm_foc.c`
- `src/stm32f1xx_it.c`
- `src/terminal.c`
- `tools/debug.py`

Cleanup format non-logical juga dilakukan pada beberapa one-line control lama di `app_adc.c`, `commands.c`, `foc_math.c`, `hw.c`, `mc_interface.c`, dan `mcpwm_foc.c`.

## Verifikasi host/static yang dilakukan

- `tools/debug.py --self-test`: PASS;
- py_compile `vesc.py`/`debug.py`: PASS;
- host `-Wall -Wextra -Werror -fsyntax-only`: `buffer.c`, `maths.c`, `packet.c`, `foc_math.c`: PASS;
- current/ADC/register/calibration/telemetry/buzzer/fault-recovery contract checks: 17/17 PASS;
- function comment scan: 870 definisi terdeteksi, missing comment 0;
- obvious unreferenced static function: 0;
- `#if 0`: 0;
- broken shift operator pattern: 0;
- one-line control/case-body format gate: 0;
- tab/trailing whitespace C/H/Python: 0;
- Python di `/tools`: tepat `vesc.py` dan `debug.py`;
- `vesc_tools/`: tidak ada.

## Yang belum boleh diklaim PASS

Sesuai instruksi, `pio run` tidak dijalankan. Karena itu berikut masih memerlukan build/flash/log pengguna:

- compile/link keseluruhan STM32 dan RAM/Flash aktual;
- `FOC one_max/dual_max < 4000` pada binary Run24;
- offset Run24 benar-benar mengikuti raw zero-vector sekitar midpoint hardware;
- Hall detect tidak lagi memicu fake `ABS_OVER_CURRENT`;
- Sensorless/Hall motion nyata LEFT/RIGHT;
- RT 50 Hz dan App 20 Hz tanpa dropout pada USART3 hardware;
- power-on melody/fault-code buzzer fisik;
- fault recoverable benar-benar auto-clear setelah 1 s sehat dan hard fault tetap latched.
