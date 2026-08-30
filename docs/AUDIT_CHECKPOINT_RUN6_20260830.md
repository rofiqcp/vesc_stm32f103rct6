# AUDIT CHECKPOINT RUN 6 — VESC STM32F103RCT6

Tanggal audit: 30 Agustus 2026  
Baseline resource dari pengguna: RAM 41,912 / 49,152 B (85.3%), Flash 157,276 / 253,952 B (61.9%).  
Catatan penting: **tidak menjalankan `pio run`**. Seluruh status di bawah adalah audit statis/host-test kecuali data cycle yang berasal dari log hardware pengguna.

## 1. Ringkasan perubahan run ini

### 1.1 Telemetry 100 Hz dibuat non-blocking
File: `src/telemetry.c`, `src/telemetry.h`

Masalah yang ditemukan:
- Cache telemetry 100 Hz memakai mutex. Writer timer dapat menunggu mutex dan reader juga dapat menunggu hingga 5 ms.
- Read-reset average memakai critical section untuk operasi accumulator. Pada Cortex-M3 FreeRTOS, critical section memakai BASEPRI sehingga ISR prioritas 0 tetap dapat preempt, tetapi interrupt RTOS-compatible dan scheduling tetap tertahan selama bagian tersebut.
- `sqrtf()` dan konversi FOC dikerjakan dekat area sinkronisasi accumulator sehingga memperbesar jitter task-side.

Perbaikan:
- Menghapus mutex telemetry dan alokasi heap-nya.
- Cache `motor_telemetry_t` dipublikasikan dengan seqlock `s_telem_seq[2]`.
- `telemetry_get()` membaca cache lock-free dan fallback ke snapshot realtime bila sequence sedang berubah.
- `Id/Iq/Vd/Vq/Iin/I_motor` tetap diambil dari satu `foc_rt_snapshot_t` yang koheren.
- Semua `sqrtf()`/konversi average dilakukan sebelum sinkronisasi accumulator.
- Accumulator task-to-task memakai `vTaskSuspendAll()/xTaskResumeAll()`, sehingga ADC/DMA IRQ prioritas tinggi tetap aktif.
- Tidak menambah `motor_telemetry_t` besar ke stack timer 1 KiB; cache ditulis langsung di bawah seqlock.

Tujuan: polling VESC Tool 50 Hz tidak membuat timer task menunggu telemetry lock, sementara nilai dq/current tetap berasal dari frame FOC yang konsisten.

### 1.2 Paket periodik tidak boleh memblokir timer
File: `src/comm/commands.c`

Masalah:
- `vesc_comm_send_payload_class()` selalu menunggu `s_send_mutex` dengan `portMAX_DELAY`, termasuk paket low-priority seperti rotor-position periodik.

Perbaikan:
- Reply request/response standar tetap boleh menunggu mutex.
- Paket low-priority memakai timeout 0. Bila UART sedang melayani RT Data/config reply, paket periodik dilewatkan dan dapat dicoba pada slot berikutnya.

Tujuan: RT Data dan timer 1 kHz/100 Hz tidak tertahan oleh stream diagnostik tambahan.

### 1.3 Blocking command queue diperkecil
File: `src/comm/commands.c`

Masalah:
- `blocking_job_t` berukuran sekitar 516 byte.
- Sebelumnya ada dua static job (`submit` dan `worker`) dan Queue FreeRTOS juga menyimpan satu copy job penuh.

Perbaikan:
- Hanya satu `s_block_job` static.
- Queue FreeRTOS hanya membawa token `uint8_t`.
- `s_block_busy` melindungi buffer sampai detect/config transaction selesai.
- Producer berikutnya ditolak bila worker masih sibuk, sehingga payload tidak dapat ditimpa.

Dampak resource yang dapat diperkirakan tanpa build:
- BSS berkurang sekitar satu `blocking_job_t` (~516 B), dikurangi beberapa byte flag/sequence baru.
- Runtime FreeRTOS heap headroom bertambah lagi karena queue tidak lagi menyimpan item ~516 B dan mutex telemetry dihapus.
- `configTOTAL_HEAP_SIZE` tetap 18 KiB, jadi peningkatan heap headroom tidak otomatis terlihat seluruhnya pada angka RAM linker. RAM/Flash aktual harus diverifikasi oleh build pengguna.

## 2. Audit ISR / ADC / timer

### 2.1 Data log hardware pengguna
Empat CSV pada `logs.zip` dibaca:

| Log | Mean cycle | P99 | Max | Max dari budget 4000 |
|---|---:|---:|---:|---:|
| `hover_20260830_020910_mode1.csv` | 2872.21 | 3178 | 3179 | 79.48% |
| `hover_20260830_020447.csv` | 2839.24 | 2972 | 3359 | 83.98% |
| `hover_20260830_020925_mode4.csv` | 1346.10 | 1352 | 1409 | 35.23% |
| `hover_20260830_020941_mode3.csv` | 3188.81 | 3498 | 3519 | 87.98% |

Max yang terlihat = **3519 cycle**, masih 481 cycle di bawah budget 4000. Ini bukan bukti checkpoint baru pasti tetap di bawah 4000; angka tersebut adalah hasil firmware/log yang diberikan pengguna.

### 2.2 Definisi overrun
`src/motor/mcpwm_foc.c` diperiksa:
- >85% slot hanya menaikkan `s_isr_near_deadline_count`.
- `isr_overruns` hanya naik saat `cycles > FOC_ISR_SLOT_CYCLES`.
- fault `MOTOR_FAULT_FOC_ISR_OVERRUN` memerlukan delapan overrun nyata berurutan.
- hard ISR menyimpan raw cycle; konversi ke detik dilakukan getter task-side.

### 2.3 ADC/current mapping vs firmware hoverboard referensi
Mapping current DMA dipertahankan karena cocok dengan `hoverboard-firmware-hack-FOC_v3_DQ_SIGN_FIX`:
- word0 low = RIGHT DC (PC1), high = LEFT DC (PC0)
- word1 low = LEFT phase A (PA0), high = LEFT phase B (PC3)
- word2 low = RIGHT phase B (PC4), high = RIGHT phase C (PC5)
- current sign runtime = `offset - raw`, sesuai referensi yang telah terbukti bergerak.

Arsitektur yang dipertahankan:
- PWM 16 kHz.
- TIM8 TRGO sebagai trigger ADC current.
- ADC1/ADC2 dual regular simultaneous.
- DMA1 Channel 1 current frame.
- FOC dipanggil dari DMA current ISR, bukan thread.
- ADC3/DMA2 dipakai terpisah untuk VBUS.

Karena pengguna menyatakan pembacaan arus sudah benar dan motor telah bergerak, pin/rank/sign utama tidak diubah tanpa bukti baru.

## 3. Audit RT Data / telemetry VESC

`COMM_GET_VALUES`/selective diperiksa untuk:
- current motor
- current input/battery
- Id filter
- Iq filter
- duty
- ERPM
- VBUS
- Ah/Wh charge/discharge
- tachometer / tachometer_abs
- fault
- position
- controller ID
- Vd
- Vq
- status

Nilai current/Id/Iq/Vd/Vq memakai read-reset average VESC; fallback memakai satu FOC seqlock frame. Snapshot realtime juga memuat phase A/B/C, raw Id/Iq, filter, rotor electrical phase, observer, offsets dan diagnostik custom.

Pada 115200 baud, response RT Data standar masih jauh lebih kecil dari payload max 512 byte. Tidak ada auto-push `COMM_GET_VALUES`; VESC Tool melakukan request/reply. Paket rotor-position periodik sekarang tidak boleh menahan reply tersebut.

**Belum diverifikasi hardware:** 50 Hz kontinu tanpa dropped request setelah checkpoint ini diflash.

## 4. Audit MCCONF/APPCONF dan VESC Tool

Ukuran wire:
- MCCONF = 481 byte; command + wire = 482 byte.
- APPCONF = 493 byte; command + wire = 494 byte.
- `VESC_PACKET_MAX_PAYLOAD` = 512 byte.

Semua masih muat tanpa truncation buffer.

Jalur yang diperiksa:
- `COMM_GET_MCCONF`
- `COMM_GET_MCCONF_DEFAULT`
- `COMM_SET_MCCONF`
- `COMM_GET_APPCONF`
- `COMM_GET_APPCONF_DEFAULT`
- `COMM_SET_APPCONF`
- `COMM_SET_APPCONF_NO_STORE`
- MCCONF temporary/setup
- battery cut read/write

Kebijakan config:
- Wire image lengkap VESC 6.00 dipertahankan untuk round-trip.
- Field yang tidak dipakai backend STM32F103 tidak dibuang hanya karena backend tidak mengeksekusinya.
- Field hardware-critical yang benar-benar dibatasi board di-clamp dan nilai hasil clamp ditulis kembali ke canonical wire image.
- LEFT dapat sensorless/Hall/encoder A/B.
- RIGHT menolak encoder dan hanya menerima sensorless/Hall.
- HFI ditolak karena backend ini memang tidak mengimplementasikannya.

Upstream VESC saat SET config juga mendeserialisasi generated config, menerapkan hardware limits, menyimpan, lalu mengaktifkan runtime config. Port ini mengikuti model tersebut sejauh relevan untuk hardware F103 dual motor.

## 5. Audit EEPROM / flash persistence

Region config:
- `0x0803E000 .. 0x0803FFFF`
- empat page × 2048 byte
- linker membatasi firmware Flash ke 248 KiB sehingga 8 KiB akhir tidak tertimpa image program.

Transactional record:
- config record < 1 page.
- CRC32 payload.
- sequence number.
- target page berikutnya di-erase, payload/header ditulis, kemudian magic ditulis terakhir sebagai commit marker.
- page committed lama tidak dihapus terlebih dahulu.
- bila write/verify baru gagal, `best_page()` mencari committed fallback lama dan runtime config write dikembalikan/dianggap gagal tanpa menyatakan page lama rusak.
- flash hanya ditulis saat motor dihentikan; current DMA IRQ dinonaktifkan selama operasi flash karena code fetch STM32F1 juga stall ketika flash diprogram.
- auto odometer persistence wear-aware (jarak 1 km) dan ditunda bila motor sedang aktif.

**Belum diverifikasi hardware:** brown-out pada tiap titik erase/program/commit dan endurance jangka panjang.

## 6. Audit command motor

Static command coverage lulus untuk:
- `COMM_SET_DUTY`
- `COMM_SET_CURRENT`
- `COMM_SET_CURRENT_BRAKE`
- `COMM_SET_RPM`
- `COMM_SET_POS`
- `COMM_SET_HANDBRAKE`
- `COMM_SET_CURRENT_REL`
- `COMM_ALIVE`
- `COMM_MOTOR_ESTOP`
- `COMM_SHUTDOWN`
- `COMM_REBOOT`

Setter masuk melalui arbitration/timeout dan backend FOC (`mcpwm_foc_set_*_motor`). Position LEFT encoder A/B mempertahankan extended coordinate; Hall/sensorless memakai circular position semantics.

**Belum diverifikasi hardware:** arah torsi/brake/handbrake dan tuning PI/PID pada beban nyata.

## 7. Audit detection / sensor

Static coverage:
- R/L detect
- flux linkage detect
- open-loop flux detect
- Hall detect
- encoder detect
- Detect-All FOC
- custom AUTO sensor detect

Encoder detector memiliki guard `m->id != MOTOR_LEFT` -> gagal aman. RIGHT tidak dapat menyimpan MCCONF encoder.

Detect/config dijalankan di blocking worker, bukan packet parser atau ISR. Detect-All memiliki rollback runtime/config bila salah satu local motor gagal sebelum atomic dual commit.

**Belum diverifikasi hardware:** hasil angka R/L/flux, Hall table, encoder ratio/offset/inversion dan sensorless observer pada motor aktual.

## 8. Audit LED / buzzer

Pin dibandingkan ke firmware hoverboard referensi dan cocok:
- LED = PB2
- buzzer = PA4
- power hold = PA5

Implementasi:
- early boot LED/power-hold.
- TIM3 buzzer IRQ priority 8.
- startup melody non-blocking.
- bounded tone memiliki self-stop guard.
- LED heartbeat idle dan cue 1/2/3 pulse untuk calibration/detect/run.
- fault LED/buzzer sequence memprioritaskan fault tertinggi.

**Belum diverifikasi hardware:** polaritas LED pada varian PCB pengguna dan volume/frekuensi buzzer fisik.

## 9. Thread/latency audit

`tools/rtos_audit.py` melaporkan 5 application tasks:
1. fault_stop priority 6
2. timer priority 5
3. packet_process priority 5
4. blocking priority 5
5. sample_send priority 4

FOC tetap ISR-driven. Tidak ada CMSIS-RTOS2 wrapper.

Run ini mengurangi dua sumber task jitter:
- telemetry cache mutex dihapus.
- low-priority UART stream tidak lagi menunggu outer send mutex.

Single-job blocking queue juga menambah runtime heap headroom.

## 10. Host/static verification yang dijalankan

Tanpa `pio run`:
- `python3 tools/rtos_audit.py` -> PASS
- `python3 tools/comm_audit.py` -> PASS
- `python3 tools/debug.py --self-test` -> PASS
- `python3 vesc_tools/selftest_protocol.py` -> PASS
- `python3 -m pytest -q vesc_tools/vesc_protocol/tests` -> **95 passed**
- custom static assertions -> PASS untuk telemetry lock removal, scheduler serialization, queue token, single blocking job, low-priority nonblocking, ISR threshold, ADC rank mapping, command setter coverage, detect coverage, left-only encoder, LED/buzzer mapping, transactional config fallback, dan packet-size margin.

## 11. Hal yang sengaja TIDAK diklaim PASS

Karena pengguna meminta tidak menjalankan build dan hardware tidak tersedia pada audit ini, checkpoint ini **tidak mengklaim**:
- compile/link PASS
- RAM/Flash aktual
- max ISR cycle firmware baru
- 50 Hz RT Data fisik tanpa dropout
- zero-fault operation pada motor nyata
- correctness rotor angle/offset di motor aktual
- Hall/encoder/sensorless detect aktual
- auto-detect dan calibration aktual
- brown-out persistence aktual
- duty/current/speed/position/brake/handbrake aktual di bawah beban

Semua poin itu harus ditutup menggunakan build/log/hardware feedback pengguna pada checkpoint berikutnya.
