# Audit Checkpoint Run 7 — STM32F103RCT6 VESC

## Scope
Run ini melanjutkan Run 6 tanpa menjalankan `pio run`. Audit mencakup semua prioritas pengguna: ISR FOC/ADC/timer, telemetry/RT data, MCCONF/APPCONF, persistence flash, thread/locking, LED/buzzer, sensor modes, rotor position, command motor, auto-detect, calibration, fault handling, dan kompatibilitas pola upstream VESC.

## Perubahan kode
### `src/conf_general.c`
1. Menambah `s_store_busy` + `store_lock_try()/store_unlock()` untuk menserialisasi seluruh transaksi flash konfigurasi.
2. Guard mencakup `conf_general_store_all`, persistent MCCONF, persistent APPCONF, dan auto-persist auxiliary/odometer.
3. Menambah `s_aux_retry_div`: auto-save yang gagal/busy tidak lagi mencoba erase/program setiap tick 100 Hz; retry dibatasi sekitar 1 Hz.

### Bug yang diperbaiki
Sebelumnya blocking worker dapat menyimpan MCCONF/APPCONF pada saat timer 100 Hz juga memulai auto-save odometer. Keduanya menggunakan `s_stage` statis yang sama dan `store_stage()` melakukan delay 5 ms sebelum erase/program. Tanpa guard, task lain dapat mengganti staging record atau masuk flash transaction bersamaan. Ini berisiko record campuran, write gagal, dan wear berlebihan. Selain itu auto-save gagal sebelumnya dapat dicoba ulang setiap 10 ms.

Guard baru hanya memakai critical section singkat untuk check/set flag. Operasi flash tetap di task context dan tidak ditaruh di critical section panjang.

## Audit area wajib
- **FOC ISR budget**: threshold hard overrun tetap >4000 cycle; log referensi worst-case 3519 cycle (87.98%), headroom 481 cycle. Ini bukan hasil build Run 7.
- **ADC/timer**: jalur TIM8 TRGO -> ADC1/ADC2 dual regular simultaneous -> DMA1 Ch1 tetap dipertahankan. DMA FOC IRQ priority 0 dan tidak memanggil RTOS.
- **Telemetry**: Run 6 seqlock tetap dipakai; GET_VALUES memakai realtime coherent FOC snapshot + read-reset averages untuk I_motor/I_in/Id/Iq/Vd/Vq. Tidak dikembalikan ke mutex telemetry.
- **RT data 50 Hz**: request/reply dan low-priority drop policy tetap ada. Stabilitas fisik 50 Hz masih perlu log hardware.
- **MCCONF/APPCONF**: full VESC6 wire image 481/493 byte tetap dipertahankan; set/get/default dan rollback persistence tetap ada. Hardware-limit clamping hanya pada field yang memang dibatasi.
- **EEPROM/flash**: 4-page transactional record, CRC32, sequence, magic committed terakhir, verify + fallback. Run 7 menutup race antar writer dan retry storm.
- **Thread/locking**: 5 task (fault_stop 6; timer/packet/blocking 5; sample_send 4), FOC di ISR. Flash writer kini single-owner.
- **LED/buzzer**: jalur PB2/PA4 dan state-machine yang sudah ada tidak diubah.
- **Sensor**: LEFT sensorless/Hall/encoder; RIGHT sensorless/Hall, encoder ditolak.
- **Rotor position**: coherent phase snapshot dan encoder-left policy tetap ada; verifikasi sign/offset fisik tetap diperlukan.
- **Commands**: duty/current/current-relative/brake/RPM/position/handbrake tersedia dan timeout-reset policy tetap ada.
- **Auto-detect**: R/L, flux, open-loop flux, Hall, encoder, Detect-All tetap melalui blocking worker; dual Detect-All commit/rollback tetap atomic pada level config image.
- **Calibration/fault**: current calibration gating, detection input-ignore, rollback, voltage/current fault counters tetap ada. False-fault behavior tetap perlu diuji pada hardware.

## Host-side verification (tanpa PlatformIO build)
- `python -m pytest -q vesc_tools/vesc_protocol/tests`: **95 passed**
- `python vesc_tools/selftest_protocol.py`: **PASS**
- `python tools/rtos_audit.py`: **PASS**
- `python tools/comm_audit.py`: **PASS**
- `python tools/debug.py --self-test`: **PASS**

## Belum boleh diklaim PASS
- Flash/RAM linker usage Run 7 karena `pio run` sengaja tidak dijalankan.
- Max ISR cycle firmware Run 7 pada hardware.
- RT Data fisik 50 Hz tanpa dropout.
- Semua mode sensor/detect pada motor nyata.
- Encoder sign/index/offset dan position closed-loop di mekanik steering.
- Brown-out tepat saat erase/program flash.
- LED/buzzer fisik, serta command duty/current/speed/position/brake/handbrake di bawah beban.

## Referensi upstream
Pola command motor dan configuration persistence dibandingkan dengan `vedderb/bldc` master `comm/commands.c` dan `conf_general.c` pada 30 Agustus 2026. Implementasi F103 tetap disesuaikan dengan single-bank flash, dual local motor, USART3-only, dan batas RAM STM32F103RCT6.
