# AUDIT CHECKPOINT RUN 11 — STM32F103RCT6 VESC PORT

Tanggal audit: 30 Agustus 2026

## Scope

Audit ulang menyeluruh dilakukan dari checkpoint Run 10 tanpa menjalankan `pio run`. Scope mencakup jalur ISR FOC, timer/PWM, ADC1/ADC2 dual simultaneous, ADC3 DCLINK, current conversion, current calibration, SVPWM/CCR, telemetry VESC, MCCONF/APPCONF VESC 6.00, flash configuration, sensorless/Hall/encoder, motor commands, auto-detect, RTOS/thread, LED/buzzer, clean code, dokumentasi fungsi/variabel, serta host-side protocol tests.

## Temuan baru yang diperbaiki

### 1. Freshness ADC3 DCLINK sebelumnya belum benar-benar membuktikan transfer baru

Checkpoint sebelumnya menganggap stream ADC3 sehat bila DMA2 Channel 5 aktif dan `CNDTR` bernilai 1/2. Circular DMA yang macet dapat tetap meninggalkan `CNDTR` pada nilai legal tersebut sehingga Vbus lama berpotensi dianggap fresh terus-menerus.

Run 11 memakai flag hardware `HTIF5/TCIF5` sebagai bukti minimal satu transfer ADC3 baru terjadi sejak ISR FOC sebelumnya. Flag dikonsumsi langsung oleh ISR FOC tanpa menambah interrupt ADC3 16 kHz. Transfer-error `TEIF5` tetap ditangani oleh IRQ DMA2 sebagai fault safety. Handler DMA2 tidak lagi membersihkan HT/TC yang dibutuhkan sebagai bukti freshness.

Perubahan ini tidak menambah buffer statik.

### 2. Magic-number offset MCCONF dibersihkan

Field kritis VESC 6.00 yang sebelumnya masih memakai literal `152`, `153`, `157`, `161`, dst diganti macro `VESC6_MC_OFF_*`. Tujuannya mencegah offset drift yang dapat menyebabkan VESC Tool melaporkan parameter truncated/corrupt walaupun panjang paket benar.

ABI tetap:

- MCCONF = 481 byte
- APPCONF = 493 byte
- `foc_sensor_mode` = offset 152
- `foc_pll_kp` = 153
- `foc_motor_l` = 161
- `foc_motor_r` = 169
- `foc_motor_flux_linkage` = 173
- `foc_openloop_rpm` = 201
- `foc_openloop_rpm_low` = 205
- `foc_hfi_start_samples` = 265

Tidak ada lagi numeric `get_auto_at(w,N)`, `put_auto_at(w,N,...)`, atau `w[N]` dalam `confgenerator.c` untuk field ABI yang diaudit.

### 3. Host pytest dibuat deterministik

`python3 -m pytest -q` sebelumnya dapat mengoleksi `tools/vesc_tool_test.py`, yang merupakan hardware probe dan membutuhkan pyserial/device. Ditambahkan `pytest.ini` agar generic pytest hanya menjalankan protocol test suite. Hasil akhir 95/95 PASS.

## ISR / ADC / timer

- CPU: 64 MHz.
- PWM: 16 kHz.
- Event FOC: 16 kHz.
- Budget ISR: 4000 cycle.
- Overrun counter hanya bertambah saat `cycles > 4000`; near-deadline terpisah.
- Semua early-return penting menutup timing melalui helper `foc_isr_finish_timing()`.
- TIM8 TRGO memicu ADC1/ADC2 dual regular simultaneous dan ADC3 DCLINK.
- DMA1 Channel1 priority IRQ 0 menjalankan current-loop FOC dari half-transfer setelah tiga rank current selesai.
- ADC clock port menggunakan DIV6 = sekitar 10.67 MHz. Referensi DQ_SIGN_FIX menggunakan DIV4 = 16 MHz; Run 11 sengaja mempertahankan DIV6 karena lebih aman terhadap spesifikasi ADC STM32F103 sambil mempertahankan topologi, urutan channel, trigger, polarity, dan kalibrasi referensi.
- Tiga rank current selesai sekitar 5.06 us setelah trigger.
- ADC3 DCLINK selesai sekitar 3.84 us setelah trigger.
- Enam rank ADC1/ADC2 selesai sekitar 56.16 us dari slot PWM 62.5 us, margin statis sekitar 6.34 us sebelum trigger berikutnya.

Mapping current yang dipertahankan:

- ADC1 rank1 PC1 = RIGHT DC
- ADC2 rank1 PC0 = LEFT DC
- ADC1 rank2 PA0 = LEFT phase A
- ADC2 rank2 PC3 = LEFT phase B
- ADC1 rank3 PC4 = RIGHT phase B
- ADC2 rank3 PC5 = RIGHT phase C
- LEFT C direkonstruksi sebagai `-(A+B)`
- RIGHT A direkonstruksi sebagai `-(B+C)`

Current conversion:

- phase/DC scale = 0.0200 A/count
- polarity = `offset - raw`
- boot calibration = 2000 sampel
- seed offset = 2000 count
- update = `(raw + offset) / 2`

CCR/SVPWM:

- U = CCR1
- V = CCR2
- W = CCR3
- update triplet/preload dipertahankan.

## Telemetry / VESC Tool

- Snapshot FOC tetap coherent/seqlock.
- Telemetry tidak menggunakan mutex blocking.
- `Imotor` mengikuti sign convention VESC `SIGN(Vq*Iq) * |Idq|`.
- `Iin/Ibattery` berasal dari DC shunt.
- Id, Iq, Vd, Vq, current refs, duty, ERPM, position dan fault tetap tersedia.
- `COMM_GET_VALUES` dan `COMM_GET_VALUES_SELECTIVE` tersedia.
- Low-priority periodic telemetry tidak boleh menahan reply UART utama.
- RT Data 50 Hz secara hardware belum dapat dinyatakan PASS tanpa log VESC Tool pengguna.

## MCCONF/APPCONF

- Full wire image dipertahankan untuk SET -> GET -> persistence.
- Unsupported runtime field tetap round-trip sehingga tidak hilang hanya karena backend F103 tidak menjalankan fitur tersebut.
- Sensorless diterima pada LEFT/RIGHT.
- Encoder hanya diterima pada LEFT.
- `foc_openloop_rpm_low` tetap diperlakukan sebagai fraksi 0..1.
- HFI field tetap round-trip, namun backend HFI tidak digunakan untuk sensorless F103.

## Flash / EEPROM emulation

Audit statis menunjukkan:

- CRC record tersedia.
- sequence/committed-record tersedia.
- single-owner flash transaction guard tersedia.
- fallback committed page lama tetap dipertahankan bila write baru gagal.

Brown-out tepat saat erase/program tetap memerlukan hardware fault-injection test.

## Command dan detection

Tersedia secara statis:

- duty
- current
- current brake
- RPM/speed
- position
- handbrake
- R/L detection
- flux-linkage detection
- Hall detection
- encoder detection
- Detect-All FOC

Hardware behavior di bawah beban belum dinyatakan PASS tanpa pengujian motor.

## Sensor policy

- LEFT: sensorless / Hall / encoder.
- RIGHT: sensorless / Hall.
- Encoder detect dijaga LEFT-only.
- Pure sensorless menggunakan open-loop startup lalu observer, bukan Hall/encoder sebagai sumber rotor phase.

## RTOS / communication

Static RTOS audit:

- `fault_stop` priority 6
- `timer` priority 5
- `packet_process` priority 5
- `blocking` priority 5
- `sample_send` priority 4
- FOC current loop tetap ISR DMA1 Channel1, bukan RTOS task.

Communication audit memastikan USART3 PB10/PB11, RX DMA circular, TX DMA, forwarding dual motor, dan FW/config dispatcher tetap tersedia.

## LED / buzzer

Implementasi LED dan buzzer tetap terdapat pada source dan jalur state/fault. Aktivasi fisik belum dapat dinyatakan PASS tanpa hardware.

## Clean code dan dokumentasi

- Tidak ada `#if 0` pada source C/H.
- Critical MCCONF magic-number offsets dibersihkan.
- Scanner dokumentasi C/H: 927 function definitions, 589 prototypes, 2082 parameters, 2990 variable declarators; missing = 0.
- Scanner script: Python/shell missing documentation = 0.

Catatan kualitas: sebagian komentar dokumentasi legacy masih bersifat generik. Secara coverage tidak ada yang missing, tetapi komentar generik dapat diperjelas lagi pada checkpoint selanjutnya tanpa mengubah algoritma.

## Host tests (tanpa pio run)

- `audit_run10_complete_docs.py` — PASS
- `audit_run10_all_files_docs.py` — PASS
- `audit_run8_contract.py` — PASS
- `audit_run9_contract.py` — PASS
- `audit_run11_full.py` — PASS
- `rtos_audit.py` — PASS
- `comm_audit.py` — PASS
- `debug.py --self-test` — PASS
- `vesc_tools/selftest_protocol.py` — PASS
- `python3 -m pytest -q` — 95/95 PASS
- Python compileall — PASS

## Belum dapat diklaim PASS tanpa build/hardware pengguna

- RAM/Flash aktual Run 11.
- Max ISR cycle aktual Run 11 setelah tambahan freshness check ADC3.
- RT Data 50 Hz fisik tanpa dropout.
- Sensorless startup dan handover observer pada motor nyata.
- Hall/encoder LEFT dan Hall RIGHT pada motor nyata.
- Rotor position/sign/offset encoder.
- Semua detection/calibration di hardware.
- Brown-out saat flash write.
- LED/buzzer fisik.
- duty/current/RPM/position/brake/handbrake di bawah beban.
- zero-fault claim pada semua operating condition.
