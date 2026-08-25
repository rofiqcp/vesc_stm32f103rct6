# Audit V33: struktur VESC, kalibrasi arus, dan fitur yang belum diterapkan

Tanggal audit: 2026-08-25

## Baseline

- VESC master: commit `471ebfba4af6cb33f4e66fc8890378cf44f3b6ad`
  (2026-08-24), https://github.com/vedderb/bldc
- EFeru hoverboard FOC: commit
  `4f141cbc97b297e194fd58e56444bd0322f902ac` (2026-07-04),
  https://github.com/EFeru/hoverboard-firmware-hack-FOC
- Target port: STM32F103RCT6, 256 KiB flash, 48 KiB RAM, dua inverter lokal.
- Pengecualian sesuai permintaan: HFI, CAN hardware, IMU, BMS, `bm_if`, NRF,
  LEDPWM, COM USB, QML UI, LispIF, NTC temperature, dan LZO.

Audit membandingkan perilaku dan kepemilikan modul, bukan menyalin seluruh
VESC master. VESC master saat ini memakai MCU/RTOS/periferal yang berbeda dan
sejumlah fiturnya tidak mungkin dipindahkan tanpa hardware tambahan.

## Ringkasan hasil

Status sumber V33: **lulus validasi host**, tetapi **belum lulus hardware**.

- 28 skrip regresi Python tersedia dan seluruhnya lulus pada audit terakhir.
- 20 translation unit logika dikompilasi host dengan warning ketat dan `-Werror`.
- 13 unit kritis lulus GCC `-fanalyzer`.
- Parser/debug self-test lulus, termasuk current-cal revision 17.
- ARM GCC/PlatformIO tidak tersedia di lingkungan audit, sehingga ukuran RAM,
  flash, dan binary target V33 belum boleh diklaim final.
- Kalibrasi dan speed test aktif belum dijalankan pada PCB dari lingkungan ini.

## Penempatan ulang file yang diminta

| File lama | Keputusan V33 | Pemilik yang benar |
|---|---|---|
| `fault.c/.h` | Dihapus; hanya wrapper tipis dan tidak ada file setara di VESC | enum di `datatypes.h`, state/mapping di `motor/mc_interface.c`, shutdown task di `motor/mc_interface_tasks.c` |
| `motor_tasks.c/.h` | Dipindah dan header publik dihapus | implementasi privat `motor/mc_interface_tasks.c`, API di `motor/mc_interface.h` |
| `motor_types.h` | Dihapus; hanya alias | `datatypes.h` |
| `status_io.c/.h` | Dipindah; header terpisah dihapus | `hwconf/hw_status.c`, deklarasi board di `hwconf/hw.h` |
| `app_config.h` | Dihapus; hanya alias | `applications/appconf_default.h` dan `applications/app.*` |
| `board_pins.h` | Dihapus; hanya alias | `hwconf/hw_hoverboard.h` |
| `debug_sample.c/.h` | Dipindah dan dinamai menurut pemilik VESC | `motor/mc_interface_sample.c/.h` |

Pemisahan `mc_interface_tasks.c` dan `mc_interface_sample.c` tetap dipakai agar
`mc_interface.c` tidak menjadi satu file sangat besar pada port RAM kecil.
Nama API dan header publik tetap berada di namespace `mc_interface`, sehingga
kepemilikannya sama dengan VESC tanpa mengorbankan keterbacaan.

## Perbaikan kode dan kepemilikan

- Boot sekarang memakai `mc_interface_init(false)`, bukan jalur inisialisasi
  port yang terpisah.
- Factory MCCONF/APPCONF benar-benar diaplikasikan ke runtime sebelum mencoba
  impor flash. Ini memperbaiki celah first boot/virgin flash yang sebelumnya
  hanya membuat wire image tanpa memastikan semua default aktif di motor.
- Backend `motor_hw_gate_global()` dihapus. Fungsi tersebut bernama seolah
  mengendalikan gate inverter, tetapi sebenarnya selalu menahan PA5 power latch
  HIGH dan mengabaikan argumennya. PA5 kini hanya dimiliki `hw_status_power_hold`.
- Stub tracking offset yang tidak melakukan apa pun beserta accumulator matinya
  dihapus. Tidak ada lagi fitur palsu yang terlihat aktif.
- Counter buzzer terbatas yang tidak pernah dipakai dihapus; durasi dimiliki
  state machine status, sedangkan TIM3 IRQ hanya menghasilkan gelombang nada.
- Sampler diformat ulang, memvalidasi ID motor, memakai satu statement per baris,
  dan mempertahankan ring-buffer ISR tanpa alokasi dinamis.
- Dokumen patch/validasi tahap lama dipindah ke `docs/history/` agar root proyek
  hanya berisi entry point yang masih berlaku.

## Analisis log kalibrasi yang diberikan

Target `6096` konsisten dengan implementasi: 4096 sampel undriven, lalu 1000
sampel driven motor kiri dan 1000 motor kanan.

| Bukti | Interpretasi |
|---|---|
| Shift undriven→driven hanya 0…16 count | Offset DC dasar kedua motor masuk akal; bukan indikasi bias op-amp yang bergeser besar |
| DC-current kiri/kanan spread 13/11 dan stddev ~1.7/2.0 | ADC/DMA dan referensi bias dasar hidup dan stabil |
| Kiri U/V menyentuh 4074/4076, tetapi mean tetap 2669/2659 | Ada spike switching sporadis dekat rail, bukan noise kontinu pada semua sampel |
| Kanan U/V memiliki spread dan bentuk noise hampir identik | Mengarah ke gangguan common-mode/schedule PWM-ADC, bukan dua kanal rusak independen |
| ISR naik 67.533 dan DMA aktif | Jalur sampling hidup; kegagalan bukan karena ISR berhenti |
| ADC3 Vbus tidak stale | Jalur Vbus terpisah bukan penyebab langsung kegagalan ini |
| TIM2 berhenti | Normal untuk desain ini; trigger aktif adalah TIM8 TRGO, bukan TIM2 |
| BDTR akhir tidak memuat MOE | Snapshot diambil sesudah fail handler mematikan bridge; tidak membuktikan MOE tidak pernah aktif |

Kesimpulan paling kuat: sanity check lama mencampur **raw spike sporadis** dengan
**noise baseline**. Satu/segenggam spike dapat membuat spread dan stddev mentah
melewati hard limit walau 99% lebih sampel valid. Namun spike tidak boleh sekadar
disembunyikan, karena jumlah besar tetap menunjukkan timing/hardware bermasalah.

## Perubahan kalibrasi revision 17

- Raw minimum/maksimum tetap direkam agar spike tetap terlihat.
- Offset driven dihitung dari inlier dalam jendela ±256 count terhadap mean
  undriven.
- Maksimum 10 outlier dari 1000 sampel per kanal. Lebih dari 1%, kurang dari
  990 inlier, atau stddev inlier >80 count tetap **hard fail**.
- Satu atau lebih outlier, raw spread >160, atau stddev inlier >16 menghasilkan
  warning tetapi tidak sendiri mengizinkan data buruk melewati hard rule.
- Mean range 128…3967 tetap hard guard terhadap ADC rail/gross fault.
- Tidak ada threshold yang mengubah PWM timing atau polaritas hardware.

Perkiraan dari statistik agregat menunjukkan spike rail kiri mungkin hanya
beberapa sampel, tetapi jumlah sebenarnya tidak dapat dipulihkan dari mean/min/
max/stddev saja. Karena itu revision 17 mengirim `outlier_count` nyata per kanal.

### Jejak MOE per motor

Revision 17 menambahkan:

- `moe_request_adc[LEFT/RIGHT]`
- `moe_confirm_adc[LEFT/RIGHT]`
- `first_sample_adc[LEFT/RIGHT]`
- `moe_confirmed_mask` dan `moe_fail_mask`

Kalibrasi hanya mulai warm-up/sampling driven saat `MotorRuntime.pwm_enabled`
**dan bit hardware `TIM_BDTR_MOE`** sama-sama aktif. Jika tidak terkonfirmasi
dalam 128 event ADC, kanal phase motor tersebut hard-fail. Jalur enable tetap
menunggu dua frame preload, lalu menerapkan delapan sampel blanking proteksi.

Pemetaan motor sesuai firmware hoverboard resmi:

- motor kiri: TIM8;
- motor kanan: TIM1.

## Tes speed RT 50 Hz / APP 20 Hz

`tools/debug.py speed-test` adalah tes aktif baru dengan dua deadline independen:

- refresh `COMM_SET_RPM` + `COMM_GET_VALUES`: 50 Hz;
- `COMM_GET_DECODED_ADC` PA2/PA3: 20 Hz.

Tes mencatat rate aktual, timeout, slot terlewat, jitter RT, jitter APP, fault,
ERPM, duty, Id/Iq, arus, Vbus, dan APP ADC. CSV berisi event `rt` dan `app`
terpisah. Default pass gate adalah RT/command ≥45 Hz dan APP ≥18 Hz tanpa
timeout atau fault. Kedua motor diuji dengan dua invokasi terpisah.

## Fitur VESC non-excluded yang belum diterapkan

| Area | Belum diterapkan / berbeda | Alasan/status |
|---|---|---|
| Wire ABI | VESC 7.01 config/protocol penuh | Port sengaja pinned VESC 6.00 untuk 481/493-byte compatibility |
| Motor type | BLDC trapezoidal dan DC motor | Port FOC-only |
| Current calibration | Online undriven current-offset LP tracking (`foc_offsets_cal_mode` bit 2) | Stub lama dihapus; implementasi nyata memerlukan policy config dan validasi drift pada PCB |
| Current hardware | Tiga phase-current sensor per motor | PCB hanya menyediakan dua phase-current kanal per motor; phase ketiga disintesis |
| FOC sampling | Dynamic V0/V7 dan high-current sample modes | Ditolak eksplisit karena topologi shared ADC/PWM belum dikualifikasi |
| Phase voltage | Kalibrasi offset phase-voltage/phase-filter seperti board VESC yang memilikinya | Tidak ada backend pengukuran phase-voltage yang setara pada PCB ini |
| Sensor | SPI/BiSS/sin-cos/resolver/encoder khusus VESC | Hardware port hanya LEFT ABI/Hall/sensorless dan RIGHT Hall/sensorless |
| Applications | PPM, Nunchuk, PAS, servo, custom app | Hanya ADC PA2/PA3 + UART yang memiliki backend nyata |
| Firmware update | Bootloader jump, erase/write new app, update-all | Belum ada bootloader/update pipeline yang terbukti untuk layout F103 ini |
| Config extension | Custom config XML/hardware config protocol | Belum ada backend custom configuration |
| IO/GNSS/power switch | IO-board commands, GNSS telemetry, PSW commands | Tidak ada periferal/backend |
| Terminal/events | Registrasi callback terminal dinamis dan event subsystem penuh | Terminal statis/diagnostik inti tersedia; plugin/event framework belum ada |
| Persistence | EEPROM variables custom/HW dan backup-data framework penuh | Diganti record flash transactional empat halaman + odometer |
| Audio motor | VESC motor-audio/tone-via-winding API | Dihapus daripada memberi stub palsu; buzzer PCB memakai TIM3 |
| ADC ISR API | `mc_interface_adc_inj_int_handler` / `mcpwm_foc_tim_sample_int_handler` | Bukan missing behavior: port memakai DMA1 Channel1 half-transfer shared dual-ADC ISR |

Perbandingan command dispatch menemukan 48 command case yang sama dengan VESC
master dan 64 case upstream yang tidak ada. Mayoritas 64 tersebut termasuk
subsystem yang dikecualikan (BMS, CAN, IMU, NRF, QML, Lisp, BM/LZO). Sisa
utamanya adalah firmware update, PPM/Nunchuk/servo, custom config, IO/GNSS, dan
power-switch seperti pada tabel.

## Risiko yang masih terbuka

1. **Kalibrasi harus diulang di hardware.** Revision 17 memperbaiki klasifikasi
   statistik dan observabilitas MOE; belum ada bukti board bahwa `valid=True`.
2. **Penyebab spike fisik belum hilang.** Bila `outlier_count >10`, jangan
   memperbesar threshold lagi. Periksa ground analog, bias op-amp, ringing gate,
   dead-time, probe TIM8/TIM1, dan posisi ADC terhadap edge PWM.
3. **BKIN eksternal masih OFF.** Pin/polaritas fault gate-driver belum divalidasi.
4. **RAM hanya memiliki headroom sekitar 6004 byte pada binary V32 yang
   dilaporkan pengguna.** V33 menghapus state mati dan menambah diagnostik kecil,
   tetapi angka final wajib berasal dari link map ARM V33.
5. **MCU temperature hanyalah proxy board.** Karena NTC sengaja dikecualikan,
   temperatur junction MOSFET tidak terukur langsung.

## Keputusan release

- Source audit: **PASS**.
- Host compile/analyzer/regression: **PASS**.
- ARM target build + ukuran final: **NOT RUN**.
- Hardware calibration revision 17: **NOT RUN**.
- Speed test LEFT/RIGHT: **NOT RUN**.
- Status produksi: **BELUM SIAP** sampai checklist hardware lulus.
