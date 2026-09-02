# Audit Checkpoint Run21 — Standard Motor Commands + Two Python Tools

Tanggal: 2026-08-31

## Feedback yang menjadi acceptance gate

Pengguna melaporkan `SET_DUTY`, `SET_CURRENT`, `SET_RPM`, dan `SET_POS` belum menghasilkan kendali motor yang dapat digunakan. Pengguna juga meminta seluruh tool Python dikonsolidasikan menjadi hanya dua file di `tools/`, default `/dev/ttyUSB0` 115200 USART3, dengan CLI teks dan debug streaming lengkap termasuk data raw.

## Bug utama sensorless startup yang diperbaiki

Pada pure sensorless, perhitungan target forced-openloop sebelumnya hanya memakai `abs(iq_filter)`. Dengan default `foc_openloop_rpm_low = 0` dan rotor diam, `iq_filter = 0`, sehingga `openloop_target_erpm` dapat menjadi 0. Fase forced-openloop kemudian tidak berputar dan observer tidak memperoleh startup yang diperlukan.

Implementasi Run21 mengikuti pola VESC yang tersedia pada referensi pengguna: arus untuk pemetaan open-loop adalah arus aktual ditambah `foc_sl_openloop_boost_q`, lalu dibatasi `foc_sl_openloop_max_q`. Dengan demikian boost Q ikut menghasilkan RPM open-loop dari zero-speed.

Perubahan ada di `src/motor/foc_math.c` dan tetap memakai `foc_openloop_rpm_low` sebagai fraction 0..1, bukan ERPM.

## Command standar VESC

`COMM_SET_DUTY`, `COMM_SET_CURRENT`, `COMM_SET_CURRENT_BRAKE`, `COMM_SET_RPM`, `COMM_SET_POS`, `COMM_SET_HANDBRAKE`, dan `COMM_SET_CURRENT_REL` tetap memakai ABI/skala VESC standar. Jalur penerimaan sekarang mempunyai breadcrumb diagnostik tanpa menambah ACK non-standar pada command VESC.

Urutan arbitration diperjelas:

1. claim UART application;
2. cek mc input lock;
3. apply `motor_set_*`;
4. reset command timeout;
5. catat accepted/rejected.

COMM_DIAG dinaikkan ke revision 17 dan menambahkan command terakhir, motor target, hasil command, alasan app reject, raw wire value, accept count, dan reject count. Ini membedakan motor diam karena command ditolak dari motor diam karena sensorless belum lock.

## Position control

`SET_POS` tetap tersedia. Namun pure sensorless dari zero-speed tidak mempunyai posisi rotor absolut. Karena itu tool tidak mengarang sudut rotor. Position dari keadaan diam direkomendasikan pada LEFT encoder. Pure sensorless position hanya dapat dicoba setelah observer valid; `--force` tersedia khusus eksperimen.

## Tool Python final

Folder `vesc_tools/` dihapus. `tools/` hanya memiliki dua file `.py`:

- `tools/vesc.py`: CLI teks command standar VESC dan PlatformIO pre-script FreeRTOS.
- `tools/debug.py`: commissioning lengkap, standard/extended telemetry, raw ADC/current/sensor, calibration, ISR, USART3, config, detection, command test, dan stream-all multi-rate.

`platformio.ini` sekarang memakai `extra_scripts = pre:tools/vesc.py`. Fungsi lama `extra_script.py` telah digabung sehingga penghapusan file tersebut tidak menghilangkan source/include FreeRTOS.

Default kedua tool: `/dev/ttyUSB0`, 115200 baud.

Streaming debug memakai fast telemetry dan slow heavy diagnostics secara multi-rate agar data lengkap tersedia tanpa memaksa payload raw besar pada 50 Hz. Default `stream-all`: 20 Hz fast dan 2 Hz raw/comm/calibration.

## Resource

Firmware hanya menambah breadcrumb command kecil. Alasan reject per-motor disimpan sebagai `uint8_t[2]`, bukan array enum, untuk menjaga RAM. Tidak ada buffer telemetry besar baru dan debug Python tidak memakai RAM MCU.

## Verifikasi tanpa pio run

`pio run` tidak dijalankan sesuai instruksi pengguna.

Sebelum konsolidasi tool, seluruh regression utama dijalankan terhadap source Run21:

- Run20 build feedback PASS
- Run19 dual sensorless PASS
- Run18 telemetry PASS
- Run17 ISR/boot calibration PASS
- Run16 VESC6 ABI PASS
- Run15 host syntax PASS
- Run14 operator integrity PASS
- Run13 ADC margin PASS
- Run12 clean format/dead-static scan PASS
- Run11 full static audit PASS
- RTOS audit PASS
- Communication audit PASS
- debug protocol self-test PASS
- pytest 95/95 PASS

Setelah konsolidasi final:

- `py_compile tools/vesc.py tools/debug.py` PASS
- `tools/debug.py --self-test` PASS, termasuk COMM_DIAG rev17
- simulasi PlatformIO/SCons pre-script `tools/vesc.py` PASS
- tepat dua Python di `tools/` PASS
- folder `vesc_tools/` tidak ada PASS
- default ttyUSB0/115200 PASS
- no `#if 0` pada `src/` PASS
- tab/trailing whitespace pada source/tools yang diaudit: 0

## Yang belum diklaim PASS

Karena tidak menjalankan PlatformIO build dan tidak memiliki hardware pada run ini, berikut belum diklaim PASS:

- build/link Run21 pada toolchain pengguna;
- RAM/Flash final;
- motor LEFT/RIGHT benar-benar start dari diam dengan sensorless;
- observer handover dan reverse pada hardware;
- ISR hardware tetap di bawah budget 4000 cycle;
- SET_POS pure sensorless dari zero-speed (secara fisik tidak memiliki absolute rotor reference);
- RT stream aktual terhadap latency USB-UART pengguna.
