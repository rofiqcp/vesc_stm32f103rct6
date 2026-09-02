# AUDIT CHECKPOINT RUN31 — ADC/PWM ISR, HFI VESC Tool, Native Fault, Detect-All

## Tujuan
Run31 dibuat dari Run30 setelah pengujian hardware menunjukkan kondisi berikut:

- current calibration selesai (`RESULT CODE: 0`, `cal_valid=True`, `motor_ready=True`),
- tetapi `SET_CURRENT +1 A` kemudian menghasilkan VESC `FAULT_CODE_DRV (3)`,
- sampling audit Run30 masih menampilkan `TRIGGER_ROUTE`,
- VESC Tool masih menampilkan `HFI Start Samples 2` sebagai parameter truncated.

Jadi Run31 tidak menganggap current calibration sebagai akar masalah. Fault muncul sesudah jalur FOC aktif.

## 1. Akar masalah yang ditangani

### 1.1 `FAULT_CODE_DRV (3)` bersifat ambigu
Firmware mempunyai fault internal terpisah:

- `MOTOR_FAULT_ADC_DMA`
- `MOTOR_FAULT_FOC_ISR_OVERRUN`

Keduanya harus dipetakan ke `FAULT_CODE_DRV` pada wire VESC. Karena itu Run31 menambahkan diagnostik native melalui:

```bash
python3 tools/debug.py fault-detail --motor 0
python3 tools/debug.py fault-detail --motor 1
```

Jika fault masih terjadi di board, jangan menebak dari angka 3 saja. Gunakan native fault, ISR max cycles, sampling flags, dan snapshot current yang dicetak command tersebut.

### 1.2 Run30 memakai ADC3/DMA2 tambahan untuk VBUS
Firmware hoverboard V13 referensi yang sudah menjalankan motor nyata tidak memakai ADC3/DMA2 untuk DCLINK. VBUS berada pada PC2 sebagai ADC1 rank-4 di scan dual ADC1+ADC2.

Run31 menghapus ADC3/DMA2 dari jalur motor. Ini menghapus:

- ADC3 trigger-route kedua,
- DMA2 Channel5 freshness gate,
- kemungkinan `ADC_DMA` palsu ketika current FOC aktif,
- penyebab audit-only `TRIGGER_ROUTE` Run30 yang membandingkan route ADC3 dengan route ADC1/ADC2.

Field diagnostik revision-16 untuk ADC3 tetap dipertahankan sebagai slot nol supaya `debug.py` lama tidak kehilangan alignment packet.

### 1.3 Build debug `-Og` tidak boleh dipakai untuk dual FOC
Clock runtime firmware adalah 64 MHz (HSI/2 × PLL16), bukan 72 MHz. Pada current loop 16 kHz tersedia 4000 CPU cycles antar event.

Run31 menetapkan:

- default environment: `stm32f103rc`
- release: `-O3`
- debug-symbol environment: tetap `-O3 -g3`, bukan `-Og`

Tujuannya agar build ber-symbol tidak mengubah deadline current-loop.

## 2. ADC/PWM/ISR Run31

### 2.1 Mapping lima rank pertama sama dengan hoverboard V13

| Rank | ADC1 low16 | ADC2 high16 | Fungsi |
|---|---|---|---|
| 1 | PC1 | PC0 | RIGHT DC / LEFT DC |
| 2 | PA0 | PC3 | LEFT A / LEFT B |
| 3 | PC4 | PC5 | RIGHT B / RIGHT C |
| 4 | PC2 | PA2 | VBAT / APP ADC1 |
| 5 | TEMP | PA3 | MCU temp / APP ADC2 |

Polaritas current tetap mengikuti firmware hoverboard referensi:

```text
current = offset - raw
```

### 2.2 Rank ke-6 hanya filler untuk boundary ISR VESC
Hoverboard V13 memakai 5 rank dan DMA transfer-complete. Backend VESC Run31 menjalankan dua loop FOC yang lebih berat. Karena itu ditambahkan rank-6 filler cepat:

- ADC1 PC2
- ADC2 PA3

DMA circular panjangnya 6 word dan IRQ FOC memakai **half-transfer**. Dengan demikian HT terjadi tepat setelah rank 1..3, ketika semua enam kanal current kedua motor sudah koheren. FOC tidak harus menunggu conversion temperature 239.5-cycle.

Rank 4/5 yang dibaca pada saat HT adalah frame lengkap sebelumnya, sehingga umur VBUS/APP/temp maksimum satu PWM period (62.5 us). Ini cukup cepat untuk VBUS normalization/protection dan memberi lebih banyak budget CPU untuk FOC.

### 2.3 Trigger/timer

- TIM1 RIGHT: center-aligned PWM, master TRGO enable
- TIM8 LEFT: center-aligned PWM, gated slave ITR0, RCR=1, TRGO update
- ADC1 master external trigger: TIM8_TRGO
- ADC1 ETRGREG AFIO remap: aktif
- ADC2 slave: regular simultaneous
- ADC clock: PCLK2/6 = 10.67 MHz pada CPU/PCLK2 64 MHz
- DMA1 Channel1: 32-bit peripheral/memory, circular, high priority
- FOC IRQ: DMA half-transfer setelah rank current 1..3

## 3. HFI truncated
Port mengiklankan ABI VESC 6.00 dan HFI runtime memang tidak digunakan pada hardware ini, tetapi setiap field MCCONF tetap harus valid terhadap range VESC Tool.

Run24–Run30 dapat membawa MCCONF lama dengan:

```text
foc_hfi_start_samples = 2
```

Run31 melakukan migrasi saat MCCONF di-import:

```text
jika foc_hfi_start_samples < 5 -> 5
```

Migrasi ini dilakukan pada image RAM yang aktif. Jadi `GET_MCCONF` langsung mengembalikan nilai valid tanpa harus menghapus seluruh EEPROM. Saat MCCONF berikutnya disimpan, nilai default VESC 6.00 tersebut ikut tersimpan dan transaksi sukses tetap menghasilkan notifikasi EEPROM 5 beep.

Factory image Run31 sekarang mengikuti default generik firmware VESC 6.00 untuk field HFI yang ada pada ABI ini (bukan profil hardware khusus):

```text
HFI voltage start = 20 V
HFI voltage run   = 4 V
HFI voltage max   = 6 V
HFI gain          = 0.30
HFI hyst          = 0
HFI transition    = 3000 ERPM
HFI start samples = 5
HFI obs override  = 0.001 s
HFI sample enum   = 1
```

Validasi langsung:

```bash
python3 tools/debug.py vesc-tool-check --motor 0
python3 tools/debug.py vesc-tool-check --motor 1
```

`HFI Start Samples` wajib `>=5`; factory/migrated Run31 adalah 5 (default VESC 6.00).

## 4. Detect-All FOC
`COMM_DETECT_APPLY_ALL_FOC` aktif dan melakukan per motor:

1. validasi/calibrate current,
2. detect resistance R,
3. detect L dan Ld-Lq,
4. detect flux linkage,
5. hitung PI current dari R/L,
6. sensor discovery:
   - LEFT: encoder ABI -> Hall -> sensorless,
   - RIGHT: Hall -> sensorless,
7. validasi sensorless bila menjadi fallback,
8. apply hasil,
9. commit MCCONF.

Jika `detect_can=true` dikirim ke M0 pada board dual-local ini, firmware mengerjakan M0/ID1 lalu M1/forwarded-ID2 dan baru melakukan **atomic dual commit** jika keduanya sukses.

Command:

```bash
python3 tools/debug.py detect-all-foc --motor 0 --both-local --yes
```

PASS hanya bila R, L, dan flux hasil readback semuanya positif.

## 5. Hall detect
Hall autodetect tidak memakai Hall sebagai phase feedback untuk memutar rotor. Detection memakai:

- forced electrical phase,
- ramp Id,
- Iq=0,
- forward electrical sweep,
- reverse electrical sweep,
- pembacaan Hall sebagai hasil,
- validasi enam Gray states.

Ini mengikuti prinsip mode open-loop Id yang telah terbukti pada firmware hoverboard referensi.

## 6. Buzzer/LED tetap sesuai kontrak

- boot: power-on melody tetap aktif,
- normal running: 3 pulse **LED saja**, tidak membunyikan buzzer,
- setiap transaksi flash/EEPROM sukses: **5 beep**.

Run30 regression audit untuk kontrak ini tetap PASS.

## 7. Verifikasi offline Run31
Dijalankan pada source yang dikemas:

```text
debug.py --self-test : PASS
audit_run29.py       : 25/25 PASS
audit_run30.py       : 41/41 PASS
audit_run31.py       : 71/71 PASS
```

Firmware hoverboard V13 referensi juga diuji ulang dari source upload:

```text
HOST_COMPILE_CHANGED_SOURCES_PASS
FOC_GCC_CLANG_RUNTIME_PASS
MOTOR_CONTROL_V12_GCC_CLANG_PASS
MOTOR_CONTROL_V13_GCC_CLANG_PASS
VESC_PROTOCOL_GCC_CLANG_PASS
CONFIG_SERIALIZER_GCC_CLANG_PASS
ALL_FINAL_HOST_CHECKS_PASS
```

Log lengkap:

- `docs/RUN31_HOST_VERIFICATION.txt`
- `docs/HOVERBOARD_REFERENCE_HOST_VALIDATION_RUN31.txt`

## 8. Batas verifikasi
Environment pembuatan checkpoint tidak memiliki `pio` maupun `arm-none-eabi-gcc`, sehingga cross-build STM32 tidak dapat dijalankan di sini. Audit source, Python self-test, regression audit, dan host validation referensi sudah PASS. Build/upload dan pembuktian motor fisik harus dilakukan pada board pengguna.

Jangan menyatakan hardware PASS hanya dari ACK protocol. Gunakan staged acceptance di `docs/RUN31_HARDWARE_ACCEPTANCE_COMMANDS.txt`.
