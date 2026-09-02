# Audit Checkpoint Run25 — Telemetry VESC, ADC/ISR Timing, dan APP

Tanggal: 2026-08-31

## Tujuan

Run25 mengaudit ulang jalur lengkap wire telemetry → cache/snapshot → FOC ISR → ADC dual-DMA → APP ADC. Fokusnya memastikan telemetry standar tetap merespons dalam kondisi idle/running/fault/calibration, timing ADC dapat diaudit dari register runtime, dan seluruh mode aplikasi yang benar-benar memiliki backend hardware pada STM32F103 hoverboard dapat dipakai tanpa klaim support palsu.

Tidak ada `pio run` pada run ini.

## Perubahan Run25

### 1. COMM_GET_DECODED_ADC selalu membalas

Run24 menahan reply `COMM_GET_DECODED_ADC` saat `app_adc_data_ready()==false`. Perilaku tersebut dapat membuat polling standar VESC Tool timeout ketika aplikasi belum di-arm, saat calibration, atau saat fault.

Run25 selalu mengirim payload standar 17 byte:

1. decoded ADC1 × 1e6
2. voltage ADC1 × 1e6
3. decoded ADC2 × 1e6
4. voltage ADC2 × 1e6

Sebelum frame ADC fisik pertama tersedia getter bernilai nol. Sesudah snapshot tersedia, data tetap berasal dari PA2/PA3 aktual meskipun output APP belum di-arm.

### 2. Runtime sampling contract diperketat

`motor_hw_sampling_contract_flags()` sekarang juga memeriksa:

- prescaler ADC = PCLK2/6;
- kode sampling time rank current/auxiliary;
- DMA1 HT interrupt aktif dan TC interrupt nonaktif;
- ADC3 sample time 28.5 cycle;
- EXTSEL ADC1 dan ADC3 ke TIM8_TRGO;
- AFIO ADC1 ETRGREG remap tetap aktif;
- rank-6 filler tetap PC2/PA3.

Dengan ini perubahan register yang menggeser timing sampling tidak hanya didokumentasikan, tetapi dapat muncul sebagai non-zero `sampling_contract_flags` pada hardware diagnostic.

### 3. APP ADC analog diperluas

Mode VESC yang hanya membutuhkan PA2/PA3 sekarang didukung:

- `ADC_CTRL_TYPE_CURRENT`
- `ADC_CTRL_TYPE_CURRENT_REV_CENTER`
- `ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER`
- `ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC`
- `ADC_CTRL_TYPE_DUTY`
- `ADC_CTRL_TYPE_DUTY_REV_CENTER`
- `ADC_CTRL_TYPE_PID`
- `ADC_CTRL_TYPE_PID_REV_CENTER`

Center modes memakai `voltage_center` sesuai pola VESC dan dipetakan ke -1..+1. Config validator memastikan `voltage_center` mempunyai jarak aman dari `voltage_start` dan `voltage_end`.

Mode berbasis tombol eksternal tetap ditolak karena PCB/build ini tidak mempunyai backend reverse/cruise button yang dapat dijamin. Traction control ADC juga tetap ditolak karena tidak ada backend TC. Ini disengaja agar VESC Tool tidak menerima konfigurasi yang sebenarnya tidak dapat dieksekusi.

## ADC dan ISR

### Mapping current-critical

Tiga dual-rank pertama sama dengan reference hoverboard yang diberikan pengguna:

| Rank | ADC1 low16 | ADC2 high16 | Sampling |
|---|---|---|---|
| 1 | PC1 / RIGHT_DC | PC0 / LEFT_DC | 1.5 cycle |
| 2 | PA0 / LEFT_A | PC3 / LEFT_B | 7.5 cycle |
| 3 | PC4 / RIGHT_B | PC5 / RIGHT_C | 7.5 cycle |

DMA1 Channel1 memakai enam word circular. Half-transfer terjadi setelah rank-3 dan menjadi satu-satunya IRQ normal current/FOC. TC tetap dimatikan.

Auxiliary:

| Rank | ADC1 | ADC2 |
|---|---|---|
| 4 | PC2 diagnostic | PA2 APP ADC1 |
| 5 | internal temperature | PA3 APP ADC2 |
| 6 | PC2 filler cepat | PA3 filler cepat |

ADC3 secara terpisah membaca PC2 DCLINK pada TIM8_TRGO yang sama melalui DMA2 Channel5.

### Timing teoritis berdasarkan register yang dikonfigurasi

CPU/PCLK2 = 64 MHz, ADC prescaler /6 → ADC clock = 10.666667 MHz.

- rank1: 1.5 + 12.5 = 14 ADC cycle
- rank2: 7.5 + 12.5 = 20 ADC cycle
- rank3: 20 ADC cycle
- DMA HT/current boundary = 54 cycle = **5.0625 µs** setelah trigger
- ADC3 PC2: 28.5 + 12.5 = 41 cycle = **3.8438 µs**
- rank4: 20 cycle
- rank5 TEMP: 239.5 + 12.5 = 252 cycle
- rank6: 20 cycle
- full six-rank sequence = 346 cycle = **32.4375 µs**
- PWM period 16 kHz = **62.5 µs**
- full-sequence margin = **30.0625 µs**

ADC3 selesai sekitar 1.22 µs sebelum current HT sehingga DCLINK fresh dapat dipakai pada frame current tersebut. Entry HT berikutnya tetap berjarak satu periode PWM atau 4000 CPU cycle.

Perbedaan terhadap reference sengaja dipertahankan: reference sederhana memakai ADC /4 dan 5 rank + DMA TC. Port VESC memakai /6 agar ADC tetap ≤14 MHz dan enam rank agar HT jatuh tepat setelah tiga dual-rank current, sehingga FOC tidak menunggu rank temperature yang lambat.

## Current calibration

Pipeline Run24 dipertahankan:

- kedua bridge 50%/50%/50% zero-vector;
- warm-up 64 frame setelah MOE confirmed;
- 2000 frame keenam kanal current;
- offset = `(min + max) / 2`;
- scale = 0.020 A/count;
- polarity = `offset - raw`.

Estimator midpoint adalah instruksi pengguna; reference `bldc.c` memakai 2000 sample tetapi exponential `(adc + offset)/2`. Mapping ADC, common-mode PWM aktif, jumlah sample, dan polarity mengikuti reference; estimator final sengaja berbeda sesuai instruksi pengguna.

## Telemetry standar VESC

`COMM_GET_VALUES` dan `COMM_GET_VALUES_SELECTIVE` tetap dapat diproses walaupun motor belum ready untuk torque, idle, timeout, atau sedang fault. Motor-control gate tidak memblokir telemetry read.

Field selective yang digunakan RT50:

- current_motor — bit 2, int32 ×100
- current_in — bit 3, int32 ×100
- Id — bit 4, int32 ×100
- Iq — bit 5, int32 ×100
- duty — bit 6, int16 ×1000
- ERPM — bit 7, int32
- Vin — bit 8, int16 ×10
- fault — bit 15
- position — bit 16, int32 ×1e6
- controller ID — bit 17
- Vd — bit 19, int32 ×1000
- Vq — bit 20, int32 ×1000
- status — bit 21

`current_motor` mengikuti konvensi VESC `SIGN(Vq*Iq) * sqrt(Id²+Iq²)`. `current_in` berasal dari DC shunt. Enam nilai average VESC (I_motor, I_in, Id, Iq, Vd, Vq) memakai accumulator 100 Hz dengan fallback snapshot koheren bila belum ada sample accumulator.

Motor RIGHT menggunakan recursion `COMM_FORWARD_CAN` controller ID 2 lalu source telemetry dipilih dari motor-thread 2, bukan cache LEFT.

## Rate USART3 115200

Tool default:

- RT Data: 50 Hz `COMM_GET_VALUES_SELECTIVE`
- App Data: 20 Hz `COMM_GET_DECODED_ADC`

Perkiraan konservatif bila LEFT dan RIGHT keduanya dipoll pada rate tersebut:

- RT pair ≈ 6000 byte/s
- APP pair ≈ 1160 byte/s
- total ≈ 7160 byte/s
- kapasitas UART 115200 8N1 ≈ 11520 byte/s
- load ≈ **62.2%**

Masih tersedia margin sekitar 38% untuk forwarding overhead tambahan dan diagnostic lambat. Raw/observer/config tidak boleh dipoll 50 Hz; `debug.py` menempatkannya pada rate lambat.

## Verifikasi host/static Run25

- `tools/debug.py --self-test`: PASS
- `py_compile tools/debug.py tools/vesc.py`: PASS
- `foc_math.c` host GCC `-Wall -Wextra -Werror`: PASS
- static ADC/telemetry/app/timing contracts: 64/64 PASS
- exact project Python files: `tools/debug.py`, `tools/vesc.py`
- `/vesc_tools`: tidak ada
- `#if 0`: 0
- broken shift token: 0
- tab/trailing whitespace C/H: 0
- obvious unreferenced static function heuristic: 0
- ADC timing: HT 5.0625 µs, full sequence 32.4375 µs, margin 30.0625 µs
- UART RT50/App20 dual-motor estimated load: 62.2%

`pytest` tidak lagi mempunyai suite terpisah setelah konsolidasi Python menjadi dua file; perintah `pytest` mengembalikan “no tests ran”. Acceptance Python yang aktif adalah `debug.py --self-test` dan checker statis run ini.

## Belum diverifikasi hardware

Tanpa `pio run` dan tanpa flash Run25, hal berikut belum boleh disebut PASS:

- RAM/Flash aktual;
- register sampling contract = 0 pada board setelah boot;
- ISR max aktual <4000 cycle;
- PA2/PA3 aktual non-zero dan benar skala;
- RT response aktual ≥45 Hz dan App response ≥18 Hz pada LEFT+RIGHT;
- semua APP ADC mode menghasilkan motor response sesuai konfigurasi;
- Hall/sensorless running pada hardware.

Acceptance hardware yang disarankan setelah build/flash:

```bash
python3 tools/debug.py calibrate --timeout 8
python3 tools/debug.py stream-all --seconds 15 --csv run25_idle.csv
python3 tools/debug.py speed-test --motor 0 --erpm 300 --seconds 5 --yes --csv left_run25.csv
python3 tools/debug.py speed-test --motor 1 --erpm 300 --seconds 5 --yes --csv right_run25.csv
```

Periksa `sampling_contract_flags=0`, `isr_overruns=0`, offset mendekati raw zero-current, PA2/PA3 berubah sesuai input, dan rate test memenuhi gate.
