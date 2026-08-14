# STM32F103RCT6 Dual FOC + CMSIS-RTOS2 — V3 Runtime Sensor / Debug

Firmware ini adalah baseline dual motor untuk STM32F103RCT6 dengan:

- LEFT PWM: TIM8 CH1/2/3 + CH1N/2N/3N.
- RIGHT PWM: TIM1 CH1/2/3 + CH1N/2N/3N.
- ADC1+ADC2 dual regular simultaneous + DMA1 Channel1.
- Fast FOC langsung di DMA half-transfer ISR, bukan thread RTOS.
- LEFT sensor dapat dipilih runtime: Hall PB5/PB6/PB7 atau encoder AB TIM4 PB6/PB7.
- RIGHT sensor: Hall PC10/PC11/PC12.
- CMSIS-RTOS2 + FreeRTOS STM32CubeF1 1.8.6.
- Protokol UART framing/CRC VESC dan subset telemetry/control VESC.
- Python commissioning/debug lengkap di `debug_vesc_f103.py`.

> **Penting:** source ini tidak boleh dianggap otomatis aman untuk power stage hanya karena berhasil compile. Sebelum motor diberi arus, nilai A/count, V/count, urutan phase U/V/W, polaritas gate, dan arus nyata harus diverifikasi dengan oscilloscope/current clamp/bench supply.

## 1. Struktur ZIP

ZIP dibuat tanpa folder pembungkus. Setelah extract langsung berisi:

```text
platformio.ini
extra_script.py
debug_vesc_f103.py
requirements.txt
README_ID.md
VALIDATION.md
FIRST_POWER_TEST.md
src/
  *.c
  *.h
  FreeRTOSConfig.h
  stm32f1xx_hal_conf.h
tests/
  host/
```

Semua header firmware berada di `src/`; tidak ada folder `include/`.

## 2. Polaritas power stage yang dikunci

Kebutuhan PCB:

```text
High-side MOSFET input : HIGH = ON
Low-side  MOSFET input : LOW  = ON
```

Konfigurasi TIM1/TIM8:

```c
oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
oc.OCNPolarity  = TIM_OCNPOLARITY_LOW;
oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
oc.OCNIdleState = TIM_OCNIDLESTATE_SET;
```

Sehingga secara fisik:

```text
                 HIGH input    LOW input
High FET ON          1             1
Low  FET ON          0             0
Deadtime             0             1
MOE/PWM OFF          0             1
```

CHx/CHxN tetap complementary dan deadtime dihasilkan oleh advanced timer hardware.

## 3. Jalur real-time FOC

```text
TIM1/TIM8 center-aligned PWM 16 kHz
          |
          +--> TIM2 CC2 ADC trigger
                    |
                    v
          ADC1 + ADC2 simultaneous
                    |
                    v
             DMA1 Channel1
                    |
             HALF TRANSFER
                    |
                    v
       DMA1_Channel1_IRQHandler()
                    |
       +------------+------------+
       |                         |
       v                         v
   LEFT FOC                   RIGHT FOC
       |                         |
       + Ia/Ib                   + Ia/Ib
       + Ic=-(Ia+Ib)             + Ic=-(Ia+Ib)
       + Clarke                  + Clarke
       + rotor phase             + rotor phase
       + Park                    + Park
       + raw Id/Iq PI            + raw Id/Iq PI
       + Vd/Vq limit             + Vd/Vq limit
       + inverse Park            + inverse Park
       + centered SVM            + centered SVM
       |                         |
       v                         v
 TIM8 CCR1/2/3              TIM1 CCR1/2/3
```

Tidak ada `osDelay`, UART, `printf`, allocation, `sinf/cosf/sqrtf`, atau blocking HAL di fast ISR.

## 4. Current-zero calibration saat startup

Sebelum MOE diizinkan ON:

1. kedua power-stage output OFF;
2. 64 sample awal dibuang sebagai pipeline warm-up;
3. 4096 sample PWM-synchronous dikumpulkan;
4. mean offset dihitung untuk LEFT U/V/DC dan RIGHT U/V/DC;
5. min/max spread dan population variance divalidasi;
6. state `Ia/Ib/Ic/Id/Iq`, filter, dan PI integrator di-zero-kan;
7. jika valid barulah motor boleh enable.

Parameter ada di `src/app_config.h`:

```c
#define ADC_OFFSET_CAL_SAMPLES       4096U
#define ADC_OFFSET_MAX_SPREAD_COUNT  200U
#define ADC_OFFSET_MAX_STDDEV_COUNT  12U
#define ADC_OFFSET_TRACK_SHIFT       12U
```

Setelah startup, offset hanya di-track sangat lambat ketika motor benar-benar OFF. Offset **tidak diubah saat PWM berjalan, command aktif, atau auto-detect berjalan**.

### Filter current

Current PI memakai **Id/Iq mentah**. Filter hanya untuk telemetry/slow logic:

```text
Id raw, Iq raw --> current PI
     |
     +--> LPF --> Id_filter / Iq_filter --> telemetry/statistics
```

Default LPF:

```c
#define FOC_CURRENT_FILTER_CONST 0.10f
```

Jadi filter tidak menambah delay ke current feedback loop.

## 5. Nilai yang WAJIB disesuaikan terhadap PCB

Auto-zero mengoreksi offset, tetapi tidak dapat mengetahui gain analog secara otomatis. Nilai berikut masih harus sesuai shunt/op-amp/divider sebenarnya:

```c
#define LEFT_CURRENT_A_PER_COUNT      0.0100f
#define RIGHT_CURRENT_A_PER_COUNT     0.0100f
#define LEFT_DC_CURRENT_A_PER_COUNT   0.0100f
#define RIGHT_DC_CURRENT_A_PER_COUNT  0.0100f
#define DCLINK_V_PER_COUNT            0.0200f
```

Jika nilai ini salah, angka Ampere/Volt pada `Iq`, `Id`, `Imotor`, `Ibatt`, `Vd`, `Vq`, dan proteksi current/voltage juga salah meskipun zero-offset tepat.

## 6. Runtime Hall / Encoder — tidak ada compile-time LEFT_SENSOR_MODE

Tidak ada lagi:

```c
#if LEFT_SENSOR_MODE == SENSOR_MODE_HALL
...
#endif
```

ISR LEFT hanya mengecek state runtime:

```c
if (g_motor_left.sensor_mode == SENSOR_MODE_HALL) {
    motor_hall_edge_isr(&g_motor_left);
}
```

Jika LEFT dipilih encoder, PB6/PB7 direconfigure menjadi TIM4 encoder. Jika LEFT dipilih Hall, TIM4 dihentikan dan PB5/PB6/PB7 menjadi EXTI Hall lagi.

**Sensor tidak diganti saat PWM masih aktif.** `motor_select_sensor_mode()` selalu stop motor lebih dulu.

### Pilih live saat motor stop

```bash
python3 debug_vesc_f103.py sensor-select --port /dev/ttyUSB0 --motor 0 --mode hall
python3 debug_vesc_f103.py sensor-select --port /dev/ttyUSB0 --motor 0 --mode encoder
```

RIGHT tidak menerima mode encoder karena pin hardware RIGHT yang diberikan hanya Hall.

## 7. Sensor auto-detect runtime

Auto-detect tidak dijalankan otomatis saat power-on karena proses ini sengaja menggerakkan/menahan rotor. Jalankan eksplisit:

```bash
python3 debug_vesc_f103.py sensor-detect --port /dev/ttyUSB0 --motor 0 --mode auto --yes
python3 debug_vesc_f103.py sensor-detect --port /dev/ttyUSB0 --motor 1 --mode auto --yes
```

LEFT AUTO:

```text
Hall detect
   |
   +-- success --> HALL
   |
   `-- fail --> TIM4 encoder detect
                  |
                  +-- success --> ENCODER
                  `-- fail --> fault
```

RIGHT AUTO:

```text
Hall detect --> success/fail
```

### Hall detect

- lock rotor pada electrical phase 0 dengan D-axis current kecil;
- sweep electrical forward 0..360 sebanyak 3 kali;
- sweep reverse 360..0 sebanyak 3 kali;
- sample raw Hall pada setiap langkah;
- circular averaging dengan sin/cos;
- hanya terima 6 Hall states dengan sample cukup;
- bangun `foc_hall_table[8]` 0..200 dan sudut u16 aktual.

### Encoder detect LEFT

- remux PB6/PB7 ke TIM4;
- lock electrical phase 0;
- reset encoder count;
- sweep 3 electrical revolutions;
- hitung arah encoder dan estimasi pole-pair dari jumlah encoder count;
- gunakan aligned count sebagai electrical-zero sesi tersebut.

Untuk incremental AB tanpa index/absolute reference, alignment perlu dilakukan setelah power cycle sebelum closed-loop position absolut dianggap valid.

## 8. Thread yang digunakan

Semua thread yang diminta ada sebagai CMSIS-RTOS2 thread nyata:

```text
timer_thread        stack 512  1 kHz
pid_thread          stack 512  1 kHz
sample_send_thread  stack 512  event-driven
fault_stop_thread   stack 512  event-driven/high priority
stat_thread         stack 512  100 Hz
vesc_comm_thread    stack 768  UART/event-driven
```

Fungsi:

- `timer_thread`: sensor detect state machine, motor housekeeping, timeout, fault dispatch.
- `pid_thread`: speed PID dan position cascade.
- `sample_send_thread`: mengirim buffer debug setelah ISR selesai merekam.
- `fault_stop_thread`: memastikan PWM off dan event fault ditangani di thread.
- `stat_thread`: energy/tachometer/telemetry snapshot dan rotor stream 100 Hz.
- `vesc_comm_thread`: UART framing, CRC, command parser.

Fast current FOC tetap **bukan thread**.

## 9. Telemetry VESC

Firmware menggunakan framing VESC + CRC16 dan menangani subset command standar:

```text
COMM_FW_VERSION
COMM_GET_VALUES
COMM_GET_VALUES_SETUP
COMM_GET_VALUES_SELECTIVE
COMM_SET_DUTY
COMM_SET_CURRENT
COMM_SET_CURRENT_BRAKE
COMM_SET_RPM
COMM_SET_POS
COMM_SET_DETECT
COMM_ROTOR_POSITION
COMM_DETECT_ENCODER
COMM_DETECT_HALL_FOC
COMM_REBOOT
COMM_ALIVE
COMM_CUSTOM_APP_DATA
```

`COMM_GET_VALUES` / selective menyediakan field:

```text
Temp FET       = 0 (tidak ada NTC pada scope hardware ini)
Temp motor     = 0
Imotor
Ibatt
Id
Iq
Duty
ERPM
Vin
Ah
Ah charged
Wh
Wh charged
Tachometer
Tachometer abs
Fault
PID/rotor position
Controller ID
3x MOS temp    = 0
Vd
Vq
Timeout status
```

`Id/Iq` standar memakai filtered/averaged display value; extended debug juga mengirim raw Id/Iq.

`Imotor` dihitung dari magnitude D/Q current dengan tanda arah daya. `Ibatt` berasal dari sensor DC-current motor masing-masing.

### Rotor Position VESC Tool

`COMM_SET_DETECT` mengubah display mode runtime dan `stat_thread` mengirim:

```text
COMM_ROTOR_POSITION + int32(position * 100000)
```

sekitar 100 Hz.

## 10. Debug Python

Install:

```bash
python3 -m pip install -r requirements.txt
```

Self-test tanpa hardware:

```bash
python3 debug_vesc_f103.py --self-test
```

Info:

```bash
python3 debug_vesc_f103.py info --port /dev/ttyUSB0
```

Passive telemetry:

```bash
python3 debug_vesc_f103.py status --port /dev/ttyUSB0
python3 debug_vesc_f103.py monitor --port /dev/ttyUSB0 --seconds 20
python3 debug_vesc_f103.py test-all --port /dev/ttyUSB0
```

Re-zero current:

```bash
python3 debug_vesc_f103.py calibrate --port /dev/ttyUSB0
```

Sensor:

```bash
python3 debug_vesc_f103.py sensor-info --port /dev/ttyUSB0
python3 debug_vesc_f103.py sensor-detect --port /dev/ttyUSB0 --motor 0 --mode auto --yes
python3 debug_vesc_f103.py sensor-detect --port /dev/ttyUSB0 --motor 0 --mode encoder --yes
python3 debug_vesc_f103.py sensor-detect --port /dev/ttyUSB0 --motor 1 --mode hall --yes
```

Rotor stream:

```bash
python3 debug_vesc_f103.py rotor --port /dev/ttyUSB0 --motor 0 --mode encoder --seconds 10
python3 debug_vesc_f103.py rotor --port /dev/ttyUSB0 --motor 1 --mode obs-hall --seconds 10
```

Fast ISR samples:

```bash
python3 debug_vesc_f103.py sample --port /dev/ttyUSB0 --motor 0 --count 256 --decimation 8 --csv left.csv
```

Motor tests (ACTIVE; harus `--yes`):

```bash
python3 debug_vesc_f103.py motor-test --port /dev/ttyUSB0 --motor 0 --mode current  --value 0.5  --seconds 2 --yes
python3 debug_vesc_f103.py motor-test --port /dev/ttyUSB0 --motor 0 --mode current  --value -0.5 --seconds 2 --yes
python3 debug_vesc_f103.py motor-test --port /dev/ttyUSB0 --motor 0 --mode rpm      --value 300   --seconds 2 --yes
python3 debug_vesc_f103.py motor-test --port /dev/ttyUSB0 --motor 0 --mode rpm      --value -300  --seconds 2 --yes
python3 debug_vesc_f103.py motor-test --port /dev/ttyUSB0 --motor 0 --mode position --value 10    --seconds 2 --yes
```

Full automated commissioning test (ACTIVE):

```bash
python3 debug_vesc_f103.py full-test --port /dev/ttyUSB0 --yes \
  --current 0.5 --erpm 300 --stage-seconds 1
```

Urutannya: stop/clear -> current zero -> sensor auto-detect kedua motor -> telemetry -> ISR sample -> current +/- -> RPM +/- -> LEFT position bila encoder -> final fault check.

Jika mekanik belum aman bergerak, gunakan:

```bash
python3 debug_vesc_f103.py full-test --port /dev/ttyUSB0 --yes --skip-rpm --position-step 0
```

## 11. Build

```bash
pio run -t clean
pio run
```

Upload setelah build sukses:

```bash
pio run -t upload
```

`platformio.ini` mempertahankan warning penting milik source, tetapi menonaktifkan noise `unused-parameter`/`pedantic` dari STM32CubeF1 third-party sources. `return-type` dan implicit function declaration tetap dijadikan error.

Host test:

```bash
tests/host/run.sh
```

## 12. Perbedaan sensor autodetect vs full VESC Detect All FOC

Firmware ini mengimplementasikan **runtime Hall/encoder sensor detection** dan command standar `COMM_DETECT_HALL_FOC` / `COMM_DETECT_ENCODER`.

Firmware ini **belum mengimplementasikan seluruh `COMM_DETECT_APPLY_ALL_FOC` VESC** untuk otomatis mengukur motor R, L, Ld-Lq, flux linkage, safe Imax, current Kp/Ki, dan observer gain. Jangan menganggap sensor auto-detect sebagai pengganti full motor-parameter identification tersebut.

Current Kp/Ki pada `app_config.h` harus dituning/diidentifikasi terhadap motor sebelum arus dan bandwidth dinaikkan.
