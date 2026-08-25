# MASTER PROMPT — AUDIT, DIFF, PERBAIKAN, DAN PENYEMPURNAAN FIRMWARE VESC STM32F103RCT6 HOVERBOARD

Saya ingin Anda melakukan **audit menyeluruh, perbandingan, perbaikan, refactoring, implementasi fitur yang belum lengkap, serta penyempurnaan terhadap seluruh source code di dalam file ZIP project firmware yang saya berikan**.

Target akhirnya adalah menghasilkan firmware motor controller berbasis:

**STM32F103RCT6 Hoverboard Controller**

yang mengadopsi algoritma, workflow, protocol, control architecture, calibration, telemetry, safety, dan application layer VESC yang relevan, tetapi tetap disesuaikan dengan keterbatasan resource serta karakteristik hardware hoverboard.

Firmware harus:

- aman,
- deterministic,
- efisien,
- clean,
- tidak memiliki dead code,
- tidak mempunyai implementasi dummy,
- mempunyai FOC yang valid,
- mendukung Hall/Encoder/Sensorless sesuai konfigurasi motor,
- mempunyai APP ADC,
- mempunyai APP UART,
- kompatibel dengan command VESC Tool yang relevan,
- mempunyai calibration dan auto-detection,
- mempunyai EEPROM/configuration persistence,
- telemetry aktual,
- fault handling,
- watchdog/failsafe,
- LED/buzzer fault indication,
- serta siap diuji pada hardware nyata.

---

# 1. WAJIB MEMBACA SELURUH PROJECT

Sebelum melakukan perubahan apa pun:

- Ekstrak dan baca **seluruh source code di dalam ZIP**.
- Jangan hanya membaca `main.c`, `bldc.c`, atau file utama.
- Jangan langsung patch sebelum memahami arsitektur keseluruhan.

Audit seluruh:

- `.c`
- `.h`
- build configuration
- `platformio.ini`
- linker/configuration file
- startup code
- HAL/LL configuration
- FreeRTOS/CMSIS-RTOS2
- task
- queue
- semaphore
- mutex
- event
- notification
- ISR
- ADC
- PWM
- timer
- DMA
- UART
- packet protocol
- motor control
- FOC
- sensor
- Hall
- Encoder
- Sensorless
- telemetry
- EEPROM/configuration
- calibration
- auto-detection
- fault handling
- buzzer
- LED
- watchdog
- APP ADC
- APP UART

Pahami hubungan antarfile dan alur program secara keseluruhan.

Cari seluruh fungsi yang:

- belum selesai,
- masih stub,
- placeholder,
- hanya return sukses,
- tidak pernah dipanggil,
- dead code,
- duplicate implementation,
- memiliki dependency tidak perlu,
- salah integrasi,
- salah state transition,
- berpotensi race condition,
- berpotensi buffer overflow,
- integer overflow,
- fixed-point overflow,
- salah interrupt priority,
- salah penggunaan `volatile`,
- salah concurrency,
- atau tidak sesuai hardware.

**Jangan menganggap fitur sudah bekerja hanya karena fungsi atau variabelnya tersedia.**

Telusuri sampai jalur eksekusi aktual:

**input → processing → control → hardware output → telemetry/status**

---

# 2. REFERENSI UTAMA

Gunakan dua repository berikut sebagai referensi utama.

## 2.1 Referensi Hardware Hoverboard

https://github.com/EFeru/hoverboard-firmware-hack-FOC

Gunakan terutama untuk memahami:

- STM32F103RCT6
- clock
- GPIO
- TIM1
- TIM8
- complementary PWM
- dead-time
- ADC
- current measurement
- phase-current mapping
- gate control
- MOSFET polarity
- Hall input
- interrupt
- hardware mapping
- power-stage behavior

Jadikan implementasi hardware yang memang sudah terbukti pada hoverboard sebagai basis.

---

## 2.2 Referensi Algoritma VESC

https://github.com/vedderb/bldc

Lakukan **diff konseptual dan implementasi secara mendalam**.

Jangan hanya melakukan diff nama file/fungsi.

Bandingkan:

- FOC
- current controller
- duty controller
- speed controller
- position controller
- Hall
- Encoder
- Sensorless
- observer
- PLL
- motor parameter detection
- Hall detection
- Encoder calibration
- motor auto-detection
- application layer
- APP ADC
- APP UART
- packet protocol
- command handling
- telemetry
- motor configuration
- application configuration
- EEPROM/config persistence
- fault handling
- timeout
- watchdog
- RTOS architecture
- startup/shutdown sequence

Identifikasi:

1. fitur yang sudah benar,
2. fitur yang hanya sebagian diterapkan,
3. fitur yang salah,
4. fitur yang belum ada,
5. fitur VESC yang memang tidak relevan.

---

# 3. TARGET HARDWARE

Target:

**STM32F103RCT6 Hoverboard Controller**

Konfigurasi motor:

## MOTOR LEFT

Motor Left harus mendukung pilihan:

- Hall sensor
- Encoder A/B
- Sensorless tanpa HFI

Pemilihan harus dilakukan melalui configuration.

Tidak boleh membutuhkan edit source code dan compile ulang hanya untuk mengganti sensor.

---

## MOTOR RIGHT

Motor Right harus mendukung:

- Hall sensor
- Sensorless tanpa HFI

Motor Right **tidak membutuhkan Encoder**.

Pastikan Motor Left dan Motor Right:

- independen,
- mempunyai state sendiri,
- mempunyai fault sendiri,
- mempunyai sensor state sendiri,
- mempunyai control state sendiri,
- tidak merusak variabel satu sama lain.

---

# 4. MOSFET DAN PWM POLARITY WAJIB BENAR

Pastikan:

**High-side MOSFET = Active HIGH**

**Low-side MOSFET = Active LOW**

Audit khusus:

- TIM1
- TIM8
- PWM polarity
- complementary output
- CHx
- CHxN
- idle state
- break state
- dead-time
- MOE
- preload
- CCR update
- ARR
- center-aligned PWM
- ADC trigger
- startup state
- shutdown state

Pastikan tidak ada kondisi yang dapat menyebabkan:

- shoot-through,
- kedua transistor satu leg aktif bersamaan,
- output floating tidak aman,
- PWM aktif sebelum ADC offset valid.

Pada:

- boot,
- reset,
- calibration,
- watchdog reset,
- HardFault,
- over-current,
- under-voltage,
- over-voltage,
- sensor fault,
- communication timeout,

power stage harus masuk kondisi aman.

---

# 5. MODUL YANG TIDAK DIGUNAKAN

Firmware ini tidak membutuhkan:

- HFI
- CAN hardware
- IMU
- BMS
- bm_if
- NRF
- LEDPWM
- COMUSB
- QMLUI
- LispIF
- NTC temperature
- LZO

Hilangkan dependency terhadap modul tersebut apabila memang tidak digunakan.

Setelah removal:

- tidak boleh ada unresolved symbol,
- tidak boleh ada include yatim,
- tidak boleh ada unused task,
- tidak boleh ada buffer tidak digunakan,
- tidak boleh ada fake interface,
- tidak boleh ada conditional compilation berlebihan,
- RAM/Flash harus berkurang atau minimal tidak bertambah sia-sia.

Jangan menghapus bagian VESC hanya berdasarkan nama modul.

Pastikan fungsi tersebut benar-benar tidak dibutuhkan oleh dependency lain.

---

# 6. CONTROL MODE WAJIB LENGKAP

Firmware harus mendukung minimal:

## SET

- SET DUTY
- SET CURRENT
- SET CURRENT BRAKE
- SET FULL BRAKE CURRENT jika relevan
- SET HANDBRAKE jika relevan
- SET RPM
- SET POSITION
- STOP
- ALIVE / KEEPALIVE

## GET

- duty
- current
- input current
- Id
- Iq
- RPM
- position
- electrical position
- mechanical position
- sensor state
- control mode
- fault
- motor configuration
- application configuration
- telemetry

Control mode utama:

- Duty Control
- Current Control
- Brake Current
- Speed/RPM Control
- Position Control

Pastikan perpindahan mode mempunyai:

- integrator reset,
- anti-windup,
- bumpless transfer bila memungkinkan,
- output clamp,
- ramping,
- safe state,
- state validation,
- communication timeout.

Motor tidak boleh melonjak ketika mode diganti.

---

# 7. FOC MOTOR CONTROL

Audit dan sempurnakan pipeline:

1. ADC sampling
2. current offset correction
3. current scaling
4. phase-current reconstruction
5. Clarke Transform
6. Park Transform
7. Id reference
8. Iq reference
9. Id PI
10. Iq PI
11. anti-windup
12. decoupling bila relevan
13. voltage limiting
14. modulation limiting
15. field weakening bila benar-benar diperlukan
16. inverse Park
17. SVPWM
18. CCR update
19. PWM synchronization
20. electrical-angle update

Pastikan matematikanya benar.

Audit:

- sign convention,
- phase order,
- current polarity,
- angle direction,
- electrical-zero convention,
- saturation,
- normalization,
- modulation index.

---

# 8. FIXED-POINT OPTIMIZATION

STM32F103RCT6 tidak memiliki hardware FPU.

Optimalkan jalur FOC ISR menggunakan:

- fixed-point,
- LUT,
- integer arithmetic,
- precomputed coefficient,

jika terbukti lebih efisien.

Jangan sekadar mengubah `float` menjadi integer.

Audit:

- Q-format,
- scaling,
- multiplication shift,
- division,
- saturation,
- sign extension,
- rounding,
- overflow,
- underflow,
- accumulator size.

Gunakan tipe yang jelas seperti:

- `int16_t`
- `uint16_t`
- `int32_t`
- `uint32_t`
- `int64_t`

sesuai kebutuhan.

Pastikan optimasi tidak merusak kestabilan FOC.

---

# 9. ADC CURRENT SENSING DAN PWM ISR

ADC current sensing dan FOC ISR adalah bagian paling kritis.

Pastikan:

- ADC sinkron PWM,
- titik sampling aman,
- current sampling tidak berada pada switching edge,
- ADC channel benar,
- DMA benar,
- conversion sequence benar,
- sampling time benar,
- offset calibration valid,
- current scaling valid,
- tidak ada stale data.

FOC ISR harus:

- deterministic,
- execution time stabil,
- tidak menggunakan blocking operation,
- tidak melakukan EEPROM write,
- tidak melakukan packet parsing,
- tidak melakukan logging berat.

Audit:

- IRQ priority
- DMA priority
- shared variable
- `volatile`
- atomic access
- race condition
- nesting
- ISR execution time

---

# 10. CURRENT OFFSET CALIBRATION

Pada startup:

1. PWM power stage harus aman.
2. Tidak ada current command.
3. Sampling ADC dilakukan beberapa kali.
4. Hitung offset setiap current channel.
5. Buang outlier bila diperlukan.
6. Validasi range.
7. Simpan offset runtime.
8. Jangan enable motor jika calibration gagal.

Tambahkan fault:

**CURRENT_OFFSET_CALIBRATION_FAULT**

jika hasil tidak masuk range yang masuk akal.

---

# 11. SENSOR HALL

Hall harus lengkap.

Implementasikan:

- raw Hall state
- Hall sequence
- illegal state 000
- illegal state 111
- Hall transition validation
- direction detection
- Hall table
- Hall interpolation
- Hall electrical angle
- Hall RPM
- Hall calibration
- Hall fault detection

Pastikan tidak hanya membaca Hall tetapi benar-benar digunakan FOC.

Hall calibration harus mampu menentukan mapping aktual terhadap electrical sector.

---

# 12. ENCODER MOTOR LEFT

Motor Left harus mendukung Encoder A/B.

Implementasikan:

- hardware timer encoder mode bila sesuai,
- quadrature A/B,
- direction,
- counter,
- overflow,
- wraparound,
- CPR,
- PPR,
- mechanical position,
- mechanical velocity,
- pole pair conversion,
- electrical angle,
- electrical offset,
- direction inversion,
- calibration.

Jika index tidak tersedia, jangan membuat index palsu.

Pastikan posisi benar melewati:

- 0°
- 360°
- multiple rotation
- counter overflow
- reverse motion.

---

# 13. ENCODER ELECTRICAL CALIBRATION

Calibration harus mampu menentukan:

**Mechanical Encoder Angle ↔ Electrical Rotor Angle**

Gunakan open-loop electrical excitation/rotation yang aman.

Contoh workflow:

1. Disable closed-loop.
2. Set current calibration rendah.
3. Inject electrical vector.
4. Sweep electrical angle.
5. Baca encoder.
6. Tentukan direction.
7. Tentukan pole-pair relationship.
8. Tentukan electrical offset.
9. Ulangi untuk validasi.
10. Hitung error.
11. Reject jika hasil tidak konsisten.
12. Simpan jika valid.

Jangan mengisi offset dengan konstanta default lalu menyatakan calibration sukses.

---

# 14. SENSORLESS TANPA HFI

Sensorless tetap harus tersedia tanpa HFI.

Gunakan algoritma VESC yang relevan dan cukup ringan.

Audit:

- flux observer,
- back-EMF observer,
- observer state,
- observer gain,
- PLL,
- angle estimation,
- RPM estimation,
- open-loop startup,
- observer lock,
- transition.

Workflow:

**Standstill**

→ **Open-loop startup**

→ **Observer mulai valid**

→ **Angle blending**

→ **Closed-loop sensorless**

Pastikan ada validasi observer.

Jangan langsung switch angle secara kasar.

---

# 15. MOTOR AUTO-DETECTION ALA VESC

Implementasikan motor auto-detection sedekat mungkin dengan workflow VESC Tool, tetapi realistis untuk STM32F103RCT6.

Minimal mampu menentukan:

- phase resistance
- phase inductance
- flux linkage
- pole pairs bila memungkinkan
- sensor type
- Hall mapping
- Hall direction
- Encoder direction
- Encoder offset
- sensor validity
- current offset

Workflow harus mempunyai state machine seperti:

**IDLE**

→ **PRECHECK**

→ **CURRENT OFFSET**

→ **RESISTANCE DETECTION**

→ **INDUCTANCE DETECTION**

→ **FLUX DETECTION**

→ **SENSOR DETECTION**

→ **HALL/ENCODER CALIBRATION**

→ **VALIDATION**

→ **SAVE CONFIG**

→ **DONE**

atau equivalent.

Calibration tidak boleh blocking lama pada task kritis.

---

# 16. VALIDASI AUTO-DETECTION

Setiap hasil detection harus mempunyai:

- min/max sanity range,
- timeout,
- repeatability check,
- sensor validity,
- motor movement validation,
- emergency abort,
- fault output.

Jika salah satu tahap gagal:

- hentikan PWM dengan aman,
- jangan menyimpan parameter invalid,
- laporkan tahap yang gagal.

---

# 17. EEPROM / FLASH PERSISTENT CONFIGURATION

Audit penyimpanan konfigurasi.

Simpan minimal:

## Motor Configuration

- motor type
- sensor mode
- current limits
- brake current limits
- voltage limits
- duty limits
- RPM limits
- pole pairs
- resistance
- inductance
- flux linkage
- Hall table
- Hall direction
- encoder CPR/PPR
- encoder direction
- encoder offset
- PI current
- PID speed
- PID position

## Application Configuration

- selected application
- ADC settings
- UART settings
- timeout
- ramp
- safety parameters

Gunakan validasi:

- magic number
- version
- structure length
- CRC/checksum

Jika configuration corrupt:

- jangan digunakan,
- load safe default,
- set CONFIGURATION_FAULT,
- jangan meng-enable motor secara sembarangan.

Perhatikan Flash endurance.

Jangan menulis setiap loop.

---

# 18. VESC TOOL COMMUNICATION

Firmware harus kompatibel dengan sebanyak mungkin bagian VESC Tool yang relevan.

Audit berdasarkan protocol pada:

https://github.com/vedderb/bldc

Minimal dukung:

- firmware identification
- GET_FW_VERSION
- GET_VALUES
- GET_VALUES_SELECTIVE
- SET_DUTY
- SET_CURRENT
- SET_CURRENT_BRAKE
- SET_RPM
- SET_POS
- SET_HANDBRAKE bila digunakan
- ALIVE
- GET_MCCONF
- SET_MCCONF
- GET_APPCONF
- SET_APPCONF
- motor detection
- Hall detection
- Encoder detection/calibration
- read config
- write config
- fault reporting

Command ID harus mengikuti protocol yang dipilih.

**Jangan membuat ID command sendiri lalu mengklaim kompatibel VESC.**

Jika command tidak didukung:

- return unsupported secara jelas,
- atau disable dengan bersih.

Jangan mengirim fake success.

---

# 19. VESC PACKET PROTOCOL

Audit seluruh packet layer.

Pastikan mendukung mekanisme yang relevan:

- start byte
- payload length
- short packet
- long packet bila dibutuhkan
- CRC
- stop byte
- packet state machine

Harus tahan terhadap:

- partial packet,
- fragmented packet,
- multiple packet,
- back-to-back packet,
- wrong CRC,
- wrong length,
- random byte,
- buffer overflow,
- communication interruption.

Parser tidak boleh menyebabkan:

- heap corruption,
- stack overflow,
- out-of-bounds memory access.

---

# 20. TELEMETRY VESC TOOL

Semua telemetry harus berasal dari data nyata.

Minimal:

- input voltage
- motor current
- input/battery current
- Id
- Iq
- duty
- RPM
- mechanical position
- electrical position
- Hall state
- Encoder state
- sensor mode
- control mode
- fault
- motor state
- command source
- APP ADC input jika aktif

Temperature hanya boleh dikirim jika sensor hardware memang tersedia.

Karena NTC temperature yang tidak digunakan dihapus:

**jangan membuat temperature dummy.**

---

# 21. APP ADC WAJIB SIAP DIGUNAKAN

Selain ADC current sensing FOC, firmware harus mempunyai **APP ADC** sebagai application input.

APP ADC berbeda dengan ADC current sensing.

Pisahkan secara jelas:

### ADC FOC

Digunakan untuk:

- phase current
- DC current/voltage bila tersedia
- FOC feedback

### APP ADC

Digunakan untuk:

- throttle
- accelerator
- potentiometer
- brake analog
- external analog command

APP ADC tidak boleh mengganggu timing ADC FOC.

---

# 22. APP ADC INPUT PIPELINE

Gunakan alur:

**ADC raw**

→ **Voltage Conversion**

→ **Filtering**

→ **Calibration**

→ **Range Validation**

→ **Deadband**

→ **Normalization**

→ **Throttle Curve**

→ **Command Mapping**

→ **Command Arbitration**

→ **Motor Control API**

Tidak boleh:

**APP ADC → langsung PWM register**

---

# 23. APP ADC CALIBRATION

Minimal parameter:

- ADC minimum
- ADC maximum
- ADC center jika diperlukan
- deadband
- hysteresis
- valid minimum voltage
- valid maximum voltage
- inversion
- filter
- ramp positive
- ramp negative

Normalized output dapat berupa:

**0.0 – 1.0**

atau:

**-1.0 – +1.0**

sesuai mode.

Semua output harus di-saturate.

---

# 24. APP ADC CONTROL MODE

APP ADC minimal dapat dikonfigurasi menghasilkan:

- Duty
- Current
- Current + Brake
- RPM

Position dapat tersedia jika masuk akal.

Contoh:

**Throttle ADC**

→ `normalized_throttle`

→ `Iq_ref`

→ Current Controller

atau:

**Throttle ADC**

→ Duty Reference

→ Duty Controller

Semua tetap harus melalui safety layer.

---

# 25. APP ADC SAFE START

Jika throttle sudah aktif saat controller dinyalakan:

**motor tidak boleh langsung bergerak.**

Gunakan state:

**DISARMED**

→ baca throttle

→ throttle berada pada neutral/minimum

→ stabil selama waktu tertentu

→ **ARMED**

→ command diperbolehkan.

Jika throttle tidak kembali neutral:

- tetap DISARMED,
- motor tidak bergerak,
- status tersedia di telemetry.

---

# 26. APP ADC FAULT

Deteksi minimal:

- ADC terlalu rendah
- ADC terlalu tinggi
- short-to-GND
- short-to-VCC
- disconnected signal jika dapat dideteksi
- invalid calibration
- implausible transition
- throttle startup active

Jika invalid:

- command = safe,
- APP ADC invalid,
- fault/status tercatat.

---

# 27. APP ADC THROTTLE CURVE

Jika resource memungkinkan, sediakan:

- linear
- exponential
- configurable curve

Tetapi jangan menambah operasi berat pada loop cepat.

Parameter disimpan dalam Application Configuration.

---

# 28. DUAL ADC THROTTLE + BRAKE

Jika tersedia dua channel ADC aplikasi:

- ADC throttle
- ADC brake

Brake harus memiliki arbitration yang jelas terhadap throttle.

Contoh:

**Brake active**

→ throttle dikurangi / dibatalkan

→ brake current command aktif.

Tidak boleh terjadi konflik throttle dan brake tanpa rule.

---

# 29. APP UART WAJIB SIAP DIGUNAKAN

Implementasikan APP UART yang production-ready.

Gunakan UART hardware yang memang tersedia pada project.

Prioritas:

**USART3**

- TX = PB10
- RX = PB11

jika mapping hardware project memang sesuai.

APP UART harus tetap stabil saat:

- FOC ISR aktif,
- kedua motor berjalan,
- ADC sampling aktif,
- telemetry dikirim,
- configuration diubah,
- calibration berjalan,
- fault monitoring aktif.

---

# 30. UART LOW-LEVEL DRIVER

Audit:

- GPIO alternate function
- baud rate
- BRR
- clock source
- RX
- TX
- IRQ
- DMA
- IDLE detection
- buffer
- circular buffer
- framing error
- parity error
- overrun
- noise
- timeout

UART error tidak boleh menyebabkan:

- deadlock,
- corrupted packet,
- HardFault,
- stale command,
- uncontrolled motor.

---

# 31. UART RX ARCHITECTURE

Gunakan metode efisien seperti:

**USART3 RX DMA Circular + IDLE Detection**

jika sesuai hardware/software architecture.

ISR hanya menangani:

- status,
- DMA pointer,
- RX boundary,
- RTOS notification.

Jangan parsing seluruh VESC packet di ISR.

Parsing dilakukan pada communication task.

---

# 32. UART TX ARCHITECTURE

TX harus:

- non-blocking,
- DMA jika sesuai,
- mempunyai TX queue,
- mempunyai busy state,
- mempunyai packet boundary,
- mempunyai timeout/recovery.

Jangan menggunakan busy-wait panjang.

Jangan membiarkan telemetry packet overwrite command response.

---

# 33. APP UART COMMAND

APP UART harus mampu menjalankan command aktual seperti:

- GET_FW_VERSION
- GET_VALUES
- GET_VALUES_SELECTIVE
- SET_DUTY
- SET_CURRENT
- SET_CURRENT_BRAKE
- SET_RPM
- SET_POS
- SET_HANDBRAKE jika digunakan
- ALIVE
- GET_MCCONF
- SET_MCCONF
- GET_APPCONF
- SET_APPCONF
- motor detection
- Hall detection
- Encoder calibration
- fault query
- diagnostic command relevan

Gunakan command ID VESC asli yang kompatibel dengan protocol yang digunakan.

---

# 34. APP UART FAILSAFE

APP UART wajib mempunyai timeout.

Contoh:

Host mengirim:

**SET_CURRENT**

kemudian komunikasi hilang.

Setelah timeout:

1. current reference diramp/down ke safe state jika diperlukan,
2. output motor dihentikan,
3. controller tetap hidup,
4. fault/status communication timeout tercatat jika sesuai.

`COMM_ALIVE` atau packet valid yang ditentukan architecture dapat memperbarui watchdog communication.

---

# 35. COMMAND SOURCE ARBITRATION

Motor dapat menerima command dari:

- APP ADC
- APP UART
- VESC Tool melalui UART
- calibration
- auto-detection
- internal test

Buat **Command Arbitration Layer**.

Minimal source state:

- APP_NONE
- APP_ADC
- APP_UART
- CALIBRATION
- MOTOR_DETECTION
- INTERNAL_TEST

atau equivalent.

Tidak boleh dua source mengubah reference motor tanpa rule.

---

# 36. CENTRAL MOTOR CONTROL API

Semua command harus melewati satu layer yang terpusat.

Arsitektur:

**APP ADC / APP UART / VESC Command**

↓

**Command Arbitration**

↓

**Motor Control API**

↓

**Safety + Limit**

↓

**Duty / Current / RPM / Position Controller**

↓

**FOC**

↓

**SVPWM**

↓

**TIM1/TIM8 PWM**

Dengan begitu semua command mendapatkan:

- current limit
- voltage limit
- duty limit
- RPM limit
- ramp
- sensor validation
- fault protection
- timeout.

---

# 37. APPLICATION CONFIGURATION

Buat `app_configuration` atau structure equivalent.

Minimal:

## General

- selected application
- command timeout
- motor target jika relevan

## APP ADC

- control mode
- ADC min
- ADC max
- ADC center
- deadband
- hysteresis
- invert
- filtering
- curve
- ramp positive
- ramp negative
- current max
- brake current max
- RPM max
- safe-start

## APP UART

- baud rate
- timeout
- telemetry rate
- packet configuration jika relevan

Application Configuration harus ikut:

- versioning
- CRC
- persistent storage.

---

# 38. DUAL MOTOR COMMAND

Pastikan protocol dapat menentukan motor target dengan jelas.

Untuk setiap motor, telemetry dan command tidak boleh ambigu.

Audit mekanisme pemilihan:

- LEFT
- RIGHT

atau controller-ID/forward mechanism equivalent.

Pastikan:

- Left command tidak mengubah Right,
- Right command tidak mengubah Left,
- telemetry dapat mengidentifikasi motor.

---

# 39. PID SPEED CONTROL

Speed controller harus memiliki:

- RPM reference
- RPM actual
- error
- Kp
- Ki
- Kd jika digunakan
- integrator limit
- anti-windup
- output current limit
- acceleration/deceleration ramp

Output speed controller sebaiknya menjadi:

**Iq_ref**

bukan PWM langsung.

---

# 40. POSITION CONTROL

Untuk Encoder Left:

**Position Reference**

↓

**Position Controller**

↓

**Speed Reference atau Iq Reference**

↓

**FOC**

Implementasikan:

- angle normalization
- shortest-path jika appropriate
- position error
- Kp
- Ki jika digunakan
- Kd jika digunakan
- output limit
- anti-windup
- position tolerance.

Jika position control digunakan pada steering:

hindari automatic wrap yang membuat steering memilih arah rotasi salah.

---

# 41. BRAKE CONTROL

Pastikan ada distinction:

- coast
- zero current
- regenerative brake current
- full brake
- handbrake bila relevan

Brake current tidak boleh mempunyai sign salah.

Audit regen terhadap:

- battery voltage,
- maximum battery charging current,
- over-voltage protection.

---

# 42. FAULT SYSTEM

Gunakan centralized fault manager.

Minimal fault:

- OVER_VOLTAGE
- UNDER_VOLTAGE
- OVER_CURRENT
- ADC_FAULT
- CURRENT_OFFSET_FAULT
- DRIVER_FAULT jika tersedia
- HALL_FAULT
- ENCODER_FAULT
- SENSORLESS_OBSERVER_FAULT
- CALIBRATION_FAULT
- MOTOR_DETECTION_FAULT
- CONFIGURATION_FAULT
- APP_ADC_FAULT
- UART_FAULT
- COMMUNICATION_TIMEOUT
- WATCHDOG_FAULT
- SYSTEM_FAULT

Saat fault:

1. zero command,
2. stop control integrator,
3. disable PWM,
4. disable MOE jika diperlukan,
5. simpan fault code,
6. telemetry update,
7. LED fault indication,
8. buzzer fault indication.

---

# 43. LED DAN BUZZER FAULT CODE

LED dan buzzer harus mempunyai fault code sama.

Contoh Fault 3:

**blink/beep – blink/beep – blink/beep – pause – repeat**

Jangan gunakan blocking delay.

Gunakan:

- state machine,
- software timer,
- RTOS timer,
- atau periodic task.

Fault prioritas tertinggi harus menentukan pattern yang aktif.

---

# 44. POWER-ON MELODY

Tambahkan power-on melody menggunakan buzzer.

Melody:

- singkat,
- terdengar seperti BLDC/VESC startup,
- non-blocking,
- tidak menghambat boot,
- tidak menghambat ADC calibration,
- tidak mengaktifkan motor.

Jika fault muncul pada startup:

**batalkan melody dan langsung tampilkan fault pattern.**

---

# 45. LED STATUS

Selain fault, LED dapat menunjukkan:

- boot,
- ready,
- calibration,
- motor detection,
- communication,
- fault,

tetapi jangan membuat pola terlalu kompleks.

Prioritas LED:

**FAULT > CALIBRATION > READY > COMMUNICATION**

atau equivalent.

---

# 46. WATCHDOG

Implementasikan MCU watchdog jika sesuai.

Jangan refresh watchdog dari satu unconditional loop.

Buat health monitoring.

Refresh hanya jika critical subsystem sehat:

- FOC ISR hidup,
- control task hidup,
- communication task tidak deadlock,
- fault manager hidup.

---

# 47. COMMAND TIMEOUT

Command motor harus mempunyai freshness timestamp.

Jika timeout:

- duty/current/RPM/position command tidak boleh tetap aktif selamanya,
- motor masuk state aman.

APP ADC lokal dapat memiliki rule timeout berbeda karena tidak bergantung external serial host.

---

# 48. FreeRTOS / CMSIS-RTOS2

Audit seluruh RTOS.

Pisahkan:

## Hard Real-Time ISR

- ADC current sample
- FOC
- angle update
- current controller
- PWM update

## Fast Sensor

- Hall
- Encoder
- Sensorless observer

## APP ADC Task

- sampling application ADC
- filtering
- calibration
- throttle processing
- safe-start
- command generation

## UART RX

- DMA
- IDLE IRQ ringan

## Communication Task

- packet parsing
- command execution
- response preparation

## Telemetry Task

- telemetry snapshot
- packet generation

## Calibration Task

- state machine calibration
- detection

## Configuration Task

- Flash/EEPROM operation

## Fault Task

- monitoring
- fault state

## Indicator Task

- buzzer
- LED
- melody

Audit:

- priority
- period
- stack
- queue
- mutex
- semaphore
- event flags
- task notification
- priority inversion
- starvation.

---

# 49. RTOS EFFICIENCY

Tidak boleh ada pada task kritis:

- `HAL_Delay()`
- long busy wait
- blocking serial TX
- polling tanpa timeout
- EEPROM write di ISR
- dynamic allocation terus-menerus
- printf berat di high priority task.

Gunakan static allocation bila memungkinkan.

Ukur/estimasi stack setiap task.

---

# 50. TELEMETRY SNAPSHOT

Karena telemetry membaca variabel FOC yang berubah cepat:

jangan mengambil data multi-variable secara sembarangan hingga menghasilkan snapshot tidak konsisten.

Gunakan:

- atomic snapshot,
- short critical section,
- double buffer,

atau mekanisme ringan lain.

Jangan memblokir FOC ISR lama hanya untuk telemetry.

---

# 51. CLEAN CODE

Setelah implementasi:

- hapus dead code,
- hapus duplicate code,
- hapus unused function,
- hapus unused variable,
- hapus debug sementara,
- hapus fake return success,
- rapikan naming,
- rapikan API,
- rapikan header,
- rapikan `static`,
- rapikan `const`,
- rapikan `volatile`,
- rapikan module responsibility.

Compiler warning harus diperiksa satu per satu.

Jangan hanya menambahkan:

`-Wno-error`

untuk menyembunyikan bug project.

Warning dari framework eksternal boleh diperlakukan berbeda apabila memang bukan berasal dari source project.

---

# 52. JANGAN ADA IMPLEMENTASI PALSU

Dilarang membuat:

- telemetry dummy,
- ADC dummy,
- Hall dummy,
- Encoder dummy,
- sensorless dummy,
- fake RPM,
- fake temperature,
- calibration yang hanya mengisi default,
- detection yang selalu return success,
- EEPROM write palsu,
- command ACK tetapi tidak menjalankan fungsi,
- function stub yang dianggap selesai.

Jika tidak tersedia:

**RETURN UNSUPPORTED**

atau:

**REQUIRES HARDWARE SUPPORT**

secara eksplisit.

---

# 53. AUDIT TERHADAP VESC

Buat matriks diff.

Kelompok:

## A. Sudah diterapkan benar

Equivalent secara fungsi.

## B. Sudah ada tetapi belum lengkap

Tunjukkan bagian yang hilang.

## C. Sudah ada tetapi salah

Tunjukkan error dan perbaiki.

## D. Belum diterapkan

Implementasikan jika relevan.

## E. Tidak relevan

Tidak perlu dipindahkan.

Contohnya:

- HFI
- CAN hardware
- IMU
- BMS
- bm_if
- NRF
- LEDPWM
- COMUSB
- QMLUI
- LispIF
- NTC temp
- LZO

---

# 54. ANALISIS FILE DAN FUNGSI

Untuk setiap temuan, sebutkan:

- nama file,
- nama fungsi,
- baris/section jika memungkinkan,
- fungsi saat ini,
- masalah,
- referensi VESC,
- perubahan yang diperlukan,
- risk level.

Contoh:

| File | Function | Status | Masalah | Referensi | Action |
|---|---|---|---|---|---|
| `bldc.c` | `foc_isr()` | Partial | ... | VESC `mcpwm_foc.c` | ... |

Jangan hanya memberikan komentar tingkat tinggi.

---

# 55. EFISIENSI STM32F103RCT6

Karena STM32F103RCT6 terbatas:

optimalkan:

- Flash
- SRAM
- CPU
- stack
- ISR latency
- DMA
- lookup table
- telemetry buffer
- task count.

Jangan port seluruh VESC mentah.

Prinsip:

**port behavior dan algorithm penting, bukan membawa dependency yang tidak dibutuhkan.**

Urutan prioritas:

1. Safety
2. Correct PWM
3. Correct ADC
4. Correct current measurement
5. Stable FOC
6. Sensor accuracy
7. Fault protection
8. Communication
9. Calibration
10. Application Layer
11. Telemetry
12. Additional feature

---

# 56. PROSEDUR PENGERJAAN WAJIB BERTAHAP

Jangan melakukan perubahan besar sekaligus tanpa audit.

Kerjakan dalam urutan:

### PART 1 — FULL SOURCE AUDIT

- baca semua file,
- mapping architecture,
- cari stub/dead code,
- buat dependency map,
- buat VESC feature diff awal.

### PART 2 — HARDWARE SAFETY

- GPIO
- TIM1/TIM8
- PWM polarity
- MOE
- dead-time
- ADC
- current sensing
- fault shutdown.

### PART 3 — FOC CORE

- current reconstruction
- Clarke/Park
- PI
- inverse Park
- SVPWM
- fixed-point
- ISR optimization.

### PART 4 — SENSOR

- Hall
- Encoder
- Sensorless
- angle source management.

### PART 5 — CONTROL LOOPS

- Duty
- Current
- Brake
- RPM
- Position.

### PART 6 — CALIBRATION

- ADC offset
- Hall detect
- Encoder calibration
- motor parameter detection
- sensor validation.

### PART 7 — AUTO-DETECTION

- VESC-like motor detection workflow.

### PART 8 — VESC PROTOCOL

- packet
- command
- SET/GET
- config
- detection commands.

### PART 9 — APP UART

- UART driver
- DMA
- packet parser
- command
- telemetry
- failsafe.

### PART 10 — APP ADC

- throttle
- brake
- calibration
- safety
- control mapping.

### PART 11 — COMMAND ARBITRATION

- ADC/UART/calibration/internal control coordination.

### PART 12 — EEPROM

- configuration
- version
- CRC
- default
- Flash wear.

### PART 13 — TELEMETRY

- actual values
- snapshot
- VESC Tool compatibility.

### PART 14 — FAULT + WATCHDOG

- fault manager
- timeout
- failsafe
- watchdog.

### PART 15 — LED + BUZZER

- startup melody
- status
- fault code.

### PART 16 — RTOS OPTIMIZATION

- tasks
- priorities
- stack
- queue
- synchronization.

### PART 17 — CLEANUP

- dead code
- duplicated implementation
- warning
- unused dependency.

### PART 18 — FINAL VERIFICATION

- build
- static test
- host test
- integration test
- hardware-test procedure.

---

# 57. PENGUJIAN APP ADC

Test minimal:

1. ADC raw minimum.
2. ADC raw maximum.
3. ADC center.
4. Deadband.
5. Inversion.
6. Filtering.
7. Ramp positive.
8. Ramp negative.
9. Low out-of-range.
10. High out-of-range.
11. Short-to-GND.
12. Short-to-VCC.
13. Startup throttle active.
14. Startup throttle neutral.
15. Throttle current mode.
16. Throttle duty mode.
17. Throttle RPM mode.
18. Analog brake.
19. Throttle + brake.
20. Configuration save.
21. Configuration reload.
22. Motor command arbitration.

Setiap test:

- PASS
- FAIL
- REQUIRES HARDWARE TEST

---

# 58. PENGUJIAN APP UART

Test minimal:

1. USART initialization.
2. Baud rate.
3. RX packet.
4. TX packet.
5. DMA RX.
6. DMA TX jika digunakan.
7. IDLE detection.
8. Partial packet.
9. Fragmented packet.
10. Back-to-back packets.
11. Invalid CRC.
12. Invalid length.
13. Random bytes.
14. Buffer overflow.
15. UART overrun.
16. Recovery.
17. GET_FW_VERSION.
18. GET_VALUES.
19. GET_VALUES_SELECTIVE.
20. SET_DUTY.
21. SET_CURRENT.
22. SET_CURRENT_BRAKE.
23. SET_RPM.
24. SET_POS.
25. ALIVE.
26. GET_MCCONF.
27. SET_MCCONF.
28. GET_APPCONF.
29. SET_APPCONF.
30. Hall detect.
31. Encoder calibration.
32. motor detection.
33. telemetry.
34. fault response.
35. communication timeout.
36. motor safe stop.

---

# 59. PENGUJIAN CONTROL

Test:

- Duty positive
- Duty negative
- Current positive
- Current negative
- Brake current
- Low RPM
- Medium RPM
- High RPM
- Position positive
- Position negative
- mode transition
- control timeout
- fault interruption.

Pastikan semua command menggunakan control API yang sama.

---

# 60. PENGUJIAN SENSOR

## Hall

- six valid states,
- invalid 000,
- invalid 111,
- forward sequence,
- reverse sequence,
- missing transition,
- wrong sequence,
- interpolation.

## Encoder

- forward,
- reverse,
- zero crossing,
- 360° crossing,
- overflow,
- direction,
- CPR,
- electrical conversion,
- calibration.

## Sensorless

- startup,
- observer locking,
- transition,
- low RPM behavior,
- high RPM behavior,
- observer loss.

---

# 61. PENGUJIAN FOC

Verifikasi software:

- Clarke result
- Park result
- inverse Park
- SVPWM
- PI saturation
- anti-windup
- Q-format
- overflow
- sign
- angle wrapping
- current scaling.

Hardware validation:

- phase current waveform,
- PWM phase relationship,
- dead-time,
- switching node,
- motor current,
- startup current,
- FOC stability.

Hardware measurement harus menggunakan alat yang sesuai.

---

# 62. TABEL AUDIT FINAL

Gunakan tabel minimal:

| Fitur | Kondisi | Referensi VESC | Masalah | Action |
|---|---|---|---|---|
| PWM | | | | |
| MOSFET polarity | | | | |
| ADC current | | | | |
| FOC ISR | | | | |
| Clarke/Park | | | | |
| Current PI | | | | |
| SVPWM | | | | |
| Hall | | | | |
| Encoder | | | | |
| Sensorless | | | | |
| Motor Detection | | | | |
| Hall Detect | | | | |
| Encoder Calibration | | | | |
| Duty Control | | | | |
| Current Control | | | | |
| Brake Control | | | | |
| Speed Control | | | | |
| Position Control | | | | |
| APP ADC | | | | |
| ADC Calibration | | | | |
| ADC Safe Start | | | | |
| ADC Throttle | | | | |
| ADC Brake | | | | |
| APP UART | | | | |
| UART DMA RX | | | | |
| UART TX | | | | |
| Packet Parser | | | | |
| VESC Commands | | | | |
| UART Failsafe | | | | |
| Command Arbitration | | | | |
| Telemetry | | | | |
| Motor Config | | | | |
| App Config | | | | |
| EEPROM | | | | |
| Fault | | | | |
| Watchdog | | | | |
| RTOS | | | | |
| LED | | | | |
| Buzzer | | | | |

---

# 63. VERIFIKASI BUILD

Sebelum dinyatakan selesai:

- clean build berhasil,
- tidak ada unresolved symbol,
- tidak ada duplicate symbol,
- tidak ada critical warning,
- tidak ada critical stub,
- tidak ada dummy telemetry.

Build configuration minimal:

- Left Hall
- Left Encoder
- Left Sensorless
- Right Hall
- Right Sensorless

Jika configuration runtime, minimal pastikan seluruh code path dapat dibuild dan digunakan tanpa conditional compilation yang merusak.

---

# 64. VERIFIKASI APP ADC

Firmware belum selesai jika:

- APP ADC hanya membaca ADC tetapi tidak mengendalikan motor,
- calibration belum berfungsi,
- safe-start belum ada,
- throttle fault belum ada,
- command arbitration belum ada.

Verifikasi:

- actual ADC,
- mapping,
- duty/current/RPM,
- brake,
- failsafe,
- persistence.

---

# 65. VERIFIKASI APP UART

Firmware belum selesai jika UART hanya bisa PING.

Verifikasi:

- RX
- TX
- DMA
- packet parser
- CRC
- SET command
- GET command
- configuration
- telemetry
- calibration command
- detection command
- timeout
- safe-stop.

---

# 66. VERIFIKASI EEPROM

Test:

1. save configuration,
2. reset MCU,
3. reload,
4. compare values,
5. corrupt CRC,
6. reset,
7. verify safe default,
8. verify CONFIGURATION_FAULT.

---

# 67. VERIFIKASI FAULT

Inject atau simulate:

- over-voltage
- under-voltage
- over-current
- Hall fault
- Encoder fault
- ADC fault
- APP ADC fault
- UART timeout
- configuration CRC fault.

Pastikan:

**fault → command zero → PWM safe → telemetry fault → LED/Buzzer fault indication**

---

# 68. FINAL DEFINITION OF DONE

Firmware baru boleh dianggap **software-ready** jika minimal:

- semua source dibaca,
- architecture dipahami,
- diff VESC selesai,
- MOSFET polarity benar,
- TIM1/TIM8 benar,
- dead-time benar,
- MOE benar,
- ADC current benar,
- current offset calibration benar,
- FOC pipeline lengkap,
- fixed-point aman,
- Hall lengkap,
- Encoder Left lengkap,
- Sensorless lengkap tanpa HFI,
- motor detection tersedia,
- Hall detect tersedia,
- Encoder calibration tersedia,
- duty control bekerja,
- current control bekerja,
- brake control bekerja,
- RPM control bekerja,
- position control bekerja,
- APP ADC lengkap,
- APP UART lengkap,
- command arbitration lengkap,
- VESC packet parser lengkap,
- VESC SET/GET command relevan lengkap,
- telemetry aktual,
- EEPROM CRC/version valid,
- watchdog aktif,
- communication timeout aktif,
- fault manager lengkap,
- LED fault indication aktif,
- buzzer fault indication aktif,
- power-on melody tersedia,
- RTOS optimal,
- dead code dibersihkan,
- build bersih,
- host/software test selesai.

---

# 69. BATASAN KLAIM VERIFIKASI

Jangan pernah menyatakan firmware:

**"100% hardware verified"**

hanya berdasarkan source audit atau compile.

Untuk setiap fitur gunakan status:

### VERIFIED BY SOURCE AUDIT

Logika source sudah diperiksa.

### VERIFIED BY BUILD

Berhasil compile/link.

### VERIFIED BY SOFTWARE TEST

Berhasil melalui host/unit/integration software test.

### REQUIRES HARDWARE TEST

Membutuhkan:

- STM32F103RCT6,
- controller hoverboard,
- motor,
- Hall,
- Encoder,
- oscilloscope,
- current probe,
- power supply,
- atau pengujian fisik lain.

---

# 70. FORMAT OUTPUT SETIAP PART

Setiap selesai satu PART, berikan:

## 1. Temuan

Apa yang salah atau kurang.

## 2. Analisis Root Cause

Mengapa hal tersebut terjadi.

## 3. Referensi VESC

File/fungsi/algoritma mana yang digunakan sebagai acuan.

## 4. Perubahan yang Dilakukan

File dan fungsi apa yang diubah.

## 5. Dampak

Apa dampaknya terhadap firmware.

## 6. Verification

- build,
- unit/software test,
- static check.

## 7. Hardware Test Required

Pengujian hardware apa yang masih diperlukan.

## 8. Remaining Issues

Apa yang belum selesai.

## 9. Next Part

Apa yang akan dikerjakan pada part berikutnya.

---

# 71. JANGAN MELEWATI MASALAH KECIL

Jika menemukan bug di luar scope part saat ini:

- catat,
- masukkan backlog,
- jangan hilangkan,
- prioritaskan jika safety-critical.

Safety-critical bug harus diperbaiki terlebih dahulu walaupun berada di part lain.

---

# 72. PRIORITAS SAFETY

Apabila implementasi VESC berbeda dengan hardware hoverboard:

**jangan memaksakan VESC.**

Prioritas:

**hardware datasheet / proven hoverboard hardware behavior**

>

**safe motor-control implementation**

>

**VESC architectural compatibility**

>

**feature completeness**

VESC digunakan sebagai referensi algoritma dan architecture, bukan alasan mengabaikan karakter hardware aktual.

---

# 73. HASIL AKHIR YANG DIINGINKAN

Target firmware akhir:

**STM32F103RCT6 Dual Motor Controller**

dengan:

### Motor Left

- Hall
- Encoder
- Sensorless

### Motor Right

- Hall
- Sensorless

### Control

- Duty
- Current
- Brake
- RPM
- Position

### Motor Control

- FOC
- SVPWM
- fixed-point optimized ISR
- current feedback
- observer
- calibration

### Detection

- ADC offset detection
- motor parameter detection
- Hall detection
- Encoder calibration
- sensor validation

### Application Layer

- APP ADC
- APP UART
- Command Arbitration

### Communication

- USART3
- VESC packet protocol
- VESC Tool commands
- telemetry

### Configuration

- Motor Configuration
- Application Configuration
- Flash/EEPROM
- CRC
- Versioning

### Safety

- fault manager
- command timeout
- watchdog
- safe PWM shutdown
- APP ADC safe-start

### User Indication

- power-on melody
- LED status
- LED fault code
- buzzer fault code

### Software Quality

- clean code
- no dead code
- no dummy feature
- no fake telemetry
- no hidden critical warning
- deterministic ISR
- efficient RTOS.

---

# 74. INSTRUKSI TERAKHIR YANG WAJIB DIIKUTI

**Baca seluruh ZIP terlebih dahulu. Jangan langsung menulis ulang firmware dari nol.**

Pertahankan bagian existing yang memang sudah benar dan sudah sesuai hardware.

Lakukan porting berdasarkan kebutuhan.

Jangan menyalin VESC secara membabi buta.

Jangan mengubah hardware mapping tanpa bukti.

Jangan mengubah MOSFET polarity yang sudah ditentukan:

**HIGH SIDE ACTIVE HIGH**

**LOW SIDE ACTIVE LOW**

Jangan menganggap compile = hardware working.

Jangan membuat stub untuk sekadar memenuhi checklist.

Jangan mengirim telemetry dummy.

Jangan membuat calibration dummy.

Jangan membuat auto-detection dummy.

Jangan mengabaikan APP ADC.

Jangan mengabaikan APP UART.

Jangan mengabaikan command source arbitration.

Jangan melakukan direct ADC/UART → PWM bypass.

Jangan mengklaim selesai sebelum seluruh dependency dan jalur program diaudit.

**Kerjakan secara bertahap, teliti, dan setiap tahap harus meninggalkan firmware dalam kondisi buildable serta lebih baik dibanding tahap sebelumnya.**

Target akhirnya adalah firmware yang:

**clean, deterministic, efisien, aman, mempunyai motor-control core yang matang, mempunyai APP ADC dan APP UART yang benar-benar siap digunakan, compatible dengan bagian VESC Tool yang relevan, serta semaksimal mungkin siap digunakan pada STM32F103RCT6 hoverboard controller.**