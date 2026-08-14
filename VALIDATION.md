# VALIDATION — V3

## Yang telah diuji di environment pembuatan source

### Python

```text
python3 -m py_compile debug_vesc_f103.py
python3 debug_vesc_f103.py --self-test
```

Hasil:

```text
SELF-TEST PASS: CRC, framing, streaming parser, sample parser
```

### Host C tests

```text
tests/host/run.sh
```

Menguji:

- CRC16 CCITT/XMODEM VESC;
- packet encode/decode streaming;
- sin/cos Q15 LUT cardinal points;
- zero-vector SVM = 50/50/50;
- duty clamp;
- Python self-test.

Hasil saat ZIP dibuat:

```text
test_packet: PASS
test_foc_math: PASS
SELF-TEST PASS: CRC, framing, streaming parser, sample parser
host tests: ALL PASS
```

### Strict syntax-check modul logika

Dengan HAL/CMSIS stub hanya untuk type/API declaration, modul berikut telah diperiksa dengan:

```text
-Wall -Wextra -Wshadow -Wconversion -Werror
```

dan lolos:

```text
foc_math.c
vesc_packet.c
motor_control.c
sensor_detect.c
telemetry.c
motor_tasks.c
vesc_comm.c
fault.c
```

Audit ini menemukan dan memperbaiki bug `reply_setup_values()` (`t->field` pada struct non-pointer) sebelum ZIP final.

### Static architecture checks

- Semua `.h` firmware berada di `src/`.
- Tidak ada `#if LEFT_SENSOR_MODE` / compile-time sensor selection.
- `OCPolarity = HIGH` dan `OCNPolarity = LOW`.
- High-side idle LOW, low-side idle HIGH.
- Requested threads ada: `timer_thread`, `pid_thread`, `sample_send_thread`, `fault_stop_thread`, `stat_thread`, `vesc_comm_thread`.
- Fast FOC path tidak memanggil RTOS/UART/printf/dynamic allocation/trigonometri float.
- Current PI menggunakan raw Id/Iq; filtered Id/Iq dipisahkan untuk telemetry/slow logic.

## Yang belum dapat divalidasi di environment ini

Environment ini tidak memiliki PlatformIO/ARM GCC STM32, sehingga V3 ini belum di-cross-compile di sini dan belum diuji pada power stage fisik.

Baseline V2 yang menjadi dasar hardware/CMSIS-RTOS2 source sudah berhasil di mesin pengguna sampai `firmware.bin`. V3 menambahkan runtime sensor mux/detect, debug, telemetry dan polarity correction di atas baseline tersebut.

## Hal yang tetap wajib diverifikasi di PCB

1. Scope gate input: high-side ON=HIGH, low-side ON=LOW, deadtime kedua MOSFET OFF.
2. `LEFT/RIGHT_CURRENT_A_PER_COUNT` sesuai shunt/op-amp.
3. `LEFT/RIGHT_DC_CURRENT_A_PER_COUNT` sesuai DC-current amplifier.
4. `DCLINK_V_PER_COUNT` sesuai divider.
5. Phase U/V/W dan tanda current U/V benar.
6. Startup current-zero calibration valid dan residual Id/Iq dekat nol.
7. Auto-detect Hall/encoder dengan motor tanpa beban/beban ringan.
8. Test current kecil sebelum RPM/position.
9. ISR cycle margin (`isr_max_cycles`, `isr_overruns`).

Tidak ada software yang bisa menjamin power-stage aman tanpa verifikasi electrical tersebut.
