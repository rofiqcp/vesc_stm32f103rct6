# Stage3 v32 — Full Re-audit and ARM Build-Error Hardening

## 1. Temuan

Re-audit dilakukan terhadap Stage3 v31 dengan master prompt sebagai acceptance criteria.
Build log hardware pengguna menunjukkan parser cascade pada `src/comm/commands.c`, dimulai saat fungsi-fungsi file tersebut tidak lagi dikenali sebagai function definitions dan diikuti error pada FreeRTOS/list.h/libc headers.

Root cause pada Stage3 v31: `commands.c` meng-include `task.h` secara langsung tanpa `FreeRTOS.h` lebih dulu. Public FreeRTOS kernel headers mensyaratkan `FreeRTOS.h` telah di-include sebelum `task.h`/queue/timer headers. Stage3 v32 menghapus dependency langsung tersebut dan menggunakan CMSIS-RTOS2 API `osThreadGetStackSpace()`.

Audit tambahan juga menemukan:
- APP ADC seqlock reader memiliki kemungkinan membaca variabel sequence kedua sebelum terinisialisasi ketika iterasi pertama melihat writer-active/odd sequence.
- Inductance-detection public motor helper sebaiknya mempunyai null-motor guard langsung, bukan bergantung pada guard internal `detect_begin()`.
- ADC1/ADC2 dual-regular-simultaneous rank 5 mempunyai sampling-time berbeda. Rank pasangan kini sama-sama 239.5 cycles.
- Low-priority sample/rotor telemetry dapat bersaing dengan command response pada single USART3 TX queue. v32 menyisakan response headroom tanpa menambah queue besar.
- Diagnostic payload lebih aman memakai shared bounded packet scratch daripada array stack berukuran manual.

## 2. Root Cause Build Error

Stage3 v31:

```c
#include "motor_tasks.h"
#include "task.h"
#include "conf_general.h"
```

Stage3 v32:

```c
#include "motor_tasks.h"
#include "conf_general.h"
#include "confgenerator.h"
#include "cmsis_os2.h"
```

Stack-space statistics sekarang:

```c
osThreadGetStackSpace(s_packet_tp)
osThreadGetStackSpace(s_blocking_tp)
```

Tidak ada lagi direct FreeRTOS `TaskHandle_t`/`uxTaskGetStackHighWaterMark()` dependency di `commands.c`.

## 3. Perubahan Source Utama

### `src/comm/commands.c` / `commands.h`
- Hapus direct `task.h` include yang menyebabkan ARM parser cascade.
- Stack watermark menggunakan CMSIS-RTOS2.
- Tambah low-priority VESC payload path untuk rotor/sample telemetry.
- `COMM_DIAG` revision 14 menggunakan shared bounded payload scratch.
- Tambah `tx_low_priority_drops` ke diagnostic.
- Tambah GCC/Clang printf-format attribute pada `commands_printf()` agar call-site format checking aktif.
- Firmware string: `vesc-f103-hoverboard-v32-full-reaudit`.

### `src/applications/app_uartcomm.c/.h`
- TX tetap DMA dan non-blocking.
- Low-priority telemetry tidak boleh menggunakan slot queue terakhir yang dibutuhkan response normal.
- Tambah low-priority drop counter.

### `src/applications/app_adc.c`
- Perbaiki seqlock snapshot loop sehingga tidak ada uninitialized read saat writer sequence sedang odd.

### `src/motor/mcpwm_foc.c`
- Tambah direct null-motor guard pada current-based inductance detection helper.

### `src/hwconf/hw.c/.h`
- Rank5 ADC1/ADC2 dual simultaneous disamakan ke 239.5 cycles.
- Runtime sampling-contract diperluas: TIM1/TIM8 mode/TRGO/slave/RCR, ADC dual mode, conversion count, hard-current/app channel ranks, DMA1/DMA2 mode+transfer, ADC3 contract.

### `src/util/buffer.c`
- Float-to-double conversion pada VESC float64-auto decode dibuat eksplisit agar warning contract bersih tanpa mengubah wire behavior.

## 4. Test Infrastructure yang Ditambah

### `tools/test_stage3_v32_compile_contract.py`
- 20 translation unit logic-heavy dikompilasi `-fsyntax-only`.
- Jika tersedia, dijalankan dengan GCC dan Clang.
- Flags: `-Wall -Wextra -Wshadow -Wdouble-promotion -Wformat=2 -Werror`.
- Stub `task.h` sengaja meniru requirement FreeRTOS asli sehingga include-order regression tidak dapat tersembunyi.
- Scan seluruh source memastikan kernel header FreeRTOS tidak di-include sebelum `FreeRTOS.h`.

### `tools/test_stage3_v32_static_analysis.py`
- Clang Static Analyzer pada 10 critical translation units.
- GCC `-fanalyzer` pada 10 critical translation units.
- Release hygiene scan terhadap TODO/FIXME/HACK dan unsafe unbounded libc calls.

## 5. Verification di Environment Audit

- 25/25 functional/regression Python scripts: PASS.
- GCC strict TU syntax/warning checks: 20/20 PASS.
- Clang strict TU syntax/warning checks: 20/20 PASS.
- Clang Static Analyzer critical units: 10/10 PASS.
- GCC `-fanalyzer` critical units: 10/10 PASS.
- `tools/debug.py --self-test`: PASS.
- Python tools compileall: PASS.
- Release hygiene scan: PASS.
- Excluded subsystem include scan: PASS.

## 6. Batas Verifikasi

Environment audit ini tidak mempunyai PlatformIO/`arm-none-eabi-gcc`, sehingga Stage3 v32 **belum boleh diberi status VERIFIED BY BUILD** sampai dijalankan dengan toolchain target.

Yang sudah berstatus:
- VERIFIED BY SOURCE AUDIT untuk perubahan di atas.
- VERIFIED BY SOFTWARE TEST untuk regression/host-runtime/compile-contract/static-analysis yang tercantum.

Masih wajib:
- Target ARM clean compile/link.
- RAM/Flash report.
- Hardware gate/ADC/current/ISR timing tests.

## 7. Build Gate di Target

Jalankan dari root project:

```bash
pio run -t clean
pio run
```

Build baru boleh dinyatakan sukses jika:
- tidak ada source project error,
- tidak ada unresolved/duplicate symbol,
- tidak ada critical project warning,
- linker selesai,
- RAM <= 49152 bytes,
- Flash <= 253952 bytes.

Vendor STM32Cube warning yang memang berasal dari framework eksternal harus dibedakan dari warning source project; jangan men-disable warning project hanya untuk menyembunyikan bug.

## 8. Hardware Mapping yang Tidak Diubah

- LEFT inverter: TIM8.
- RIGHT inverter: TIM1.
- High-side gate input: active HIGH.
- Low-side complementary gate input: active LOW.
- APP UART: USART3 PB10 TX / PB11 RX.
- APP ADC: PA2 throttle / PA3 brake.
- FOC ISR: 16 kHz fixed-point architecture.
- LEFT: Hall / Encoder A-B / Sensorless no-HFI.
- RIGHT: Hall / Sensorless no-HFI.
- Excluded modules tetap tidak diaktifkan.

## 9. Remaining Hardware Verification

Walaupun build nanti sukses, masih perlu bench validation: PWM polarity/deadtime/MOE, ADC current polarity/offset, rank timing, dual motor current waveform, Hall/encoder direction, sensorless handover, Detect-All, APP ADC safe-start/fault/brake, UART timeout, flash corruption recovery, watchdog, ISR WCET, stack/heap margin, dan brownout/power-cycle persistence.
