# STM32F103RCT6 48-KiB RAM Linker Fix

Baseline: `vesc_stm32f103rct6_batch11_part2_arm_buildfix.zip`
Target remains: STM32F103RCT6, 48 KiB SRAM, 256 KiB Flash (248 KiB application + 8 KiB transactional config pages).

## Real target-build failure addressed

The first real PlatformIO ARM link reported:

`region RAM overflowed by 6184 bytes`

This revision fixes the RAM architecture rather than enlarging the linker RAM region.

## SRAM reductions

1. Debug/experiment sample capture is reduced from 256 to 64 entries.
   - `debug_sample_t` is 30 bytes.
   - old: 256 * 30 = 7680 bytes
   - new: 64 * 30 = 1920 bytes
   - SRAM saved: 5760 bytes

2. FreeRTOS heap_4 reservation is reduced from 22 KiB to 18 KiB.
   - SRAM saved: 4096 bytes
   - Application thread stacks request about 9.0 KiB in total; idle/timer stacks add about 1.5 KiB. 18 KiB leaves additional heap room for TCBs, mutexes, the blocking queue and allocator metadata.

Combined static SRAM saving: 9856 bytes.

Using the real linker overflow of 6184 bytes, the projected post-fix linker headroom is about 3672 bytes. The real `pio run` output remains authoritative.

## Runtime heap guard

After motor-service/sample/fault threads are created, motor-ready is refused unless `xPortGetFreeHeapSize()` reports at least 2048 bytes remaining. malloc-failure and stack-overflow hooks still hard-disable both bridges.

## Functionality intentionally preserved

No reduction was made to:
- dual motor state/control;
- 16-kHz fixed-point FOC;
- ADC1/ADC2 current acquisition;
- ADC3/DMA2 Vbus;
- LEFT Hall/ABI and RIGHT Hall-only policy;
- current-offset calibration and dual Detect-All;
- R/Ld/Lq/flux/sensor detection;
- VESC Tool FW/config/telemetry/control commands;
- transactional flash persistence;
- fault LED/buzzer and startup melody.

Only the optional high-rate debug/experiment capture history is capped to 64 samples. Standard VESC telemetry is unaffected.

## Compiler-warning cleanup

The source warnings observed after the vendor-HAL warning was unblocked are corrected at source:
- `debug_sample.c`: redundant enum lower-bound check removed (`-Wtype-limits`).
- `hw.c`: chained duty clamps split into unambiguous statements (`-Wmisleading-indentation`).
- `conf_general.c`: misleading one-line guards/flash-program flow reformatted without changing behavior.

Vendor STM32CubeF1 is not patched. Project warning policy remains source-scoped through `build_src_flags`.

## Required real target verification

From a freshly extracted directory:

```
pio run -t clean
pio run
```

Expected: link succeeds with RAM below 49152 bytes. Then:

```
pio run -t upload
```

Do not copy only `platformio.ini` into an old tree; use the whole ZIP because the RAM reduction also changes FreeRTOSConfig and the debug sample buffer.
