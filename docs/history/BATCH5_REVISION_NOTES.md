# Batch 5 — VESC API / Structure Alignment

Baseline: `vesc_stm32f103rct6_batch4_uart_dma_watchdog.zip`

## Scope

Batch 5 is deliberately structural. It does **not** change the FOC control law,
TIM1/TIM8/ADC timing, sensor observer, UART DMA backend, or IWDG behavior.

Implemented:

1. Canonical `setup_values` cumulative totals and separate `setup_stats` state.
2. Canonical `COMM_PACKET_ID` numbering through current ID 159 moved to
   `datatypes.h`; unsupported commands remain unimplemented.
3. Reduced standard application manager (`applications/app.c`, `app.h`) with
   only `APP_NONE` and `APP_UART` accepted by the STM32F103 backend.
4. Reduced typed `app_configuration` front-end while retaining the full
   493-byte VESC-6 raw image as the wire/persistence source of truth.
5. Canonical `confgenerator_*` typed wrappers over the pinned VESC-6
   481/493-byte wire ABI.
6. Canonical `conf_general_*` typed read/store wrappers over the existing
   transactional four-page flash backend.
7. Canonical `buffer_*` compatibility wrappers, including signed/unsigned
   64-bit and `float64_auto` helpers.
8. Canonical UART app entry points layered over the Batch-4 USART3 DMA
   transport; packet framing is preserved and large packet scratch storage is
   static rather than placed on a small FreeRTOS task stack.
9. `COMM_GET_VALUES_SETUP` now consumes canonical aggregate setup totals,
   while normal `COMM_GET_VALUES` remains per-selected-motor telemetry.

## Compatibility policy

- Firmware wire ABI remains VESC **6.00**.
- MCCONF size remains **481 bytes**.
- APPCONF size remains **493 bytes**.
- MCCONF signature remains **776184161**.
- APPCONF signature remains **486554156**.
- A typed serializer can represent deferred/unsupported fields faithfully,
  but applying a changed unsupported field is still rejected by the existing
  hardware ownership gate. This prevents fake settings from being ACKed.
- Controller ID is represented by typed APPCONF serialization, but this board's
  single USART3 management endpoint still owns the left controller ID at
  apply-time; local motor-2 selection remains via `COMM_FORWARD_CAN` routing.

## Files added

- `src/applications/app.c`
- `src/applications/app.h`
- `tools/test_batch5.py`
- `tools/test_batch5_roundtrip.c`
- `BATCH5_REVISION_NOTES.md`

## Files modified

- `src/datatypes.h`
- `src/comm/commands.c`
- `src/motor/mc_interface.c`
- `src/confgenerator.c`
- `src/confgenerator.h`
- `src/conf_general.c`
- `src/conf_general.h`
- `src/util/buffer.c`
- `src/util/buffer.h`
- `src/applications/app_uartcomm.c`
- `src/applications/app_uartcomm.h`

## Explicitly unchanged from Batch 4

SHA-256 comparison verifies byte identity for:

- `src/motor/mcpwm_foc.c`
- `src/motor/foc_math.c`
- `src/hwconf/hw.c`
- `src/timeout.c`

Thus Batch 5 does not alter current-loop/FW/MTPA/dead-time/braking/observer,
TIM/ADC/DMA motor timing, or watchdog behavior.

## Verification performed

All existing regression suites pass:

- `tools/verify_vesc_port.py`
- `tools/test_batch2.py`
- `tools/test_batch3.py`
- `tools/test_batch4.py`
- `tools/test_batch5.py`

Typed host round-trip test passes:

- MCCONF 481-byte serialize → deserialize.
- APPCONF 493-byte serialize → deserialize.
- represented deferred MCCONF fields round-trip correctly.
- represented APPCONF controller ID round-trips before apply-time validation.
- canonical signed/unsigned 64-bit buffer helpers round-trip.
- `float64_auto` round-trips within the expected wire quantization.

Strict host syntax checks (`clang -std=c11 -Wall -Wextra -Werror`) pass for
all B5-modified C translation units using focused STM32F1/CMSIS-RTOS2 stubs:

- `src/util/buffer.c`
- `src/confgenerator.c`
- `src/applications/app.c`
- `src/applications/app_uartcomm.c`
- `src/conf_general.c`
- `src/motor/mc_interface.c`
- `src/comm/commands.c`

This remains source/host verification. A real `arm-none-eabi-gcc`/PlatformIO
build and physical STM32F103 hoverboard bench test are still required before
claiming hardware validation.

## Deferred to Batch 6

Batch 6 should implement VESC Tool ecosystem behavior, not change the hard FOC:

- terminal command / synchronous terminal command,
- VESC plot commands,
- experiment sample command integration,
- temporary MCCONF commands,
- `COMM_APP_DISABLE_OUTPUT`,
- real setup statistics + reset statistics,
- odometer command/persistence,
- `COMM_MOTOR_ESTOP`.
