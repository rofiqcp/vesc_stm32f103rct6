# VESC STM32F103RCT6 - FreeRTOS / USART3 communication revision

This package is the communication/debug revision of the 5-application-task
native-FreeRTOS build.

## Architecture kept intentionally

Application tasks remain exactly five:

1. `fault_stop` priority 6
2. `timer` priority 5
3. `packet_process` priority 5
4. `blocking` priority 5
5. `sample_send` priority 4

The FOC current loop is **not** a FreeRTOS task. It remains in
`DMA1_Channel1_IRQHandler()` -> `foc_adc_dma_isr()`.

No CMSIS-RTOS2 application API is used.

## USART3 transport

The management/VESC Tool serial transport follows the proven SmartESC pattern:

- 115200 baud, 8N1
- PB10 = USART3 TX
- PB11 = USART3 RX
- RX = DMA1 Channel 3, circular, 512-byte buffer
- RX task polls the DMA write position from `CNDTR`
- TX = DMA1 Channel 2, normal DMA
- final USART3 `TC` is polled in task context; USART3 NVIC is not required
- DMA IRQs delegate to `HAL_DMA_IRQHandler()`
- packet parsing/CRC/command dispatch stays in `packet_process` task context

The VESC packet framing is compatible with upstream VESC `packet.c`:
start 2/3, payload length, payload, CRC16-CCITT, end byte 3.

## Critical TX bug fixed in this revision

Do not wait for:

```c
HAL_UART_GetState(&huart3_vesc) == HAL_UART_STATE_READY
```

while circular RX DMA is active. STM32 HAL combines `gState` (TX/global) and
`RxState`; the permanent RX DMA intentionally keeps `RxState` at
`HAL_UART_STATE_BUSY_RX`, so the combined state can never equal `READY`.

TX readiness now checks:

```c
huart3_vesc.gState == HAL_UART_STATE_READY
HAL_DMA_GetState(&hdma_usart3_tx) == HAL_DMA_STATE_READY
```

This lets RX remain continuously active while `COMM_FW_VERSION` and all other
VESC replies are transmitted normally.

## Boot order

The VESC management UART is brought up before motor ADC/PWM/FOC initialization.
Therefore a later motor-subsystem startup failure can still answer a firmware
handshake and expose boot diagnostics.

Expected normal boot breadcrumb:

```text
g_vesc_boot_stage = 100
g_vesc_boot_error = 0
```

## First tests after flashing

```bash
pio run -e stm32f103rc
pio run -e stm32f103rc -t upload
python3 tools/debug.py handshake --port /dev/ttyUSB0 --baud 115200 --attempts 5 --timeout 0.7
python3 tools/debug.py info --port /dev/ttyUSB0 --baud 115200
python3 tools/debug.py comm-diag --port /dev/ttyUSB0 --baud 115200
```

Expected `handshake`: a valid framed `COMM_FW_VERSION` response.

## OpenOCD/GDB

```bash
chmod +x openocd_debug/run_debug.sh openocd_debug/uart_probe.sh
./openocd_debug/run_debug.sh
```

After normal boot, interrupt GDB with Ctrl-C and use:

```gdb
vesc_boot
vesc_uart
vesc_tasks
vesc_motors
vesc_faults
vesc_snapshot
monitor vesc_uart_hw
```

Expected UART steady state with circular RX active:

- `huart3_vesc.gState` = READY when TX is idle
- `huart3_vesc.RxState` = BUSY_RX (this is normal)
- DMA1 Channel3 CNDTR changes when host bytes reach PB11
- `g_vesc_uart_stats.rx_bytes` increases when the packet task drains RX
- `tx_complete_count` increases after replies physically finish on PB10

## Build warning

The GCC 7.2 warning inside STM32CubeF1's vendor `FreeRTOS/Source/queue.c`
(`memcpy` nonnull warning on the zero-sized mutex queue path) is suppressed only
for that vendor translation unit through PlatformIO build middleware. Project
source warnings remain enabled.

Reference implementations used for the communication architecture:

- https://github.com/vedderb/bldc
- https://github.com/Koxx3/SmartESC_STM32_v2

## STM32F1 SWD / normal reflash fix

A separate STM32F1-specific issue could make a successfully flashed firmware
appear to require **connect under reset** on the next ST-Link session. Generic
`AFIO->MAPR` remap helpers perform read-modify-write operations. On STM32F1,
the `SWJ_CFG` field must not be trusted as ordinary readback data; a later ADC
remap can therefore corrupt the earlier JTAG-off/SWD-on setting.

This revision removes all generic MAPR remap calls from the application and
applies one controlled MAPR image in `hw.c`:

- `SWJ_CFG = 0b010`: JTAG off, SWD PA13/PA14 on
- USART3 remap cleared: PB10/PB11
- ADC1 ETRGREG remap set: TIM8_TRGO

The commissioning build also keeps the hardware IWDG disabled by default.
This avoids a second way to create a fast reset loop that is difficult to attach
to normally.

## Debugger exit behavior

The previous debug flow could stop at `main()`, then GDB `quit` would detach
from a still-halted MCU. Running `tools/debug.py` immediately afterwards then
produced a UART timeout even though the UART code had not failed. The revised
launcher runs to either `packet_process_thread` or an early-fatal breakpoint,
and both GDB `hook-quit` and shell cleanup issue `reset run` before OpenOCD is
stopped.
