# OpenOCD/GDB debug - VESC STM32F103RCT6

This folder is specific to this firmware. The management link is USART3 at 115200 8N1:

- PB10 = STM32 TX -> USB-UART RX
- PB11 = STM32 RX <- USB-UART TX
- GND must be common
- use 3.3-V TTL levels
- RX = DMA1 Channel3 circular, 512 bytes
- TX = DMA1 Channel2 normal; final USART3 TC is polled in task context (no USART3 IRQ dependency)
- FOC = DMA1 Channel1 ISR; no FreeRTOS API is called from the 16-kHz current loop

## 1. Build release

```bash
pio run -e stm32f103rc
```

The old GCC 7.2 FreeRTOS `queue.c` `-Wnonnull` false-positive is suppressed only for that vendor translation unit. Project source warnings are not hidden.

## 2. Raw UART first

```bash
./openocd_debug/uart_probe.sh /dev/ttyUSB0 115200
```

Or manually:

```bash
python3 tools/debug.py handshake --port /dev/ttyUSB0 --baud 115200 --attempts 5 --timeout 0.7
python3 tools/debug.py info --port /dev/ttyUSB0 --baud 115200
python3 tools/debug.py comm-diag --port /dev/ttyUSB0 --baud 115200
```

A level-1 handshake failure (`RX raw: <no bytes>`) means debug transport/boot/UART first, not FOC tuning.

## 3. One-command OpenOCD + GDB

```bash
./openocd_debug/run_debug.sh
```

The script builds `stm32f103rc_debug`, verifies DWARF, starts ST-LINK DAP-direct OpenOCD (automatic HLA fallback), loads the ELF, then runs until either `packet_process_thread` starts successfully or an early-fatal breakpoint is reached. It does **not** stop at `main()`, because UART/DMA/task state is expected to be zero there. The debug build also keeps the OpenOCD FreeRTOS compatibility symbol `uxTopUsedPriority`, so `info threads` can enumerate the RTOS tasks on the older CubeF1 FreeRTOS kernel.

If an OpenOCD server is already running:

```bash
./openocd_debug/run_debug.sh --reuse-openocd --no-flash
```

Only reuse it if it was started with equivalent STM32F1/FreeRTOS configuration; otherwise the custom `monitor vesc_*` commands will not exist.

## 4. GDB commands

At the GDB prompt:

```gdb
vesc_boot
vesc_uart
vesc_motors
vesc_faults
vesc_tasks
vesc_snapshot
monitor vesc_uart_hw
monitor vesc_clock_hw
monitor vesc_nvic_hw
monitor vesc_fault_hw
monitor vesc_motor_hw
```

When RX DMA is running continuously, `huart3_vesc.RxState = HAL_UART_STATE_BUSY_RX` is **normal**. TX idle is determined from `huart3_vesc.gState = HAL_UART_STATE_READY`; do not expect the combined `HAL_UART_GetState()` value to be `READY` while circular RX is active.

`vesc_tasks` is useful after `vTaskStartScheduler()`. The launcher normally reaches the first `packet_process_thread` entry at `stage=100`. Inspect `vesc_boot` and `vesc_uart`, then use `continue` to run normally. `quit` is hooked to `monitor reset run`, and the shell cleanup has a second reset/run fallback, so leaving GDB no longer leaves the MCU halted with a silent UART.

The expected steady-state boot breadcrumb is `stage=100 error=0`.

Boot stage meanings:

- 1: HAL init
- 10: clock/status init
- 20: five task objects created
- 30: commands + USART3 management transport initialized
- 40: motor hardware init
- 50: motor runtime init
- 60: default/persistent VESC config
- 70: telemetry/app/timeout init
- 80: motor worker resource validation
- 90: ADC/DMA sampling contract
- 100: scheduler start

Boot errors currently include 11=status init, 21=task creation, 31=commands/UART, 61=config apply, 71=telemetry, 72=timeout, 81=task resource validation, 91=ADC/DMA sampling contract.

## 5. UART timeout decision tree

### A. `g_vesc_boot_stage < 30`

The firmware did not reach the management link. Fix the reported boot error first.

### B. `stage >= 30`, RX DMA CNDTR never changes

Send repeated handshakes while watching `vesc_uart`. If DMA1 Channel3 `CNDTR` stays at 512, bytes are not reaching PB11. Check USB-UART port identity, TX/RX crossing, common GND, 3.3-V TTL, PB11 continuity, USART3 clock/GPIO, and baud.

### C. RX counter/CNDTR changes but `rx_frames_ok` remains 0

The MCU sees bytes but does not accept a complete VESC frame. Check BRR/clock, framing/CRC, and packet-process task scheduling.

### D. `rx_frames_ok` increments but host receives zero bytes

Break on TX completion:

```gdb
vesc_break_uart_tx
continue
```

Also inspect DMA1 Channel2, PB10 and USART3 TC. `tx_frames` should increment when the reply is encoded; `tx_complete_count` should increment when the frame physically completes.

### E. scheduler is running but `packet_process` is absent

`vesc_tasks` should show the five application threads: `fault_stop`, `timer`, `packet_process`, `blocking`, `sample_send`, plus FreeRTOS kernel tasks. A missing application thread indicates task creation/stack/heap corruption.

## 6. FreeRTOS task priorities

- `fault_stop`: 6
- `timer`: 5
- `packet_process`: 5
- `blocking`: 5
- `sample_send`: 4

The hard current loop is not a FreeRTOS thread; it runs in `DMA1_Channel1_IRQHandler()`.

## 7. SWD / normal reflash protection

STM32F1 `AFIO->MAPR.SWJ_CFG` must not be modified with generic read-modify-write remap helpers after startup. This firmware therefore applies the required MAPR state in one controlled write:

- JTAG disabled, **SWD PA13/PA14 remains enabled**
- USART3 remains non-remapped on PB10/PB11
- ADC1 regular trigger remap to TIM8_TRGO remains enabled

The commissioning build also sets `VESC_IWDG_ENABLE=0`; motor command timeout/fault protection remains active, but the irreversible hardware IWDG is not allowed to create a fast reset loop that makes ST-Link appear to require connect-under-reset. Re-enable the hardware IWDG only after communication, FOC sampling and task heartbeats are validated on the actual board.

If an older firmware has already disabled SWD or entered a reset loop, use connect-under-reset **once** to replace it with this build. Subsequent normal ST-Link sessions should not require it.

## Safety

Halting an MCU during active FOC destroys deterministic current-loop timing. For ISR breakpoints or full register snapshots, keep the motor/power stage in a safe commissioning state. Prefer UART `comm-diag`, `status`, and `monitor` for live non-halting observation once communication works.

## Emergency: replacing an older image that already locks SWD

This should **not** be needed after the fixed build is installed. If the currently
flashed older image can only be reached while reset is asserted and ST-LINK NRST
is wired, flash the fixed image once with:

```bash
./openocd_debug/recover_under_reset.sh
```

After that, return to normal `pio run -t upload` / `run_debug.sh` operation.
