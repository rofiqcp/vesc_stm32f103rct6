# Batch 4 — USART3 DMA + IWDG/Heartbeat Safety

Baseline: `vesc_stm32f103rct6_batch3_sensor_observer.zip`

Batch 4 intentionally changes only communication transport and task-level
safety/recovery. Motor-control algorithms, VESC6 configuration layout,
TIM1/TIM8 PWM timing, dual-ADC current sampling, Hall/encoder policy and B3
observer/PLL behavior are unchanged.

## 1. USART3 RX DMA circular + IDLE

- USART3 remains PB10 TX / PB11 RX at 115200 8N1.
- RX is now DMA1 Channel3 circular instead of RXNE interrupt per byte.
- A 256-byte DMA circular buffer is drained into the existing 1024-byte
  software ring.
- Drain points are USART IDLE, DMA half-transfer, DMA transfer-complete, and
  task-side polling. Continuous traffic therefore does not depend on an IDLE
  gap.
- USART3 IRQ now handles only IDLE/error status; packet framing and command
  parsing remain in the `uartcomm proc` task.
- RX DMA transfer error restarts the circular DMA receiver. A partially damaged
  VESC frame is left to CRC/parser resynchronization rather than wedging UART.

## 2. USART3 TX DMA queue

- TX is now DMA1 Channel2 memory-to-peripheral.
- The existing complete-frame queue is retained; one full VESC frame is sent
  per DMA transfer.
- The queue advances only on DMA transfer-complete/error, eliminating TXE
  interrupt-per-byte load.
- Defensive corrupt-slot skipping prevents one invalid queue entry from
  wedging all later VESC replies.

Interrupt priority remains below the ADC/FOC path:

- DMA1 Channel1 ADC/FOC: priority 0 (unchanged)
- USART3 RX/TX DMA + IDLE: priority 2

No RTOS API is called from USART/DMA IRQ context.

## 3. Independent watchdog + liveness heartbeats

Three independent liveness sources are monitored:

- FOC ADC/DMA ISR heartbeat
- 1 kHz motor-service heartbeat
- communication parser-task heartbeat

The watchdog starts only after the control threads were created. FOC heartbeat
is not required until `motor_hw_start_sampling()` has actually started ADC/DMA,
which avoids false resets during communication-first boot.

The health gate evaluates progress over 100 ms windows. While healthy the 1 kHz
motor-service feeds IWDG every 10 ms. If a required heartbeat stops changing,
feeding stops and hardware reset follows.

STM32F1 IWDG uses prescaler /64 and reload 300. Because LSI frequency has wide
tolerance, the hardware reset time is intentionally approximate (~0.4–1.2 s),
providing margin for F103 flash page operations while still recovering a wedged
controller.

## 4. Reset-reason diagnostics

`RCC->CSR` reset flags are captured at boot before reset flags are cleared.
Runtime diagnostics expose:

- raw reset flags
- whether the previous boot followed an IWDG reset
- watchdog started/healthy state
- required heartbeat mask
- FOC / motor-service / communication heartbeat counters
- UART DMA IRQ/error counters and USART IDLE count

These fields are appended only to the custom communication diagnostic payload;
the VESC 6.00 standard wire ABI is unchanged.

## Changed source files

- `src/applications/app_uartcomm.c`
- `src/applications/app_uartcomm.h`
- `src/comm/commands.c`
- `src/main.c`
- `src/motor/mcpwm_foc.c` (heartbeat call only)
- `src/motor_tasks.c`
- `src/stm32f1xx_it.c`
- `src/timeout.c`
- `src/timeout.h`
- `tools/test_batch4.py` (new)

## Verification

Run:

```text
python tools/verify_vesc_port.py
python tools/test_batch2.py
python tools/test_batch3.py
python tools/test_batch4.py
```

Batch 4 additionally syntax-checks the DMA transport and watchdog source with
strict Clang warnings against a focused STM32F1 register/RTOS stub. This is a
source-level check, not a replacement for the real PlatformIO ARM build.

## Hardware commissioning additions for B4

Before running motors at power:

1. Confirm VESC Tool handshake over USART3 after the DMA conversion.
2. Stream `GET_VALUES` and `COMM_SAMPLE_PRINT` while checking UART DMA error and
   overrun counters remain zero.
3. Confirm USART3 IDLE + DMA HT/TC paths all receive data correctly.
4. Verify reset-reason reporting with one controlled software reset.
5. On a bench supply, deliberately stop feeding one heartbeat in a test build
   and confirm IWDG resets the MCU and the next boot reports IWDG reset.
6. Only then re-enable normal motor commissioning.
