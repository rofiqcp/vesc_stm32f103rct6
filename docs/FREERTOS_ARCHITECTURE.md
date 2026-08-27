# Native FreeRTOS Architecture

This revision intentionally uses **native FreeRTOS only**. The CMSIS-RTOS2 wrapper is not built and application code does not use `osThread*`, `osMutex*`, `osMessageQueue*`, `osDelay`, or `osKernel*` APIs.

## Application task layout

| VESC task | FreeRTOS priority | Role |
|---|---:|---|
| `fault_stop` | 6 | Highest-priority task-side fault stop and fault watchdog heartbeat |
| `timer` | 5 | 1 kHz motor service, RPM/PID/application arbitration, calibration service, timeout/watchdog; also services LED/buzzer state at 10 ms |
| `packet_process` | 5 | USART3 DMA RX drain, VESC framing/CRC, non-blocking command dispatch |
| `blocking` | 5 | Blocking VESC operations such as configuration flash writes and motor/sensor detection |
| `sample_send` | 4 | Low-priority debug sample/telemetry transmission |

FreeRTOS also creates its normal **Idle task (priority 0)** and, because software timers are enabled, **Timer Service task (priority 2)**. These are kernel tasks and are not counted among the five application tasks.

## Hard real-time FOC

FOC is deliberately **not** moved into a FreeRTOS task. `DMA1_Channel1_IRQHandler()` dispatches `foc_adc_dma_isr(g_adc_dual_dma)` directly after the coherent current-sampling half-transfer. This keeps current acquisition, Clarke/Park, current PI, inverse transform/SVPWM timing independent from task scheduling.

The DMA current ISR and other hard power-stage protection ISRs do not call queue, semaphore, notification, delay, or task-creation APIs. ISR-detected faults are latched into pending state and consumed by task context.

## Synchronization

Native FreeRTOS primitives replace the former wrapper objects:

- `SemaphoreHandle_t` + `xSemaphoreCreateMutex` / `xSemaphoreTake` / `xSemaphoreGive` for telemetry and TX serialization.
- `QueueHandle_t` + `xQueueCreate` / `xQueueSend` / `xQueueReceive` for blocking VESC jobs.
- Direct task notifications (`xTaskNotify`, `xTaskNotifyWait`) for `fault_stop` and `sample_send` wakeups.
- `vTaskDelayUntil` for the deterministic 1 kHz `timer` service.

## Regression check

Run:

```bash
python3 tools/rtos_audit.py
```

The audit fails if CMSIS-RTOS2 APIs return, if the application task count changes from five, if the task names change, or if RTOS synchronization calls are introduced into the hard ISR glue.

For a firmware build on the development machine:

```bash
pio run -e stm32f103rc
```
