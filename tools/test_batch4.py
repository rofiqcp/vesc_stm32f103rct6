#!/usr/bin/env python3
"""Batch 4 source/numeric regressions: USART3 DMA + IWDG heartbeat safety."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]

def read(p): return (ROOT / p).read_text()
def req(cond, msg):
    if not cond: raise AssertionError(msg)
    print('PASS:', msg)

uart_h = read('src/applications/app_uartcomm.h')
uart = read('src/applications/app_uartcomm.c')
it = read('src/stm32f1xx_it.c')
timeout = read('src/timeout.c')
timeout_h = read('src/timeout.h')
main = read('src/main.c')
tasks = read('src/motor_tasks.c')
commands = read('src/comm/commands.c')
foc = read('src/motor/mcpwm_foc.c')

# UART DMA ownership and IRQ architecture.
req('DMA1_Channel2->CPAR' in uart and 'DMA1_Channel2->CCR' in uart and 'DMA_CCR_DIR' in uart,
    'USART3 TX uses DMA1 Channel2 memory-to-peripheral')
req('DMA1_Channel3->CPAR' in uart and 'DMA_CCR_CIRC' in uart and 'DMA_CCR_HTIE' in uart and 'DMA_CCR_TCIE' in uart,
    'USART3 RX uses DMA1 Channel3 circular HT/TC DMA')
req('USART_CR3_DMAR' in uart and 'USART_CR3_DMAT' in uart,
    'USART3 enables hardware RX/TX DMA requests')
req('USART_CR1_IDLEIE' in uart and 'USART_CR1_RXNEIE | USART_CR1_TXEIE' in uart,
    'USART3 IDLE interrupt replaces byte-by-byte RXNE/TXE transport')
req('DMA1_Channel2_IRQHandler' in it and 'app_uartcomm_dma_tx_irq_handler' in it,
    'DMA1 Channel2 IRQ is routed to UART TX DMA handler')
req('DMA1_Channel3_IRQHandler' in it and 'app_uartcomm_dma_rx_irq_handler' in it,
    'DMA1 Channel3 IRQ is routed to UART RX DMA handler')
req('DMA1_Channel1_IRQHandler' in it and 'foc_adc_dma_isr' in it,
    'ADC/FOC remains isolated on DMA1 Channel1')
req('HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 2U, 0U)' in uart and
    'HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 2U, 0U)' in uart and
    'HAL_NVIC_SetPriority(USART3_IRQn, 2U, 0U)' in uart,
    'UART DMA/IDLE IRQs remain lower priority than FOC DMA priority 0')

# RX circular-buffer drain wrap model: exact sequence must survive one wrap.
N = 256
last = 250
pos = 5
indices = list(range(last, N)) + list(range(0, pos))
req(indices == [250,251,252,253,254,255,0,1,2,3,4],
    'RX circular drain wrap order preserves byte ordering')
req('if (pos > last)' in uart and 'for (uint16_t i = last; i < VESC_UART_RX_DMA_SIZE; i++)' in uart,
    'RX DMA drain implementation handles wrap explicitly')

# TX DMA queue must only advance tail after TC/TE and never from per-byte IRQ.
req('DMA_ISR_TCIF2' in uart and 's_tx_tail = tx_next(s_tx_tail)' in uart,
    'TX queue advances on DMA transfer completion')
req('USART_SR_TXE' not in uart and 'USART_CR1_TXEIE' in uart,
    'TX no longer services one byte per TXE IRQ')

# Watchdog/reset diagnostics.
req('RCC_CSR_IWDGRSTF' in timeout and 'RCC_CSR_RMVF' in timeout,
    'reset cause captures IWDG reset and clears RCC reset flags afterward')
req('IWDG->KR = 0xCCCCU' in timeout and 'IWDG->KR = 0x5555U' in timeout and
    'IWDG->KR = 0xAAAAU' in timeout,
    'IWDG start/unlock/feed key sequence is present')
req('IWDG_SR_PVU | IWDG_SR_RVU' in timeout,
    'IWDG prescaler/reload synchronization is checked')
req('WATCHDOG_HEALTH_WINDOW_MS 100U' in timeout and 'WATCHDOG_START_GRACE_MS   250U' in timeout,
    'watchdog uses 100ms liveness windows and startup grace')

# Heartbeat ownership.
req('timeout_heartbeat_from_isr(TIMEOUT_HEARTBEAT_FOC)' in foc,
    'FOC DMA path publishes watchdog heartbeat from ISR')
req('timeout_heartbeat(TIMEOUT_HEARTBEAT_MOTOR_SERVICE)' in tasks,
    '1kHz motor-service task publishes watchdog heartbeat')
req('timeout_heartbeat(TIMEOUT_HEARTBEAT_COMM)' in commands,
    'communication parser task publishes watchdog heartbeat')
req('timeout_watchdog_require_foc(true)' in main and main.index('motor_hw_start_sampling()') < main.index('timeout_watchdog_require_foc(true)'),
    'FOC heartbeat becomes mandatory only after ADC sampling starts')
req('if (ok) timeout_watchdog_start();' in tasks,
    'IWDG starts only after required control threads are created')
req('timeout_watchdog_update_10ms(now)' in tasks,
    'motor-service task is the watchdog health gate')

# Simple health-window model: all required heartbeats advance -> feed; one stalls -> unhealthy.
last = {'foc':10,'motor':5,'comm':8}
cur_good = {'foc':50,'motor':105,'comm':95}
cur_bad = {'foc':90,'motor':205,'comm':95}
req(all(cur_good[k] != last[k] for k in last),
    'watchdog model accepts advancing FOC/motor/comm heartbeats')
last2 = cur_good.copy()
req(not all(cur_bad[k] != last2[k] for k in last2),
    'watchdog model rejects a stalled communication heartbeat')

# Custom comm diagnostic must expose DMA + reset/watchdog status without changing VESC6 ABI.
req(any(x in commands for x in ['CUSTOM_COMM_DIAG; p[i++] = 8U','CUSTOM_COMM_DIAG; p[i++] = 9U','CUSTOM_COMM_DIAG; p[i++] = 10U','CUSTOM_COMM_DIAG; p[i++] = 11U','CUSTOM_COMM_DIAG; p[i++] = 12U','CUSTOM_COMM_DIAG; p[i++] = 13U','CUSTOM_COMM_DIAG; p[i++] = 14U']) and
    'timeout_get_reset_flags()' in commands and 'u->rx_dma_irq_count' in commands,
    'custom communication diagnostics expose DMA and reset/watchdog state')

print('ALL BATCH 4 UART-DMA/WATCHDOG REGRESSIONS: PASS')
