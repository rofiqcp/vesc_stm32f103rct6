#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."

forbidden='HAL_UART_Transmit_DMA|HAL_UART_Receive_DMA|HAL_UARTEx_ReceiveToIdle_DMA|USART_CR3_DMAR|USART_CR3_DMAT|hdma_usart3_rx|hdma_usart3_tx|DMA1_Channel2_IRQHandler|DMA1_Channel3_IRQHandler|uart_process_thread|HAL_UART_IRQHandler'
if grep -RInE "$forbidden" src >/tmp/v5_uart_forbidden.txt; then
  cat /tmp/v5_uart_forbidden.txt
  echo "V5 UART AUDIT FAIL: forbidden UART DMA transport remains"
  exit 1
fi

grep -q 'void USART3_IRQHandler' src/stm32f1xx_it.c
grep -q 'USART_SR_RXNE' src/stm32f1xx_it.c
grep -q 'USART_CR1_TXEIE' src/stm32f1xx_it.c
grep -q 'USART_CR1_TCIE' src/stm32f1xx_it.c
grep -q 'vesc_uart_rx_isr_put' src/stm32f1xx_it.c
grep -q 'vesc_uart_tx_isr_get' src/stm32f1xx_it.c
grep -q 'VESC_UART_RX_BUF_SIZE 256U' src/vesc_uart.h
grep -q 'VESC_UART_TX_BUF_SIZE 2048U' src/vesc_uart.h
grep -q 'DMA1_Channel1_IRQHandler' src/stm32f1xx_it.c

echo "V5 UART AUDIT PASS: RXNE/TXE/TC software-ring transport, UART DMA absent, ADC/FOC DMA1_CH1 retained"
