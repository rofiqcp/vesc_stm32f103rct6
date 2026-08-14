#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
fail(){ echo "AUDIT V8 FAIL: $*" >&2; exit 1; }

# Hoverboard clock and board liveness.
grep -q 'CPU_CLOCK_HZ[[:space:]]*64000000UL' src/app_config.h || fail "CPU clock not 64 MHz"
grep -q 'RCC_PLLSOURCE_HSI_DIV2' src/main.c || fail "HSI/2 PLL source missing"
grep -q 'RCC_PLL_MUL16' src/main.c || fail "PLL x16 missing"
grep -q 'status_io_early_gpio_init' src/main.c || fail "early PB2/PA4/PA5 status init missing"
grep -q 'HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET)' src/status_io.c || fail "PA5 power hold HIGH missing"
grep -q 'startup_melody' src/motor_tasks.c || fail "startup melody missing"
grep -q 'buzzer_thread' src/motor_tasks.c || fail "buzzer thread missing"
grep -q 'led_thread' src/motor_tasks.c || fail "LED thread missing"

# Exact user pin map essentials.
grep -q '#define LED_PIN GPIO_PIN_2' src/board_pins.h || fail "PB2 LED missing"
grep -q '#define LED_PORT GPIOB' src/board_pins.h || fail "PB2 LED port wrong"
grep -q '#define BUZZER_PIN GPIO_PIN_4' src/board_pins.h || fail "PA4 buzzer missing"
grep -q '#define BUZZER_PORT GPIOA' src/board_pins.h || fail "PA4 buzzer port wrong"
grep -q '#define VESC_UART_TX_PIN GPIO_PIN_10' src/board_pins.h || fail "PB10 TX missing"
grep -q '#define VESC_UART_RX_PIN GPIO_PIN_11' src/board_pins.h || fail "PB11 RX missing"

# Proven VESC Tool transport: USART3 + RX CH3 circular + TX CH2 queue.
grep -q 'VESC_UART_BAUD[[:space:]]*115200U' src/app_config.h || fail "115200 missing"
grep -q 'DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_CIRC' src/vesc_uart.c || fail "RX circular DMA CH3 missing"
grep -q 'VESC_UART->CR3 |= USART_CR3_DMAR' src/vesc_uart.c || fail "USART DMAR missing"
grep -q 'DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_TEIE' src/vesc_uart.c || fail "TX DMA CH2 config missing"
grep -q 'VESC_UART->CR3 |= USART_CR3_DMAT' src/vesc_uart.c || fail "USART DMAT missing"
grep -q 'VESC_UART_RX_DMA_SIZE[[:space:]]*1024U' src/vesc_uart.h || fail "1024-byte RX DMA missing"
grep -q 'VESC_UART_TX_QUEUE_DEPTH[[:space:]]*6U' src/vesc_uart.h || fail "6-deep TX queue missing"
grep -q 'HAL_NVIC_DisableIRQ(USART3_IRQn)' src/vesc_uart.c || fail "USART3 IRQ not disabled"
grep -q 'HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn)' src/vesc_uart.c || fail "RX DMA IRQ not disabled"
grep -q 'vesc_uart_service();' src/vesc_comm.c || fail "packet thread DMA service missing"
grep -q 'DMA1_Channel2_IRQHandler' src/stm32f1xx_it.c || fail "TX DMA IRQ handler missing"

# VESC 6.00 handshake known to connect in reference firmware.
grep -q 'p\[i++\] = 6U' src/vesc_comm.c || fail "FW major 6 missing"
grep -q 'p\[i++\] = 0U' src/vesc_comm.c || fail "FW minor 0 missing"
grep -q 'HOVERBOARD_DUAL_FOC' src/vesc_comm.c || fail "known HW name missing"
grep -q 'pairing done' src/vesc_comm.c || fail "FW version pairing field missing"

# Left local / right virtual CAN must stay.
grep -q 'COMM_FORWARD_CAN' src/vesc_comm.c || fail "forward CAN missing"
grep -q 'VESC_VIRTUAL_CAN_RIGHT_ID' src/vesc_comm.c || fail "virtual right CAN ID missing"
grep -q 'COMM_PING_CAN' src/vesc_comm.c || fail "ping CAN missing"
! grep -RInE '\bHAL_CAN_|\bCAN1\b|CAN_HandleTypeDef' src >/dev/null || fail "physical CAN hardware unexpectedly present"

# FOC CPU liveness and ADC DMA stay independent.
grep -q 'DMA1_Channel1_IRQHandler' src/stm32f1xx_it.c || fail "ADC/FOC DMA CH1 missing"
grep -q 'shed_next' src/stm32f1xx_it.c || fail "FOC liveness shed missing"
grep -q 'foc_adc_dma_quick_guard_isr' src/stm32f1xx_it.c || fail "short safety pass missing"
grep -q 'CPU_CLOCK_HZ / PWM_FREQUENCY_HZ' src/foc_control.c || fail "FOC 4000-cycle period formula missing"

# Old IRQ-ring transport must no longer be the live path.
! grep -RInE 'vesc_uart_rx_isr_put|vesc_uart_tx_isr_get|VESC_RX_AVAILABLE|VESC_TX_COMPLETE' src >/dev/null || fail "old RXNE/TXE ring path remains"

echo "audit_v8_rofiq_transport: PASS"
