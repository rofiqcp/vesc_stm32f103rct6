#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
fail(){ echo "FAIL: $*" >&2; exit 1; }

# VESC UART must be software-ring IRQ transport, never HAL/DMA UART.
! grep -R -nE 'HAL_UART_Init[[:space:]]*\(|HAL_UART_IRQHandler[[:space:]]*\(|HAL_UART_(Transmit|Receive)_DMA[[:space:]]*\(|HAL_UARTEx_ReceiveToIdle_DMA[[:space:]]*\(|void[[:space:]]+DMA1_Channel[23]_IRQHandler[[:space:]]*\(' src || fail "HAL/DMA UART transport remains"
grep -q 'VESC_UART->BRR = HAL_RCC_GetPCLK1Freq() / VESC_UART_BAUD' src/vesc_uart.c || fail "direct BRR setup missing"
grep -q 'USART_CR1_RXNEIE' src/vesc_uart.c || fail "RXNEIE missing"
grep -q 'USART_CR1_TXEIE' src/vesc_uart.c || fail "TXEIE missing"
grep -q 'USART_CR1_TCIE' src/stm32f1xx_it.c || fail "TCIE handling missing"
grep -q 'GPIO_PULLUP' src/vesc_uart.c || fail "RX pull-up missing"
grep -q 'HAL_NVIC_SetPriority(USART3_IRQn, 7U, 0U)' src/vesc_uart.c || fail "USART priority not upstream 115200 value"
grep -q 'while ((sr & (USART_SR_RXNE' src/stm32f1xx_it.c || fail "upstream RX drain loop missing"
grep -q 'VESC_UART->SR = (uint16_t)~USART_SR_TC' src/stm32f1xx_it.c || fail "explicit TC clear missing"

# Handshake must be boot-independent from motor/ADC/FOC.
python3 - <<'PY'
from pathlib import Path
s=Path('src/main.c').read_text()
a=s.index('osKernelInitialize()')
b=s.index('vesc_comm_task_init()')
c=s.index('osKernelStart()')
assert a < b < c
# Motor init is executed by the RTOS boot thread, not before kernel start.
main_body=s[s.index('int main(void)'):s.index('static void motor_boot_thread')]
assert 'motor_hw_init();' not in main_body
boot=s[s.index('static void motor_boot_thread')+1:]
assert 'motor_hw_init();' in boot and 'motor_hw_start_sampling();' in boot
PY

# Packet parser must use canonical upstream 8/16-bit length encoding for max=512.
grep -q 'if (payload_len < 255U) return -1' src/vesc_packet.c || fail "noncanonical 16-bit length not rejected"

# Local LEFT and virtual CAN RIGHT semantics remain present.
grep -q 'COMM_FORWARD_CAN' src/vesc_comm.c || fail "FORWARD_CAN missing"
grep -q 'VESC_VIRTUAL_CAN_RIGHT_ID' src/vesc_comm.c || fail "virtual right CAN missing"

echo "audit_v7_handshake: PASS"
