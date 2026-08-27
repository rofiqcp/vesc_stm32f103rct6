#!/usr/bin/env python3
from pathlib import Path
import re, sys

root = Path(__file__).resolve().parents[1]
app = (root/'src/applications/app_uartcomm.c').read_text(errors='replace')
it = (root/'src/stm32f1xx_it.c').read_text(errors='replace')
cmd = (root/'src/comm/commands.c').read_text(errors='replace')
main = (root/'src/main.c').read_text(errors='replace')
hw = (root/'src/hwconf/hw.c').read_text(errors='replace')
issues=[]

def need(text, token, desc):
    if token not in text: issues.append(f'missing: {desc}')

def forbid(text, token, desc):
    if token in text: issues.append(f'forbidden: {desc}')

for token,desc in [
    ('huart3_vesc.Instance = USART3','USART3 handle'),
    ('DMA1_Channel3','USART3 RX DMA channel3'),
    ('DMA_CIRCULAR','circular RX DMA'),
    ('HAL_UART_Receive_DMA','HAL RX DMA start'),
    ('DMA1_Channel2','USART3 TX DMA channel2'),
    ('DMA_NORMAL','normal TX DMA'),
    ('HAL_UART_Transmit_DMA','HAL TX DMA start'),
    ('huart3_vesc.gState == HAL_UART_STATE_READY','TX-only UART gState readiness'),
    ('HAL_DMA_GetState(&hdma_usart3_tx) == HAL_DMA_STATE_READY','TX DMA readiness'),
]: need(app,token,desc)

# The old combined state test is fatal with permanent BUSY_RX circular DMA.
if re.search(r'HAL_UART_GetState\s*\([^)]*huart3_vesc[^)]*\)\s*==\s*HAL_UART_STATE_READY', app):
    issues.append('combined HAL_UART_GetState READY check still present')

for token,desc in [
    ('void DMA1_Channel2_IRQHandler(void)','TX DMA IRQ'),
    ('app_uartcomm_dma_tx_irq_handler();','TX DMA HAL dispatch'),
    ('void DMA1_Channel3_IRQHandler(void)','RX DMA IRQ'),
    ('app_uartcomm_dma_rx_irq_handler();','RX DMA HAL dispatch'),
]: need(it,token,desc)

# Match SmartESC topology: VESC management UART does not depend on USART3 IRQ.
forbid(it, 'void USART3_IRQHandler(void)', 'USART3 IRQ handler should not be required')
forbid(app, 'HAL_UART_IRQHandler(&huart3_vesc)', 'USART3 HAL IRQ dependency')
need(app, 'HAL_NVIC_DisableIRQ(USART3_IRQn)', 'USART3 NVIC disabled')
need(app, 'UART_FLAG_TC', 'task-context final stop-bit TC polling')

for token,desc in [
    ('case COMM_FW_VERSION:','COMM_FW_VERSION dispatcher'),
    ('reply_fw_version();','COMM_FW_VERSION reply'),
    ('vesc_packet_process_byte(&s_parser, byte, process_payload);','packet parser in packet task'),
    ('case COMM_FORWARD_CAN:','COMM_FORWARD_CAN dispatcher'),
    ('mc_interface_select_motor_thread(2);','upstream-style local motor-2 thread selection'),
    ('process_payload_for_motor(&data[2], (uint16_t)(len - 2U), MOTOR_RIGHT);','recursive local motor-2 inner command'),
    ('mc_interface_select_motor_thread(1);','forwarding restores primary motor-thread exactly like upstream'),
    ('const char *hw = "MOTOR_LEFT";','same HW name for both local motor contexts'),
    ('if (mc_interface_get_motor_thread() == 2) p[i - 1U]++;','second-motor UUID +1 identity from thread context'),
]: need(cmd,token,desc)

if 'const char *hw = (id == MOTOR_RIGHT)' in cmd:
    issues.append('motor-2 FW_VERSION must not report a different HW name from motor-1')


# STM32F1 SWD safety: never use generic MAPR RMW remap helpers after boot.
# SWJ_CFG readback is unsafe; the firmware owns one explicit final MAPR write.
for token, desc in [
    ('afio_apply_vesc_mapr_once()', 'single explicit AFIO MAPR setup'),
    ('AFIO_SWJ_CFG_MASK_LOCAL', 'explicit SWJ_CFG mask'),
    ('AFIO_SWJ_JTAG_OFF_SWD_ON_LOCAL', 'JTAG-off/SWD-on value'),
    ('mapr &= ~AFIO_SWJ_CFG_MASK_LOCAL;', 'SWJ readback bits discarded before write'),
    ('mapr |= AFIO_SWJ_JTAG_OFF_SWD_ON_LOCAL;', 'SWD explicitly preserved'),
    ('AFIO->MAPR = mapr;', 'single controlled MAPR write'),
]: need(hw, token, desc)
forbid(hw, '__HAL_AFIO_REMAP_SWJ_NOJTAG()', 'generic SWJ MAPR RMW helper')
forbid(hw, '__HAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE()', 'generic ADC MAPR RMW helper')
if re.search(r'AFIO->MAPR\s*[|&^]?=', app):
    issues.append('UART MSP must not modify AFIO->MAPR; default PB10/PB11 mapping is used')

# Communication must be initialized before motor_hw_init in main.
comm_pos = main.find('commands_init();')
motor_pos = main.find('motor_hw_init();')
if comm_pos < 0 or motor_pos < 0 or comm_pos > motor_pos:
    issues.append('boot order: commands/UART must initialize before motor_hw_init')

if issues:
    print('COMM AUDIT: FAIL')
    for x in issues: print(' -',x)
    sys.exit(1)
print('COMM AUDIT: PASS')
print('USART3 PB10/PB11: RX DMA1_Ch3 circular -> task CNDTR parser; TX DMA1_Ch2 + task-polled USART TC')
print('TX readiness: huart3_vesc.gState only (RxState BUSY_RX allowed)')
print('COMM_FW_VERSION dispatcher/reply present; dual-motor forwarding restores thread 1; UART initialized before motor_hw_init')
print('AFIO/SWD: one controlled MAPR write; JTAG disabled, SWD preserved, USART3 PB10/PB11 + ADC1 TIM8 trigger retained')
