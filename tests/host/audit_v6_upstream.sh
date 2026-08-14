#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
fail(){ echo "FAIL: $*" >&2; exit 1; }
! grep -R -nE 'HAL_UART_(Transmit|Receive)_DMA|HAL_UARTEx_ReceiveToIdle_DMA|DMA1_Channel2_IRQHandler|DMA1_Channel3_IRQHandler|HAL_UART_IRQHandler' src || fail "UART DMA/HAL IRQ handler remains"
grep -q 'COMM_FORWARD_CAN' src/vesc_comm.c || fail "FORWARD_CAN missing"
grep -q 'VESC_VIRTUAL_CAN_RIGHT_ID' src/vesc_comm.c || fail "virtual right CAN missing"
grep -q 'COMM_PING_CAN' src/vesc_comm.c || fail "PING_CAN missing"
grep -q 'COMM_GET_VALUES_SETUP_SELECTIVE' src/vesc_comm.c || fail "setup selective missing"
grep -q 'COMM_SET_HANDBRAKE' src/vesc_comm.c || fail "handbrake missing"
grep -q 'COMM_SET_CURRENT_REL' src/vesc_comm.c || fail "current-rel missing"
grep -q 'static void adc_thread' src/app_adc_port.c || fail "adc_thread missing"
grep -q 'static void packet_process_thread' src/vesc_comm.c || fail "packet_process_thread missing"
grep -q 'static void blocking_thread' src/vesc_comm.c || fail "blocking_thread missing"
grep -q 'static void timeout_thread' src/vesc_timeout.c || fail "timeout_thread missing"
for t in timer_thread sample_send_thread fault_stop_thread stat_thread pid_thread rpm_thread periodic_thread led_thread; do
  grep -q "static void ${t}" src/motor_tasks.c || fail "$t missing"
done
grep -q 'CFG_FLASH_ADDR.*0x0803F800' src/config_store.c || fail "config flash page missing"
grep -q 'board_upload.maximum_size = 260096' platformio.ini || fail "flash reserve missing"
grep -q 'VESC_PACKET_MAX_PAYLOAD 512' src/vesc_packet.h || fail "512 payload limit missing"
echo "audit_v6_upstream: PASS"
