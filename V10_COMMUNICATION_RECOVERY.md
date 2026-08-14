# V10 communication recovery

## Frozen known-good path

- USART3 PB10/PB11, 115200 8N1
- RX DMA1 Channel3 circular, parser in `packet_process_thread`
- TX queue + DMA1 Channel2; TX ISR only advances transport state
- V8 `COMM_FW_VERSION` wire identity retained exactly
- UART always selects LEFT local; RIGHT only through `COMM_FORWARD_CAN` ID 2
- `COMM_PING_CAN` advertises virtual ID 2

## V9 regression isolated

V9 added hand-written MCCONF/APPCONF wire images and synchronous config/store paths to the packet dispatcher. V10 removes those experimental commands from the live dispatcher and does not import V9 full-wire config on boot. This is intentional: connection reliability is restored first, then exact VESC 6.00 configuration serialization can be reintroduced behind a blocking config worker.

## ISR/thread boundary

- RX DMA only moves bytes. No packet parsing in DMA ISR.
- `packet_process_thread` drains the circular buffer and calls `vesc_packet_process_byte`.
- Commands run in thread context.
- Detect operations are queued to `blocking_thread`.
- TX packet encoding happens in task context under a mutex.
- DMA1 Channel2 ISR does not parse packets, touch flash, run motor control, or call blocking RTOS APIs.

## First test

1. Flash V10.
2. Confirm startup buzzer and PB2 heartbeat.
3. Connect VESC Tool at 115200.
4. Local must identify with the same V8 firmware-version envelope.
5. CAN scan should show only virtual ID 2.
6. Only after local identification is stable, test current/duty/RPM/POS at low setpoints.
