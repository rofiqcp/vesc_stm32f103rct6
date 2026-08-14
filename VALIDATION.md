# V6 Validation

## Static source audit

Target checks:

- no UART DMA calls/handlers;
- no HAL CAN/CAN1/physical comm_can implementation;
- RXNE/TXE/TC USART path retained;
- `COMM_FORWARD_CAN` + virtual RIGHT ID2 present;
- `COMM_PING_CAN`, GET_VALUES selective/setup selective, handbrake/current-rel present;
- expected CMSIS-RTOS2 worker functions present;
- persistent flash page reserved;
- packet max payload 512.

Run:

```bash
bash tests/host/audit_v6_upstream.sh
```

## Host protocol tests

```bash
gcc -std=c11 -O2 -Wall -Wextra -Werror -Isrc \
  tests/host/test_packet_v6.c src/vesc_packet.c -o /tmp/test_packet_v6
/tmp/test_packet_v6

python3 debug_vesc_f103.py --self-test
```

Host packet test covers normal frame, 512-byte frame, CRC corruption/noise recovery, and re-synchronization. Python self-test covers framing, CRC vector, FW_VERSION parser, LEFT/local vs RIGHT/COMM_FORWARD_CAN routing, and standard COMM_SAMPLE_PRINT parser.

## Syntax audit

The modified protocol/config/timeout/app-ADC modules were checked with Clang `-fsyntax-only` against minimal HAL/CMSIS type stubs. This caught and fixed a missing `foc_control.h` include in the persistence module. These stubs are only for syntax checking and are not included in the firmware artifact.

## Cross-build limitation

The artifact environment does not contain PlatformIO or `arm-none-eabi-gcc`, therefore **an actual STM32F103 cross-build is not claimed here**. Run on the development machine:

```bash
pio run -t clean
pio run
```

Then live validation should start with UART/virtual CAN before energizing a motor:

```bash
python3 debug_vesc_f103.py handshake --port /dev/ttyUSB0
python3 debug_vesc_f103.py can-scan --port /dev/ttyUSB0
python3 debug_vesc_f103.py comm-diag --port /dev/ttyUSB0
python3 debug_vesc_f103.py config-status --port /dev/ttyUSB0
```

Only after current-zero calibration and sensor detection are valid should active motor tests be used.

## V7 handshake-first validation

Executed in host environment:

```text
test_packet_v7: PASS
audit_v7_handshake: PASS
SELF-TEST PASS: CRC, framing, V7 FW parser, virtual-CAN routing, standard sample parser
V7 host/audit tests: ALL PASS
```

The V7 audit verifies:

- no HAL UART protocol handler or UART DMA path remains;
- direct USART3 BRR/CR1/CR2/CR3 setup is present;
- USART3 RX pull-up and IRQ priority 7 are present;
- RX/error drain loop and explicit TC clear are present;
- communication resources are initialized before motor boot;
- motor initialization is moved behind `osKernelStart()` into `motor_boot_thread`;
- packet parser rejects noncanonical 16-bit short frames;
- LEFT local / RIGHT virtual CAN routing remains present.

Hardware waveform validation on PB10/PB11 is still required after flashing.

## V8 hoverboard-proven transport validation

Host validation command:

```bash
./tests/host/run_v8.sh
```

Expected:

```text
test_packet_v8: PASS
audit_v8_rofiq_transport: PASS
SELF-TEST PASS: CRC, framing, V8 VESC-6.00 FW parser, virtual-CAN routing, standard sample parser
V8 host/audit tests: ALL PASS
```

The V8 audit checks 64 MHz HSI PLL, PB10/PB11 USART3, RX DMA1_CH3 circular, TX DMA1_CH2 queue, USART3 IRQ disabled in the normal path, known VESC-6.00 FW_VERSION layout, virtual CAN RIGHT ID 2, PB2/PA4/PA5 status hardware, ADC/FOC DMA1_CH1 retention, and ISR liveness relief.
