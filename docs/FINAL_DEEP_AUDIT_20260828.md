# Final deep audit — 2026-08-28

References used:
- vedderb/bldc VESC 6.00: command ABI, dual-motor forwarding, MCCONF/APPCONF semantics.
- Koxx3/SmartESC_STM32_v2: USART3 PB10/PB11 circular RX DMA + CNDTR task parser.
- EFeru/hoverboard-firmware-hack-FOC: stock hoverboard pin map, PWM 16 kHz, A2BIT_CONV=50, I_MOT_MAX=15 A, I_DC_MAX=17 A.

Final fixes:
- Python COMM_GET_VALUES_SELECTIVE constant 50.
- GET_VALUES M1/M2 uses ISR seqlock snapshot; averages are copied/reset under a very short critical section instead of blocking the command task on telemetry mutex.
- Virtual-CAN command trace exported to GDB/OpenOCD.
- RX DMA is 1024 bytes; UART parser stays SmartESC-style CNDTR polling in packet_process.
- 17 A startup DC and 25 A absolute-current software trips require two consecutive 16-kHz samples (125 us); hardware/BKIN protection remains immediate and no current limit is raised.
- Correct buzzer half-period math, boot chirp, startup melody/fault-tone debug state.
- Removed duplicate sample packet send.
- MCCONF remains a complete VESC-6.00 wire image; hardware-limit clamps are explicit and read back consistently.
