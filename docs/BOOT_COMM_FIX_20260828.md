# Boot / VESC UART fix (2026-08-28)

Observed hardware debug result: `g_vesc_boot_stage=90`, `g_vesc_boot_error=91` before the FreeRTOS scheduler. USART3 itself was configured and RX DMA was active, but `packet_process` could never run.

Root cause: `init_adc_dma()` configures APP ADC PA2/PA3 at ADC1/ADC2 regular **rank 4**, while `motor_hw_sampling_contract_flags()` incorrectly validated those channels at **rank 6**. Rank 6 is intentionally the thermal/spare slot, so the contract failed deterministically on every boot.

Fixes:
- Validate PA2/PA3 at rank 4.
- Export `g_vesc_sampling_contract_flags` to GDB.
- If a genuine sampling contract failure occurs, keep PWM hard-off and motor commands inhibited, but still start FreeRTOS so USART3 `packet_process` remains available for COMM_FW_VERSION and diagnostics.
- PlatformIO default environment is `stm32f103rc_debug`.

Expected successful debug boot: `stage=100 error=0 sampling_flags=0x00000000`, with packet_process selected and USART3 RX DMA CNDTR changing when host bytes arrive.
