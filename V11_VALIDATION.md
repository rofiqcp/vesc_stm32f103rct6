# V11 Validation Status

## Host/source audit — PASS

`tests/host/run_v11.sh` currently verifies:

- V8 UART DMA transport SHA freeze.
- packet framing/CRC regression.
- 64-MHz F103 clock constants and 16-kHz center-aligned PWM constants.
- TIM1 -> TIM8 -> TIM2 VESC dual-motor timing topology.
- TIM2 CC2 ADC1 external trigger.
- ADC2 `ADC_SOFTWARE_START` slave configuration.
- ADC dual regular simultaneous mode.
- DMA1_CH1 circular very-high-priority setup.
- first three packed ADC ranks are current channels and DMA HT is used.
- FOC/TIM2 ISR contains no RTOS calls.
- TIM1 direction selects one motor per V0/V7 current-control event.
- forced detect angle overrides Hall/encoder angle.
- exact VESC6 MCCONF/APPCONF sizes and early GET_MCCONF safety.
- standard duty/current/RPM/POS command scaling and virtual RIGHT routing.
- Detect-All does not fake R/L/flux success.
- no physical CAN dependency.
- LED heartbeat is not driven by UART packet reception.

Last host result: `V11 host/audit tests: ALL PASS`.

## Not yet proven in this environment

The runtime here does not have PlatformIO or `arm-none-eabi-gcc`, and no STM32F103RCT6 board is attached. Therefore this release does **not** claim an ARM cross-build or physical PWM/ADC waveform has been verified here.

Board validation must use the diagnostic commands and a logic analyzer/oscilloscope. The most important first measurement is `adc_isr_count`: if it does not increase, do not troubleshoot Hall/PID/config yet—fix TIM2_CC2 -> ADC1/ADC2 -> DMA1_CH1 first.
