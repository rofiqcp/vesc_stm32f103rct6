# V14 Validation

Host/source validation completed:

- V8 UART-DMA transport SHA freeze: PASS
- V11 TIM1/TIM8/TIM2 + ADC1/ADC2 + DMA1_CH1 half-transfer topology: PASS
- V13 current polarity / LEFT A+B / RIGHT B+C / Hall detect regression: PASS
- V14 driven/undriven offset calibration state machine: PASS
- ISR blocking-call audit: PASS
- fixed-point current conversion arithmetic: PASS
- packet/framing/VESC6/virtual CAN self-test: PASS
- Python diagnostic syntax: PASS

No ARM cross compiler or physical STM32F103RCT6 is available in this environment, so hardware build/waveform validation must be performed on the target board.
