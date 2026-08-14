# V13 Validation

Host/static validation performed for V13:

- V8 USART3 DMA transport SHA freeze: PASS.
- V11 TIM1/TIM8/TIM2 + ADC1/ADC2 dual + DMA1_CH1 timing audit: PASS.
- V12 calibration diagnostics/regression: PASS.
- Current polarity `offset - raw`: PASS.
- LEFT A/B and RIGHT B/C two-shunt reconstruction: PASS.
- ABS-over-current check only while firmware PWM is active: PASS.
- Hall active-low + GPIO NOPULL: PASS.
- Hall fast-path polling/source-of-truth: PASS.
- VESC-style Hall detect 1000-ms ramp, 1 deg / 5 ms, 3 forward + 3 reverse: PASS.
- Detect timeout 60 s and restore: PASS.
- Standard detect result separated from active config: PASS.
- Clean algorithmic detect failure is non-latching: PASS.
- Custom TXT detect supports 0.2..2.0 A, default 0.5 A: PASS.
- Python syntax check: PASS.

Hardware validation is still required. Start with current-limited supply and wheel lifted. Do not use VESC Tool full Auto Detect All until physical R/L/flux routines are validated.
