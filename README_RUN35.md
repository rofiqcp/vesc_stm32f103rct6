# Run35 — Run31 Final + V15 ADC/ISR only

Base firmware: `vesc_stm32f103rct6_checkpoint_run31_adc_isr_hfi_detect_all_final_20260902.zip`.

Run35 deliberately keeps the complete Run31 source tree and feature set. The only original Run31 files changed are listed in `docs/RUN35_CHANGED_FILES.txt`. All other original files are SHA256-locked by `docs/RUN35_RUN31_UNCHANGED_SHA256.txt`.

## Timing change

- PWM remains 16 kHz.
- ADC1+ADC2 remain dual regular simultaneous, triggered by TIM8_TRGO every PWM period.
- ADC rank order and sample times are the proven hoverboard V15 five-rank layout.
- DMA stores 3 consecutive 5-word frames (15 words total).
- DMA1 Channel1 uses TC-only IRQ after the third complete frame.
- The full Run31 FOC pipeline uses the newest frame and runs at 16k/3 ≈ 5.333 kHz.
- First two frames are DMA-only sampling, so no VESC config/observer/FOC workload executes in their CPU path.
- FOC control slot is 12,000 cycles at 64 MHz, instead of the failing 4,000-cycle Run31 slot.

This preserves Run31 Detect-All, R/L/flux, Hall detect, duty/current/brake/RPM/position, EEPROM, telemetry, VESC Tool 6.00 protocol, HFI migration and debug tooling.

ID reporting is also corrected as previously requested: ID1/local = `MOTOR_LEFT`, forwarded ID2 = `MOTOR_RIGHT`.

Run `python3 tools/audit_run35.py` before flashing.
