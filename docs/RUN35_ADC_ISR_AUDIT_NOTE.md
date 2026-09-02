# Run35 ADC/ISR audit note

Run31 hardware logs proved that calibration and sampling were valid, while an active motor FOC path reached roughly 4.4k cycles and the end-to-end ISR reached roughly 5.6k cycles. Therefore the 4,000-cycle 16-kHz CPU slot was structurally too short for the complete Run31 VESC FOC pipeline.

Run35 does not delete any Run31 control feature. Instead, it restores the V15 ADC sequence exactly (5 dual ranks, no filler rank, no ADC3/DMA2) and uses DMA batching so the CPU receives one FOC IRQ after three complete PWM-synchronous ADC frames. The newest frame is stable in words 10..14 while DMA wraps to words 0..4, giving the CPU two additional PWM periods before that source region can be overwritten.

This means ADC acquisition remains 16 kHz, PWM remains 16 kHz, and full FOC control is 5.333 kHz with an exact 12,000-cycle slot at 64 MHz.

Important: this checkpoint is intended to re-test the board without the known Run31 ISR-overrun mechanism. Hardware current protection is evaluated at the FOC control cadence in this checkpoint; do the first test with wheels raised and low current (0.5 A).
