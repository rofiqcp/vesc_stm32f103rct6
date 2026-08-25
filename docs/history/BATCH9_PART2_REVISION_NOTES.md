# Batch 9 Part 2 of 3 — Dedicated ADC3 / DMA2 Vbus Acquisition

Baseline: `vesc_stm32f103rct6_batch9_part1_safety_thermal_rpm.zip`

## Scope

This batch deliberately changes only the DCLINK/Vbus acquisition backend and its diagnostics/safety supervision. It does **not** change:

- TIM1/TIM8 complementary PWM topology or dead-time,
- TIM1/TIM8 synchronization or the TIM8 TRGO cadence,
- ADC1/ADC2 current ranks 1..3,
- DMA1 Channel 1 half-transfer FOC entry,
- 16-kHz current-control law, observer, PLL, SVM, MTPA or field weakening,
- LEFT Hall/ABI and RIGHT Hall sensor policy,
- USART3 DMA transport, IWDG/heartbeats, flash format or VESC packet framing,
- VESC 6.00 481-byte MCCONF / 493-byte APPCONF ABI,
- the Part-1 thermal, additional-RPM-fault and startup-current backends.

## 1. Dedicated ADC3 DCLINK conversion

PC2/DCLINK is removed from the ADC1/ADC2 slow ranks and assigned to ADC3 channel 12.

ADC3 is configured as:

- one regular conversion,
- channel 12 / PC2,
- 28.5-cycle sampling,
- external trigger `TIM8_TRGO`,
- no continuous mode.

On STM32F103xE, ADC3 has its own regular-trigger encoding for TIM8_TRGO; the STM32F1 HAL maps the generic `ADC_EXTERNALTRIGCONV_T8_TRGO` setting to ADC3's correct EXTSEL value.

The current scan remains:

- rank 1: ADC1 PC1 RIGHT DC | ADC2 PC0 LEFT DC,
- rank 2: ADC1 PA0 LEFT A | ADC2 PC3 LEFT B,
- rank 3: ADC1 PC4 RIGHT B | ADC2 PC5 RIGHT C.

DMA1 Channel 1 HT still fires immediately after rank 3 and remains the only hard FOC entry.

## 2. Dedicated DMA2 Channel 5

ADC3 regular EOC requests use DMA2 Channel 5 on the STM32F103 high-density target.

The implementation uses:

- peripheral-to-memory,
- halfword peripheral and memory width,
- two-halfword circular buffer,
- very-high DMA priority,
- HT/TC interrupts disabled,
- transfer-error interrupt enabled.

The two-entry buffer is intentional. With circular length 2, DMA2_CH5 CNDTR alternates between 1 and 2 as each PWM-triggered ADC3 conversion is transferred. The hard current ISR therefore gets both the newest DCLINK sample and a cheap transfer-progress signal without adding a second 16-kHz interrupt.

## 3. Same-frame timing

The ADC clock remains 64 MHz / 6 = 10.667 MHz.

ADC3 DCLINK conversion time:

`28.5 sample cycles + 12.5 conversion cycles = 41 cycles = about 3.844 us`.

ADC1/ADC2 time to the end of current rank 3 / DMA HT:

`(1.5+12.5) + (7.5+12.5) + (7.5+12.5) = 54 cycles = about 5.063 us`.

Nominal conversion margin before FOC HT is therefore about 13 ADC cycles / 1.219 us.

This removes the previous ambiguity where ADC1 rank-4 DCLINK could still be the prior scan when the rank-3 HT ISR started.

## 4. Vbus DMA freshness fault

The FOC ISR checks DMA2_CH5 CNDTR every current frame.

- normal: CNDTR alternates 1 / 2,
- one missed/late frame: stale counter increments,
- any resumed alternation: stale counter clears,
- 3 consecutive stale/invalid observations: both bridges are emergency-disabled and `MOTOR_FAULT_ADC_DMA` is requested for both motors.

Three samples at 16 kHz correspond to 187.5 us. This keeps occasional bus-arbitration jitter from creating a one-frame false trip while refusing to run indefinitely on a frozen Vbus sample.

DMA2 transfer-error itself is handled asynchronously in `DMA2_Channel4_5_IRQHandler()` and immediately disables both bridges.

## 5. ADC1/ADC2 slow ranks after the move

ADC1/ADC2 keep six ranks so DMA half-transfer remains exactly after the first three current ranks.

Ranks 4..6 are diagnostics only. The Part-1 internal MCU-temperature conversion remains ADC1 rank 5 with 239.5-cycle sample time, after the hard FOC HT boundary. The complete dual-ADC scan duration is therefore unchanged from Part 1.

## 6. Diagnostics

Custom current-calibration diagnostics are bumped to revision 16 and append, after the complete revision-15 prefix:

- ADC3 enabled state,
- DMA2_CH5 enabled state,
- DMA2_CH5 CNDTR,
- both raw ADC3 DCLINK buffer entries,
- Vbus stale-frame count,
- cumulative stale events,
- ADC3 CR1/CR2/SQR1/SQR3,
- DMA2_CH5 CCR/CNDTR,
- DMA2 ISR.

`tools/debug.py` understands the appended revision-16 block. Standard VESC packets and VESC6 configuration ABI are unchanged.

## Verification

The following source/host regressions pass together:

- `python tools/verify_vesc_port.py`
- `python tools/test_batch2.py`
- `python tools/test_batch3.py`
- `python tools/test_batch4.py`
- `python tools/test_batch5.py`
- `python tools/test_batch6.py`
- `python tools/test_batch7.py`
- `python tools/test_batch8.py`
- `python tools/test_batch9_part1.py`
- `python tools/test_batch9_part2.py`
- `python tools/debug.py --self-test`

The historical Part-1 assertion that `mcpwm_foc.c` is byte-identical to Batch 8 is conditionally superseded only when the explicit Part-2 ADC3 Vbus marker is present. All Part-1 functional checks remain active.

## Hardware validation still required

This environment has no PlatformIO / `arm-none-eabi-gcc` installation and no physical hoverboard controller. Before power testing:

1. run `pio run` on the actual project;
2. logic-power only: verify ADC3 and DMA2_CH5 are enabled in revision-16 diagnostics;
3. verify DMA2_CH5 CNDTR alternates and stale counters stay at zero;
4. compare raw ADC3 DCLINK against a DMM over several bus voltages;
5. scope TIM8 update/current sampling if available;
6. test low-current motor operation before regenerative or loaded tests;
7. deliberately break ADC3 DMA in a test build and confirm both MOEs are cleared and ADC/DMA fault is reported.

Do not treat host regressions as proof of safe full-power operation.
