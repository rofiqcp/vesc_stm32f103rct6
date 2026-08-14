# V11 VESC FOC Timing Audit — STM32F103RCT6 + HAL + CMSIS-RTOS2

## 1. Why V10 could communicate but not drive the motor

V10 recovered the proven V8 UART transport, but the motor-side sampling topology was still not sufficiently aligned with VESC FOC timing. A valid packet could reach the dispatcher while the ADC/current-loop proof remained invalid; in that state the motor safety gate correctly kept TIM1/TIM8 MOE disabled. This produces the observed combination of communication activity, blank/partial telemetry and no reaction to DUTY/CURRENT/RPM/POS.

V11 therefore treats communication and motor sampling as two independent liveness domains.

## 2. VESC timing model used by this port

Current VESC `mcpwm_foc.c` configures TIM1/TIM8 as center-aligned PWM timers and TIM2 as the current-sampling timer. In the dual-motor topology TIM1 is a master, TIM8 is a trigger slave, TIM8 update feeds TIM2 reset, and TIM2 CH2 provides the ADC sampling offset. The TIM2 CC2 event also generates the COM synchronization event.

The VESC ADC handler identifies V0/V7 from TIM1 direction. For dual motor it uses that half-cycle to select which motor executes the full FOC path. V11 preserves this timing principle while swapping logical motor names to match this PCB: TIM1 is RIGHT and TIM8 is LEFT.

## 3. F103 implementation

Clock:

- HSI 8 MHz / 2 * PLL16 = 64 MHz SYSCLK/HCLK.
- APB1 = 32 MHz, but TIM2/TIM4 timer clock = 64 MHz because APB1 prescaler is 2.
- APB2 = 64 MHz, therefore TIM1/TIM8 = 64 MHz.
- ADC clock = PCLK2 / 6 = about 10.67 MHz, below the STM32F103 ADC maximum.

PWM:

- TIM1 RIGHT and TIM8 LEFT: center-aligned mode 1.
- ARR = 64,000,000 / (2 * 16,000) = 2000.
- TIM1 counter starts at 0; TIM8 starts at TIM1 ARR for the dual-motor phase relationship.
- CCR1/2/3 preload enabled; FOC writes direct CCR values.
- High-side active HIGH, complementary low-side active LOW as required by this power stage.
- MOE remains off until current calibration and command/detect safety checks pass.

Sampling timer:

- TIM2 prescaler 0, up counter, ARR 0xFFFF.
- TIM2 CH2 PWM1, CCR2 = 250 for the baseline sample offset 500/2.
- TIM2 is reset from TIM8 TRGO update.
- TIM2 CC2 is the ADC1 regular external trigger.
- TIM2 CC2 ISR is deliberately tiny and contains no RTOS calls.

## 4. F103 ADC1+ADC2 dual regular simultaneous

STM32F1 differs from the F4 target used by much of current upstream VESC. The port therefore uses the STM32F1 HAL multimode API instead of copying F4 ADC registers blindly.

- ADC1 = multimode master, external trigger `ADC_EXTERNALTRIGCONV_T2_CC2`.
- ADC2 = multimode slave and is configured with `ADC_SOFTWARE_START`, as required by STM32F1 HAL for the slave regular group.
- Mode = `ADC_DUALMODE_REGSIMULT`.
- DMA = DMA1 Channel1, peripheral/memory word width, circular, very-high priority.
- The packed DMA word contains ADC1 in the lower 16 bits and ADC2 in the upper 16 bits.

Six packed ranks are used so DMA half-transfer happens after exactly three ranks. Those first three ranks contain all current measurements needed by the two two-shunt bridges:

1. LEFT phase U (ADC1) + LEFT phase V (ADC2)
2. RIGHT phase U (ADC1) + RIGHT phase V (ADC2)
3. LEFT DC current (ADC1) + RIGHT DC current (ADC2)
4-6. DCLINK/auxiliary slow samples

The FOC ISR begins at DMA half-transfer, while the voltage/aux ranks finish in the background. DCLINK uses the last completed slow value, so startup warmup is discarded before offset calibration.

## 5. ISR policy

`DMA1_Channel1_IRQHandler`:

- TE: immediately clears bridge MOE and records ADC-DMA fault.
- HT: clears HT and calls `foc_adc_dma_isr()`.
- no RTOS API, UART, flash, printf or blocking HAL.

`TIM2_IRQHandler`:

- on CC2, issues TIM1/TIM8 COM events and clears CC2.
- no RTOS API.

At 16-kHz PWM there are two V0/V7 sampling events per PWM period, so the shared fast ISR occurs at 32 kHz. One motor is processed per event and each motor gets a 16-kHz current-loop update. The hard per-event slot at 64 MHz is 2000 cycles. Persistent execution beyond that slot is treated as a deadline fault rather than intentionally skipping current-control updates.

## 6. Fixed-point FOC path

Hot path:

```
ADC raw -> offset/scale -> ia/ib/ic Q15
       -> Clarke Q15
       -> forced/Hall/encoder phase uint16
       -> interpolated sin/cos Q15
       -> Park Q15
       -> Id/Iq PI (Q16.16 gains + Q31 integrator)
       -> voltage magnitude limiting
       -> inverse Park Q15
       -> SVM Q15
       -> TIM1/TIM8 CCR preload
```

No `sinf`, `cosf`, `atan2f`, `sqrtf`, heap allocation, mutex or packet output is used in the normal current-loop ISR.

## 7. Hall / Encoder detection without a pre-existing sensor angle

Detection must be able to generate a rotating stator field before the requested rotor sensor is known. V11 therefore gives `detect_force_angle` precedence over Hall/encoder phase selection. The detect worker sets low D-axis current plus a forced electrical angle and slowly sweeps that angle while the ISR still performs normal current control and SVM.

Hall detect accumulates the six valid raw Hall states over forward/reverse forced-angle sweeps. LEFT encoder detect switches PB6/PB7 to TIM4 TI12 hardware quadrature only with PWM off, then performs controlled phase sweeps to determine direction, ratio/offset quality and establishes a per-boot synchronization reference. RIGHT encoder detect returns unsupported.

## 8. Full Auto Detect

The current upstream VESC Detect-All sequence performs DC offset calibration, then motor R/L/current-limit measurement, flux-linkage measurement, then sensor detection and only commits configuration after success. V11 does not yet have board-validated physical R/L/flux routines, so `COMM_DETECT_APPLY_ALL_FOC` deliberately reports `-10` instead of claiming success. This prevents unsafe or meaningless motor parameters from being committed.

Individual Hall and LEFT encoder detect are implemented and use the sensor-independent forced-phase FOC path.

## 9. RTOS2 mapping

Hard realtime remains outside the kernel. CMSIS-RTOS2 threads handle:

- packet processing and command dispatch;
- blocking detect/config jobs;
- current-calibration supervision;
- VESC timer/management equivalent;
- speed/position PID outer loop;
- RPM/rotor state updates;
- sample sender;
- fault stop/history;
- statistics/telemetry snapshots;
- periodic/status work;
- timeout;
- LED/buzzer.

The communication parser never runs in UART DMA ISR, matching the VESC architecture boundary even though this F103 board keeps the V8 proven DMA transport instead of ChibiOS SerialDriver RXNE queues.

## 10. Definition of first hardware success

Before testing torque, all of these must be true:

1. VESC Tool FW handshake works.
2. PB2 normal heartbeat is independent of UART traffic.
3. `calibrate` reports ADC1, ADC2, DMA1_CH1, TIM1, TIM8 and TIM2 enabled.
4. `adc_isr_count` increases continuously.
5. Current offset calibration finishes and is valid.
6. DCLINK telemetry is plausible against a DMM.
7. TIM1/TIM8 six gate signals are 16-kHz center-aligned, complementary and dead-time safe with MOE controlled.
8. Only then test 0.5 A current, ±0.02 duty and low eRPM with the wheel lifted and current-limited supply.
