# V14 Driven Current Offset Calibration

## Root cause addressed

V13 measured final current offsets only with TIM1/TIM8 MOE disabled. The analog current-sense operating point can move when the bridge starts switching. Hall detection then enabled PWM and the first active samples were interpreted using the undriven offsets, allowing a brief false/real apparent current spike to trip `ABS_OVER_CURRENT` before normal telemetry could capture it.

VESC explicitly keeps driven and undriven offset states separate. Its low-side-shunt alternative DC calibration sets all phases to 50% modulation (zero SVM amplitude) and averages 1000 samples. V14 ports that idea to STM32F103 HAL/CMSIS-RTOS2 while keeping the hard ISR non-blocking.

## State machine

```text
UNDRIVEN (MOE OFF, 4096 samples)
  -> WAIT_LEFT_DRIVEN
  -> LEFT_WARMUP (50% zero vector, 64 ADC/FOC events)
  -> LEFT_DRIVEN (1000 decimated samples)
  -> WAIT_RIGHT_DRIVEN
  -> RIGHT_WARMUP
  -> RIGHT_DRIVEN (1000 decimated samples)
  -> WAIT_FINALIZE
  -> DONE or FAILED
```

`current_cal_thread` owns bridge enable/disable. `DMA1_Channel1_IRQHandler` only collects samples and performs bounded integer checks.

## Safety during driven calibration

Normal phase-current PI and phase `ABS_OVER_CURRENT` checking are not executed while calibration owns the bridge. Instead a gross DC-link current check compares raw DC-current ADC against the undriven baseline. A >300-count deviation (~6 A on the stock 50-count/A scaling) for 8 consecutive samples aborts calibration and clears MOE.

The 6 A value is intentionally a commissioning guard, not the final motor current limit. Normal configured phase-current protection resumes after calibration.

## Normal PWM-start blanking

After calibration, every MOE enable gets 8 motor-current-loop updates at exact 50%/50%/50% duty. Integrators are reset during these samples. This is 0.5 ms at a 16-kHz per-motor loop and avoids treating bridge-enable settling as valid FOC current feedback. A gross DC-current guard remains active during the blanking interval.

## Fault snapshot

Before the first active-drive current fault clears MOE, V14 stores:

- motor/fault/calibration stage
- raw U/V/DC ADC
- U/V/DC offsets
- reconstructed Ia/Ib/Ic Q15 and amperes (host decoded)
- trip threshold
- Id/Iq targets
- CCR1/CCR2/CCR3
- timer CNT
- DMA CNDTR
- ADC ISR count

This snapshot persists until recalibration/reset so a later diagnostic packet does not lose the transient that caused the shutdown.

## Fixed-point validation

The stock current conversion is 0.020 A/count. With a 64-A Q15 base:

```text
scale_q16 = int((0.020 / 64) * 32768 * 65536) = 671088
50 ADC counts  -> 0.998046875 A
1250 counts    -> 24.998046875 A
25 A trip      -> 12800 Q15
```

The residual conversion error is smaller than an ADC-count current step. PI integrators retain Q31 precision; sine/cosine and SVM remain fixed point in the fast loop.

## Important

The driven 50% calibration assumes the motor is stationary, consistent with the VESC low-side-shunt alternative calibration warning. Keep both wheels free and stationary during boot/recalibration.
