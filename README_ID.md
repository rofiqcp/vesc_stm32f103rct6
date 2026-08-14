# V11 — VESC Timing / ADC-DMA Recovery for STM32F103RCT6

V11 keeps the **known-working V8 VESC Tool transport frozen** and repairs the motor-side timing and data path to follow the current VESC FOC execution model as closely as practical on STM32F103RCT6 + STM32 HAL + CMSIS-RTOS2.

## Frozen communication path

- USART3 PB10 TX / PB11 RX, 115200 8N1.
- RX: DMA1 Channel 3 circular, parser drains bytes in `packet_process_thread`.
- TX: DMA1 Channel 2 queued DMA.
- No packet parser, config serializer, flash write, or motor command execution inside UART/DMA ISR.
- LEFT is local motor; RIGHT is virtual `COMM_FORWARD_CAN` node ID 2.

The four V8 transport files are protected by `audit/V8_TRANSPORT_FROZEN.sha256`.

## V11 motor timing path

```
TIM1 RIGHT, center aligned 16 kHz, master TRGO=ENABLE
        |
        +--> TIM8 LEFT, ITR0 trigger slave, 180-degree counter phase
                    |
                    +--> TIM8 TRGO=UPDATE
                              |
                              +--> TIM2 ITR1 RESET slave
                                      CH2 compare = sample offset/2
                                              |
                                              +--> ADC1 external T2_CC2
                                                   + ADC2 regular simultaneous
                                                        |
                                                        +--> DMA1_CH1 circular
                                                             HT after rank 3
                                                                  |
                                                                  +--> foc_adc_dma_isr()
```

ADC ranks are arranged so all two-shunt phase currents for LEFT and RIGHT plus both DC-current channels are already present by DMA half transfer. Slower DCLINK ranks finish in the second half of the scan.

Each ADC event performs the full current loop for exactly one motor. TIM1 `DIR` selects the V0/V7 half; because this PCB maps RIGHT=TIM1 and LEFT=TIM8, DIR=0 services LEFT and DIR=1 services RIGHT. Each motor therefore receives a 16-kHz current-loop update while the shared DMA fast ISR occurs twice per PWM period.

## Hard-real-time boundary

`DMA1_Channel1_IRQHandler` and `TIM2_IRQHandler` do not call CMSIS-RTOS2, FreeRTOS, UART, printf, malloc or flash. The FOC ISR uses fixed-point Clarke/Park/current-PI/inverse-Park/SVM and direct timer-register updates. Slow control, telemetry, config, detect and persistence remain in RTOS threads.

## Sensor detect

Hall and LEFT encoder detect use forced electrical phase before the normal sensor selector. This means the PWM/FOC path can produce a controlled rotating field without requiring a valid Hall/encoder angle first. RIGHT remains Hall-only; LEFT PB6/PB7 can be Hall V/W or TIM4 encoder A/B, never both simultaneously.

`COMM_DETECT_APPLY_ALL_FOC` does **not** fake a successful full motor detect. Until physical R/L/flux measurement routines are ported and verified, it returns the VESC-style flux-detect failure code (`-10`) after calibration instead of storing invented parameters. Individual Hall/encoder detection remains available.

## Motor/App configuration

- VESC 6.00 MCCONF wire size: 481 bytes.
- VESC 6.00 APPCONF wire size: 493 bytes.
- `GET_MCCONF` is safe even when it arrives immediately after FW_VERSION, before motor boot finishes.
- SET config executes in the blocking worker, never in RX/DMA ISR.
- V8 physical UART settings remain immutable even if App Config is written.
- Persistent config uses the final 8 KiB of STM32 flash with version, sequence and CRC; firmware linker is restricted to 248 KiB.

## Runtime diagnosis

Use:

```bash
python3 debug_vesc_f103.py handshake --port /dev/ttyUSB0 --baud 115200
python3 debug_vesc_f103.py calibrate --port /dev/ttyUSB0 --baud 115200 --timeout 5
```

The calibration diagnostic reports ADC-FOC ISR count, DMA CNDTR, TIM1/TIM8/TIM2 running state, ADC1/ADC2 enable and DMA1_CH1 enable. `adc_isr_count` must continuously increase before motor commands can be expected to produce torque.

See `V11_VESC_F103_TIMING.md` and `FIRST_POWER_TEST.md` before applying motor power.
