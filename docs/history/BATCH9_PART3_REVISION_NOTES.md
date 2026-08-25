# Batch 9 Part 3 — Power-Stage Safety and Zero-Vector Low-Side Brake

Baseline: `vesc_stm32f103rct6_batch9_part2_adc3_vbus_dma.zip`

## Scope

Part 3 deliberately closes the three-part Batch-9 hardware optimization series without changing the proven PWM frequency, TIM1/TIM8 synchronization, ADC1/ADC2 current ranks, ADC3/DMA2 DCLINK acquisition, UART DMA transport, watchdog topology, or VESC-6.00 wire ABI.

Implemented:

1. STM32F1 internal PVD brownout shutdown.
2. Optional TIM1/TIM8 BKIN hardware-break backend (safe default disabled until PCB fault wiring is validated).
3. VESC-style `foc_short_ls_on_zero_duty` backend using forced-inactive complementary outputs and COM events (safe default disabled until gate-driver/bootstrap behavior is validated).
4. Explicit VESC fault mapping for MCU undervoltage and BRK.
5. Reset-latched power-stage fault policy that prevents software clear/re-enable after PVD/BKIN.
6. Non-destructive `powerstage` terminal diagnostics.

## 1. PVD brownout protection

STM32F103 PVD is enabled at the highest threshold band (`PLS=111`, nominally about 2.9 V). PVD output is routed to EXTI line 16 and handled at IRQ priority 0 without RTOS calls.

When PVDO reports VDD below threshold:

- TIM1 MOE is cleared immediately;
- TIM8 MOE is cleared immediately;
- a reset-latched `POWERSTAGE_FAULT_PVD` bit is set;
- both local motors receive `MOTOR_FAULT_MCU_UNDER_VOLTAGE`;
- the VESC-facing fault is `FAULT_CODE_MCU_UNDER_VOLTAGE`;
- later PWM enable requests are rejected until MCU reset.

This is intentionally independent from DCLINK/Vbus undervoltage. DCLINK protection supervises the motor supply; PVD supervises the MCU 3.3-V domain.

## 2. TIM1/TIM8 BKIN backend

The advanced-timer Break backend now exists, but the stock defaults are:

- `HOVERBOARD_TIM1_BREAK_ENABLE = 0`
- `HOVERBOARD_TIM8_BREAK_ENABLE = 0`

The default STM32F103 pin mappings represented by the backend are:

- TIM1_BKIN: PB12
- TIM8_BKIN: PA6

If a validated PCB revision connects a comparator/gate-driver fault to either pin, enabling the matching compile-time macro configures the input, enables timer Break and BIE, and installs the break IRQ.

The hardware Break action clears MOE asynchronously. The IRQ then:

- clears/latches the corresponding TIM1/TIM8 break source;
- clears both bridge MOEs because both inverters share the same board power domain;
- reports `MOTOR_FAULT_BREAK` / `FAULT_CODE_BRK` for both local motors;
- prevents any automatic output re-enable (`AutomaticOutput = DISABLE`).

Do **not** enable BKIN merely because the MCU supports it. First verify the actual PCB net and active polarity.

## 3. VESC-style zero-vector low-side brake

A real runtime backend for `foc_short_ls_on_zero_duty` is added. It remains **false by default**.

The hard FOC path enters full low-side brake only when:

- PWM is already enabled;
- no motor/power-stage fault is active;
- `foc_short_ls_on_zero_duty == true`; and
- `du == dv == dw` exactly.

For the hoverboard polarity used by this firmware, timer `forced inactive` produces:

- CHx low -> high-side input OFF;
- CHxN low after complementary/polarity logic -> low-side input ON.

All three OC modes are changed coherently and latched with a COM event. Leaving full brake restores all three channels to PWM1 and generates another COM event.

The applied line-to-line voltage of this state is zero, so the observer voltage model remains consistent with an equal-duty zero vector.

### Why default OFF

Current VESC uses static all-low-side conduction for equal-duty zero vectors to reduce switching loss/dead-time distortion and increase low-speed braking torque. On this hoverboard PCB, continuous low-side gate drive and bootstrap/gate-driver behavior have not yet been physically validated. Therefore the backend is present but cannot be enabled by a normal VESC-6.00 configuration image.

`foc_short_ls_on_zero_duty` is newer than the pinned VESC-6.00 wire schema. Deserialization defaults it to false; serialization explicitly rejects `true` rather than pretending it was persisted.

## 4. Reset-latched safety semantics

PVD and BKIN are treated differently from ordinary recoverable motor faults. `motor_clear_fault()` refuses to clear them while `motor_hw_powerstage_fault_latched()` is true.

This prevents a dangerous state where VESC Tool reports no fault even though the hardware safety latch still blocks MOE.

## 5. Diagnostics

Terminal command:

`powerstage`

reports:

- reset-latched power-stage flags;
- current PVD-low state;
- TIM1/TIM8 BKIN compile-time enable state;
- runtime `foc_short_ls_on_zero_duty` state;
- whether the selected motor is currently in static full-brake mode.

The existing standard VESC packet/MCCONF/APPCONF layouts are unchanged.

## 6. Small compile-hygiene correction

Because `terminal.c` was touched for the new diagnostic, pre-existing variadic float calls were made explicit `double` promotions. This allows the file to pass the project's own `-Wdouble-promotion -Werror` policy in the host syntax check. Behavior is unchanged.

## Verification

All of the following pass together:

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
- `python tools/test_batch9_part3.py`
- `python tools/debug.py --self-test`

Part-3 regression additionally strict-compiles with host GCC and `-Wall -Wextra -Wshadow -Wdouble-promotion -Wformat=2 -Werror`:

- `src/motor/mc_interface.c`
- `src/motor/mcpwm_foc.c` (host-only DMA alias for the existing stub limitation)
- `src/terminal.c`

A strict host codec test confirms the VESC6 wire remains 481 bytes and refuses fake persistence of `foc_short_ls_on_zero_duty=true`.

## Hardware validation still mandatory

This environment has no `pio` or `arm-none-eabi-gcc`, so this is source/host verified, not an ARM/PlatformIO build claim.

Before full-power use:

1. Build with the real STM32F103 PlatformIO ARM toolchain.
2. Verify normal 3.3-V PVD state and intentionally test brownout using a current-limited logic supply before motor power.
3. Confirm a PVD event drops both TIM1/TIM8 MOE and remains reset-latched.
4. Leave BKIN disabled unless PB12/PA6 are proven to receive real gate/comparator fault signals with known polarity.
5. Leave `foc_short_ls_on_zero_duty` disabled initially.
6. If testing short-LS later, use a current-limited supply and oscilloscope on gate-driver inputs/phase nodes, then verify no bootstrap/gate-driver overheating before enabling under load.
