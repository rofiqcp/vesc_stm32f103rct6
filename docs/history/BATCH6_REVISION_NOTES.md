# Batch 6 — VESC Tool Ecosystem, Statistics, Odometer & ESTOP

Baseline: `vesc_stm32f103rct6_batch5_api_structure.zip`

Batch 6 deliberately does **not** modify the hard FOC algorithm, TIM1/TIM8 PWM/ADC timing, B3 observer/PLL, B4 UART DMA backend, or B4 watchdog.

## Implemented

### Terminal
- `COMM_TERMINAL_CMD` runs in the blocking command worker.
- `COMM_TERMINAL_CMD_SYNC` is supported inline.
- Reduced terminal commands: `help`, `faults`, `currents`, `foc`, `sensor`, `observer`, `timing`, `config`, `stats`, `reset_stats`, `odometer`, `stop`.
- Fault diagnostics do not consume the pending-fault latch.

### VESC plot / experiment packet helpers
Canonical public helpers were added:
- `commands_send_rotor_pos`
- `commands_send_experiment_samples`
- `commands_init_plot`
- `commands_plot_add_graph`
- `commands_plot_set_graph`
- `commands_send_plot_points`

Their packet IDs and payload format follow the VESC command protocol. They are helpers for firmware-side diagnostics; no fake background plot source is added.

### Temporary MCCONF
Implemented:
- `COMM_SET_MCCONF_TEMP`
- `COMM_SET_MCCONF_TEMP_SETUP`
- `COMM_GET_MCCONF_TEMP`

Supported temporary fields:
- current min/max scale
- min/max ERPM (or vehicle speed for `_SETUP`)
- min/max duty
- min/max watt
- optional min/max input current

The two local bridges are used as the divide-by-controller count. Physical CAN forwarding is intentionally not implemented. Temporary writes use the blocking worker because STM32F1 flash programming stalls code fetch when `store=true`.

### Persistent-vs-temporary isolation
The transactional record was migrated from version `0x0012` to `0x0013`.

- v0x0012 remains readable for lossless B5 migration.
- v0x0013 keeps the exact two 481-byte MCCONF images and one 493-byte APPCONF image, then appends two 64-bit odometers.
- Persistent component saves read the last committed config directly from memory-mapped flash (or compiled VESC6 defaults if no record exists), so a temporary runtime MCCONF cannot accidentally become permanent during an odometer or APPCONF save.
- This avoids a ~1.45 KiB persistent-shadow SRAM allocation.

### Statistics
`setup_stats` is now a real 100-Hz task-side accumulator per local motor:
- average/max mechanical speed
- average/max power magnitude
- average/max motor-current magnitude
- elapsed statistics time
- temperature statistics remain zero because this target intentionally has no validated NTC backend

Implemented:
- `COMM_GET_STATS`
- `COMM_RESET_STATS`

### Odometer
Implemented:
- explicit left/right odometer accessors
- `COMM_SET_ODOMETER`
- persistent odometer storage in config record v0x0013
- legacy v0x0012 migration with odometer initialized to zero
- automatic save every 1000 m of odometer delta
- automatic save is deferred while either motor is PWM-active or command-active

### Motor ESTOP input gate
Implemented:
- `COMM_MOTOR_ESTOP`
- `mc_interface_try_input_motor()`

ESTOP sets the shared ignore-input deadline and releases both motors. UART commands for duty/current/brake/RPM/position/handbrake/current-relative all pass through the per-motor input gate, so the next command cannot immediately defeat ESTOP.

### Application output-disable
Implemented `COMM_APP_DISABLE_OUTPUT` against the reduced B5 app manager. As in VESC, this is application-output state; it is not presented as a replacement for `COMM_MOTOR_ESTOP`.

## Files changed from B5
- `src/comm/commands.c`
- `src/comm/commands.h`
- `src/conf_general.c`
- `src/conf_general.h`
- `src/confgenerator.c`
- `src/motor/mc_interface.c`
- `src/motor/mc_interface.h`
- `src/terminal.c` (new)
- `src/terminal.h` (new)
- `tools/test_batch6.py` (new)

## Explicitly unchanged
The following are byte-identical to B5:
- `src/motor/mcpwm_foc.c`
- `src/motor/foc_math.c`
- `src/hwconf/hw.c`
- `src/timeout.c`
- `src/applications/app_uartcomm.c`
- `src/comm/packet.c`
- `src/comm/packet.h`

Thus Batch 6 does not change FOC ISR behavior, SVPWM, current control, field weakening, MTPA, observer/PLL, ADC sampling, USART3 DMA transport, watchdog, or packet framing.

## Verification
- VESC6 MCCONF/APPCONF ABI remains 481 / 493 bytes.
- All static port invariants pass.
- Batch 2, 3, 4, 5 and 6 regressions pass together.
- Typed VESC6 configuration/buffer host round-trip passes.
- Changed C translation units pass Clang `-Wall -Wextra -Werror` syntax checks with focused STM32F1/RTOS stubs.

This remains source/host verified, not yet physical-board verified. ARM PlatformIO / `arm-none-eabi-gcc` is not available in the audit environment.
