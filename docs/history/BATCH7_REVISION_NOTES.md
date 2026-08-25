# Batch 7 — Controller Semantics

Baseline: `vesc_stm32f103rct6_batch6_tool_ecosystem.zip`

Scope of this batch is deliberately limited to controller/config semantics. It does **not** change the proven TIM1/TIM8 PWM topology, ADC1/ADC2 dual-DMA sampling, USART3 DMA transport, IWDG, packet framing, flash record format, or VESC6 481/493-byte wire ABI.

## Implemented

### Speed PID
- Added VESC-style speed setpoint ramp using `s_pid_ramp_erpms_s`.
- Added `s_pid_min_erpm` release behavior.
- Activated VESC6 `s_pid_allow_braking`.
- Uses normalized P/I/D output with the VESC historical `1/20` gain scaling.
- Added PLL / FAST / FASTER runtime speed-source enum.
- VESC6 has no speed-source byte, so VESC6 deserialization defaults to PLL and the serializer refuses to silently persist FAST/FASTER.
- Corrected `m_invert_direction` handling so raw PLL/FAST feedback is converted to the VESC-facing coordinate before PID error is calculated, while current direction is multiplied only once at the output path.

### Position PID
- Activated `p_pid_kd_proc` derivative-on-process.
- Activated `p_pid_ang_div` and `p_pid_gain_dec_angle` near-target gain reduction.
- Activated `p_pid_offset` in the runtime controller and persistence API.
- Added derivative time accumulation for low-resolution/unchanged position samples.
- Added VESC-style proportional-headroom I-term windup limiting.
- Public compatibility position PID follows the same semantics as the active task-side controller.

### `cc_min_current` and current-off delay
- Activated VESC6 `cc_min_current`.
- `current_off_delay_s` is now an actual 1-kHz countdown state.
- PWM hold/release policy uses `max(cc_min_current, 0.001 A)` so the `cc_min_current = 0` case can still release the bridge.
- Sub-threshold current releases modulation without deleting the speed/position command, allowing closed-loop control to re-engage automatically when error grows.
- Active field weakening extends current-off delay by 1 second to avoid an abrupt modulation drop at the FW threshold.
- Added explicit per-motor current-off-delay API.

### Encoder/Hall compatibility helpers
- Public `foc_correct_encoder()` now uses the same 5% encoder/observer hysteresis policy as the hard ENCODER_AB path.
- Public `foc_correct_hall()` now uses Hall interpolation and the same `foc_sl_erpm_start` → `foc_sl_erpm` Hall-to-observer blend as the hard path.
- Hard Hall-to-observer transition remains integer/fixed-point in the 16-kHz phase path.

### VESC6 configuration ownership
The existing VESC6 bytes are now runtime-writable for:
- `s_pid_min_erpm`
- `s_pid_allow_braking`
- `s_pid_ramp_erpms_s`
- `p_pid_kd_proc`
- `p_pid_ang_div`
- `p_pid_gain_dec_angle`
- `p_pid_offset`
- `cc_min_current`

The VESC6 MCCONF remains exactly 481 bytes and APPCONF remains exactly 493 bytes.

## Intentionally not included
- No VESC7 wire migration.
- No additional observer algorithms (MXLEMMING / lambda / MXV).
- No changes to ADC/PWM/DMA timing.
- No UART/watchdog changes.
- No terminal/plot/persistence changes.
- `cc_startup_boost_duty`, `cc_gain`, and `cc_ramp_step_max` remain deferred because their controller backend is not implemented in this batch.

## Verification
- `tools/verify_vesc_port.py`
- Batch 2–6 regression suites
- `tools/test_batch7.py`
- `tools/test_batch7_roundtrip.c`
- Clang strict syntax check (`-Wall -Wextra -Werror`) for:
  - `src/confgenerator.c`
  - `src/motor/mc_interface.c`
  - `src/motor/foc_math.c`
  - `src/motor/mcpwm_foc.c`

All above checks pass in the host verification environment.

Hardware validation on the real STM32F103 hoverboard board is still required before this firmware can be called hardware-proven.
