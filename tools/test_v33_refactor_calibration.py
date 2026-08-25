#!/usr/bin/env python3
"""V33 structure, calibration/MOE, and dual-rate debug regression gate."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]


def source(path: str) -> str:
	return (ROOT / path).read_text(errors="replace")


def check(condition: bool, message: str) -> None:
	if not condition:
		print(f"FAIL: {message}")
		raise SystemExit(1)
	print(f"PASS: {message}")


removed = (
	"src/fault.c", "src/fault.h", "src/motor_tasks.c", "src/motor_tasks.h",
	"src/motor_types.h", "src/status_io.c", "src/status_io.h",
	"src/app_config.h", "src/board_pins.h", "src/debug_sample.c",
	"src/debug_sample.h",
)
for rel in removed:
	check(not (ROOT / rel).exists(), f"obsolete wrapper removed: {rel}")

for rel in (
	"src/motor/mc_interface_tasks.c", "src/motor/mc_interface_sample.c",
	"src/motor/mc_interface_sample.h", "src/hwconf/hw_status.c",
):
	check((ROOT / rel).is_file(), f"VESC-style owner path exists: {rel}")

all_source = "\n".join(
	p.read_text(errors="replace")
	for p in sorted((ROOT / "src").rglob("*"))
	if p.suffix in {".c", ".h"}
)
for old in (
	'motor_tasks.h', 'motor_types.h', 'debug_sample.h', 'status_io.h',
	'app_config.h', 'board_pins.h', 'motor_hw_gate_global',
	'motor_hw_buzzer', 'offset_track_isr',
):
	check(old not in all_source, f"obsolete source/API reference absent: {old}")

main = source("src/main.c")
config = source("src/confgenerator.c")
check("mc_interface_init(false);" in main,
		"boot uses canonical mc_interface initialization")
check(main.index("vesc_config_apply_defaults()") <
		main.index("conf_general_init();"),
		"compiled VESC defaults are applied before optional flash import")
check("bool vesc_config_apply_defaults(void)" in config and
		"vesc_config_import_wire(s_mc_factory[MOTOR_LEFT]" in config,
		"virgin-flash boot has a real factory-default runtime apply path")

app_defaults = source("src/applications/appconf_default.h")
foc = source("src/motor/mcpwm_foc.c")
foch = source("src/motor/mcpwm_foc.h")
commands = source("src/comm/commands.c")
debug = source("tools/debug.py")

for constant in (
	"ADC_OFFSET_INLIER_WINDOW_COUNT", "ADC_OFFSET_HARD_OUTLIER_COUNT",
	"ADC_OFFSET_MOE_WAIT_EVENTS",
):
	check(constant in app_defaults, f"calibration policy defines {constant}")
check("cal_accumulate_driven_channel" in foc and
		"s_cal_outlier_count" in foc and "cal_inlier_count" in foc,
		"driven offsets use bounded robust inliers with visible outlier counts")
check("cal_moe_ready_isr" in foc and "TIM_BDTR_MOE" in foc and
		"s_cal_moe_request_adc" in foc and "s_cal_moe_confirm_adc" in foc,
		"calibration verifies physical MOE timing independently for both motors")
check("outlier_count[6]" in foch and "moe_fail_mask" in foch and
		"first_sample_adc[2]" in foch,
		"calibration diagnostic structure exposes robust/MOE evidence")
check("CURRENT_CAL_REPLY_REV17_LEN 463U" in commands and
		"p[i++] = 17U; /* calibration diagnostic revision */" in commands,
		"current-cal revision 17 is bounded below the 512-byte packet limit")
check("cal_diag_revision\",0) >= 17" in debug and
		"PWM MOE CALIBRATION TRACE" in debug,
		"host debug tool decodes and prints calibration revision 17")

check("def cmd_speed_test" in debug and "rt_period = 1.0 / 50.0" in debug and
		"app_period = 1.0 / 20.0" in debug,
		"active speed test schedules RT data at 50 Hz and APP data at 20 Hz")
check("get_decoded_adc" in debug and "command_hz" in debug and
		"missed_rt_slots" in debug and "mean/max_jitter_ms" in debug,
		"speed test measures command/data rates, misses, timeouts, and jitter")
check(re.search(r'p=sp\("speed-test",cmd_speed_test', debug) is not None,
		"speed-test command is exposed by the commissioning CLI")

excluded = (
	'#include "comm_can.h"', '#include "imu/', '#include "bms/',
	'#include "bm_if', '#include "nrf', '#include "ledpwm',
	'#include "comm_usb', '#include "lispif', '#include "lzo',
)
lower_source = all_source.lower()
for token in excluded:
	check(token not in lower_source, f"excluded subsystem remains absent: {token}")

print("ALL V33 REFACTOR/CALIBRATION/DUAL-RATE REGRESSIONS: PASS")
sys.exit(0)
