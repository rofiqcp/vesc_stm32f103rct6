#!/usr/bin/env python3
"""Source-level regression checks for Batch 2.

These checks deliberately avoid requiring an ARM toolchain. They verify the
fixed-point dead-time constants/math contract, VESC duty down-ramp structure,
brake zero-cross state, and that the observer voltage is reconstructed from the
actual SVM duties before dead-time compensation.
"""
from pathlib import Path
import math, re, sys

ROOT = Path(__file__).resolve().parents[1]
app = (ROOT / "src/applications/appconf_default.h").read_text()
foc = (ROOT / "src/motor/foc_math.c").read_text()
mc = (ROOT / "src/motor/mc_interface.c").read_text()
isr = (ROOT / "src/motor/mcpwm_foc.c").read_text()
data = (ROOT / "src/datatypes.h").read_text()

def req(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print("PASS:", msg)

def macro_integer(name):
    m = re.search(rf"^\s*#define\s+{re.escape(name)}\s+([0-9]+)", app, re.M)
    if not m:
        raise AssertionError(f"macro {name} not found")
    return int(m.group(1))

# Hardware contract: 750 ns = 0.75 us, and f_zv=2*16 kHz -> Q15 ~786.
pwm_hz = float(macro_integer("PWM_FREQUENCY_HZ"))
dt_us = float(macro_integer("PWM_DEADTIME_NS")) / 1000.0
dt_q15 = round(dt_us * 1e-6 * (2.0 * pwm_hz) * 32768.0)
req(dt_q15 == 786, "dead-time default precompute is Q15=786 (0.75us x 32kHz)")
req("m->foc_dt_us * 1.0e-6f * (float)VESC_FOC_F_ZV_HZ" in foc,
    "dead-time coefficient is precomputed task-side from foc_dt_us*foc_f_zv")
dt_body = re.search(r"void foc_deadtime_compensate_voltage_q15\(.*?\n\}", foc, re.S)
req(dt_body is not None and "float" not in dt_body.group(0),
    "dead-time compensation helper uses integer/fixed-point arithmetic")

# Observer must model what PWM actually applied, including Batch-1 SVM scaling.
pos_svm = isr.find("foc_svm_q15(v_alpha, v_beta")
pos_recon = isr.find("foc_pwm_applied_voltage_q15(du, dv, dw")
pos_dt = isr.find("foc_deadtime_applied_voltage_q15")
pos_store = isr.find("m->observer_v_alpha_q15_prev = obs_v_alpha", pos_svm)
req(pos_svm >= 0 and pos_recon > pos_svm and pos_store > pos_recon,
    "observer voltage is reconstructed from final SVM duties before publication")
# The first occurrence of deadtime helper is its definition; require the call after reconstruction.
pos_dt_call = isr.find("foc_deadtime_applied_voltage_q15(m, vbus_q15", pos_recon)
req(pos_dt_call > pos_recon and pos_dt_call < pos_store,
    "dead-time correction is applied to reconstructed applied voltage")

# VESC duty semantics: PI only on down-ramp; normal/ramp-up limits modulation and requests allowed current.
req("fabsf(duty_set) < (duty_abs - 0.01f)" in mc,
    "duty PI is gated to VESC-style down-ramp threshold")
req("m->duty_limit_now = fabsf(duty_set);" in mc,
    "normal/ramp-up duty mode lowers dynamic modulation ceiling to requested duty")
req("return (duty_set > 0.0f ? 1.0f : -1.0f) * current_max_for_duty;" in mc,
    "normal/ramp-up duty mode requests the allowed current limit")
req("if (fabsf(out) < min_duty) out = 0.0f;" in isr,
    "sub-minimum duty commands release instead of being promoted")

# Brake guard state: direction/Vq/near-zero transition + at least 10 hard FOC samples equivalent.
req(all(x in data for x in ["brake_speed_before_q16", "brake_vq_before_q15", "brake_zero_hold_ticks", "brake_zero_active"]),
    "brake zero-cross state is stored per motor")
req("speed_sign_now != speed_sign_prev" in mc and "vq_sign_now != vq_sign_prev" in mc,
    "brake guard detects speed and Vq sign changes")
req("fabsf(m->duty_now) < 0.001f" in mc,
    "brake guard detects the near-zero modulation region")
req("const float brake_target = fminf(fabsf(m->brake_current_a), brake_lim);" in mc and
    "brake_zero_guard_1khz(m, brake_target)" in mc,
    "brake guard threshold is clamped to the reachable runtime brake-current limit")
req("BR_ZERO_MIN_HOLD_TICKS = 1U" in mc and "FOC_ISR_EVENT_HZ" in app,
    "brake guard holds at least one 1kHz tick (>10 hard FOC cycles at 16kHz)")
req("m->force_zero_modulation" in isr,
    "hard FOC path can emit centered zero modulation during brake transition")

print("ALL BATCH 2 SOURCE REGRESSIONS: PASS")
