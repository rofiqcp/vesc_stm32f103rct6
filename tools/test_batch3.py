#!/usr/bin/env python3
"""Source/numeric regression checks for Batch 3 sensor/observer work.

No ARM toolchain is required. The checks protect the fixed-point PLL contract,
ENCODER_AB 5% hysteresis, VESC observer-gain semantics, saliency/saturation
parameter flow, and the intentional restriction to the validated Ortega backend.
"""
from pathlib import Path
import math, re

ROOT = Path(__file__).resolve().parents[1]
app = (ROOT / "src/applications/appconf_default.h").read_text()
data = (ROOT / "src/datatypes.h").read_text()
foc = (ROOT / "src/motor/foc_math.c").read_text()
isr = (ROOT / "src/motor/mcpwm_foc.c").read_text()
mc = (ROOT / "src/motor/mc_interface.c").read_text()
conf = (ROOT / "src/confgenerator.c").read_text()

def req(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print("PASS:", msg)

def macro_number(name):
    m = re.search(rf"^\s*#define\s+{re.escape(name)}\s+\(?([0-9.]+)", app, re.M)
    if not m:
        raise AssertionError(f"macro {name} not found")
    return float(m.group(1))

def function_body(text, signature):
    p = text.find(signature)
    if p < 0:
        raise AssertionError(f"function {signature} not found")
    b = text.find("{", p)
    depth = 0
    for i in range(b, len(text)):
        if text[i] == "{": depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[p:i+1]
    raise AssertionError(f"unterminated {signature}")

# Defaults now follow VESC observer semantics: gain_slow is a fraction.
req("MCCONF_FOC_OBSERVER_GAIN_DEFAULT        900000.0f" in app,
    "observer gain default is 900000")
req("MCCONF_FOC_OBSERVER_GAIN_SLOW_DEFAULT   0.05f" in app,
    "observer slow gain default is fractional 0.05")
req(all(name in data for name in ["FOC_OBSERVER_ORTEGA_ORIGINAL", "FOC_OBSERVER_MXLEMMING",
                                  "FOC_OBSERVER_ORTEGA_LAMBDA_COMP", "FOC_OBSERVER_MXLEMMING_LAMBDA_COMP",
                                  "FOC_OBSERVER_MXV", "FOC_OBSERVER_MXV_LAMBDA_COMP",
                                  "FOC_OBSERVER_MXV_LAMBDA_COMP_LIN"]),
    "observer enum keeps current VESC canonical ordering/names")

# Fixed-point PLL coefficient derivation at the actual one-frame-per-PWM FOC rate.
pwm_hz = int(macro_number("PWM_FREQUENCY_HZ"))
fs = pwm_hz
kp = 2000.0
ki = 30000.0
kp_q16 = round(kp / fs * 65536.0)
ki_q16 = round(ki * 60.0 / fs * 65536.0)
req(kp_q16 == 8192, "default PLL Kp*dt coefficient is Q16=8192 at 16kHz")
req(ki_q16 == 7372800, "default PLL Ki*dt*60 coefficient is Q16=7372800 at 16kHz")
req("m->pll_kp_dt_q16" in foc and "m->pll_ki_dt60_q16" in foc,
    "PLL gains are precomputed task-side")
pll_body = function_body(foc, "void foc_pll_run_fixed")
forbidden = ["float", "double", "sqrt", "sin(", "cos(", "atan", "lrint", "fabs"]
req(not any(tok in pll_body for tok in forbidden),
    "hard fixed-point PLL contains no floating-point/math-library operations")

# Numeric PLL simulation using exactly the integer equations implemented in C.
pll_phase = 0
pll_speed_q16 = 0
src_phase_f = 0.0
target_erpm = 3000.0
for _ in range(fs // 2):
    src_phase_f = (src_phase_f + target_erpm * 65536.0 / (60.0 * fs)) % 65536.0
    src = int(src_phase_f) & 0xFFFF
    err = ((src - pll_phase + 32768) & 0xFFFF) - 32768
    speed_adv = int(pll_speed_q16 / (60 * fs))
    prop_adv = (err * kp_q16) >> 16
    pll_phase = (pll_phase + speed_adv + prop_adv) & 0xFFFF
    pll_speed_q16 += (err * ki_q16) >> 16
    cap = int(30000 * 65536)
    pll_speed_q16 = max(-cap, min(cap, pll_speed_q16))
pll_erpm = pll_speed_q16 / 65536.0
req(abs(pll_erpm - target_erpm) < 15.0,
    "fixed-point PLL converges to a 3000 ERPM synthetic phase ramp")

# Current VESC ENCODER_AB-style source selection: 5% hysteresis and fast speed.
req("const int32_t h = sw / 20; /* 5 percent */" in isr,
    "ENCODER_AB source selector uses 5% hysteresis")
req("m->speed_est_fast_erpm_q16" in function_body(isr, "static inline void encoder_ab_update_source_isr"),
    "ENCODER_AB hysteresis uses the fast corrected phase-speed estimate")
req("if (!m->using_encoder) foc_encoder_ab_sync_from_observer(m);" in mc,
    "task-side encoder logic only rebases ABI while observer source is active")
req("abs_i32_sat(m->speed_est_fast_erpm_q16) >= m->foc_sl_erpm_q16" in foc,
    "initial ENCODER_AB observer-to-counter sync uses fast speed threshold")

# Observer correction remains fixed-point in the hard ISR.
obs_body = function_body(foc, "void foc_observer_update_fixed")
req("observer_gamma_coeff_q30" in obs_body and "flux_mag_sq_q30" in obs_body,
    "fixed observer applies Ortega magnitude correction using Q30 coefficients")
req("if (err > 0) err = 0;" in obs_body,
    "Ortega correction clamps positive lambda error to zero")
req("grow_q15 = 36045" in obs_body and "observer_flux_target_q30 / 2" in obs_body,
    "observer state has the VESC-style low-flux anti-collapse safeguard")

# Slow observer adaptation: Ld-Lq saliency and SAT_COMP_FACTOR reduce L/lambda.
slow_body = function_body(foc, "void foc_observer_update_1khz")
req("m->foc_motor_ld_lq_diff * (iq * iq / i2)" in slow_body,
    "observer effective inductance includes Ld-Lq saliency correction")
req("case SAT_COMP_FACTOR:" in slow_body and
    "l_eff *= (1.0f - comp);" in slow_body and "flux_eff *= (1.0f - comp);" in slow_body,
    "SAT_COMP_FACTOR reduces both effective L and flux linkage")
req("fabsf(m->duty_now) * vbus / 40.0f" in slow_body and
    "foc_observer_gain_slow" in slow_body,
    "observer gamma follows duty/Vbus with configured slow-gain floor")

# Configuration ownership now exposes only implemented observer controls.
req("if (byte_in_range(i, 153U, 8U)) return true" in conf,
    "VESC6 PLL Kp/Ki wire fields are runtime-writable")
req("if (byte_in_range(i, 177U, 8U)) return true" in conf,
    "VESC6 observer gain/slow-gain wire fields are runtime-writable")
req("VESC6_MC_OFF_FOC_SAT_COMP_MODE" in conf and "SAT_COMP_LAMBDA_AND_FACTOR" in conf,
    "VESC6 saturation-compensation configuration has a real backend")
req("observer_type > FOC_OBSERVER_MXV_LAMBDA_COMP_LIN" in conf and
    "VESC6_MC_OFF_FOC_OBSERVER_TYPE) return true" in conf,
    "all implemented VESC observer types are range-checked and writable")
req("legacy_gain - 1000.0f" in conf and "legacy_slow - 1000.0f" in conf,
    "Batch-2 observer placeholder defaults are migrated on flash import")

# Right motor Hall-only policy must remain present after sensor work.
req("m->id == MOTOR_RIGHT" in mc and "FOC_SENSOR_MODE_HALL" in mc,
    "Batch 3 retains right-motor Hall-only policy")

print("ALL BATCH 3 SENSOR/OBSERVER REGRESSIONS: PASS")
