#!/usr/bin/env python3
"""Static invariants for the STM32F103 hoverboard VESC port.

This is intentionally a source-level verifier. It does not replace an ARM build
or bench test; it protects the architectural contracts that are easy to break
while porting upstream VESC algorithms onto the proven hoverboard timing layer.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"


def fail(msg: str) -> None:
    print(f"FAIL: {msg}")
    raise SystemExit(1)


def passed(msg: str) -> None:
    print(f"PASS: {msg}")


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8")


def macro_int(text: str, name: str) -> int:
    m = re.search(rf"^\s*#\s*define\s+{re.escape(name)}\s+([0-9]+)(?:U|UL|L)?\b", text, re.M)
    if not m:
        fail(f"missing integer macro {name}")
    return int(m.group(1))


def strip_c_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//.*?$", "", text, flags=re.M)


def extract_function(text: str, signature_fragment: str) -> str:
    pos = text.find(signature_fragment)
    if pos < 0:
        fail(f"cannot find function signature containing {signature_fragment!r}")
    brace = text.find("{", pos)
    if brace < 0:
        fail(f"cannot find opening brace for {signature_fragment!r}")
    depth = 0
    for i in range(brace, len(text)):
        ch = text[i]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return text[pos : i + 1]
    fail(f"unterminated function containing {signature_fragment!r}")
    return ""


def verify_wire_abi() -> None:
    h = read("src/confgenerator.h")
    expected = {
        "VESC6_MCCONF_WIRE_SIZE": 481,
        "VESC6_APPCONF_WIRE_SIZE": 493,
        "VESC6_MCCONF_SIGNATURE": 776184161,
        "VESC6_APPCONF_SIGNATURE": 486554156,
    }
    got = {name: macro_int(h, name) for name in expected}
    if got != expected:
        fail(f"VESC 6.00 wire ABI changed: expected {expected}, got {got}")
    passed("VESC 6.00 wire sizes/signatures are locked (481/493)")


def verify_fw_handshake() -> None:
    c = read("src/comm/commands.c")
    fn = extract_function(c, "static void reply_fw_version")
    body = strip_c_comments(fn)
    if not re.search(r"p\s*\[\s*i\+\+\s*\]\s*=\s*6U?\s*;", body):
        fail("COMM_FW_VERSION no longer advertises major version 6")
    if not re.search(r"p\s*\[\s*i\+\+\s*\]\s*=\s*0U?\s*;", body):
        fail("COMM_FW_VERSION no longer advertises minor version 0")
    passed("COMM_FW_VERSION remains aligned with the VESC 6.00 wire schema")


def verify_fixed_point_isr() -> None:
    forbidden = [
        r"\bfloat\b", r"\bdouble\b", r"\bsqrtf?\s*\(", r"\bsinf?\s*\(",
        r"\bcosf?\s*\(", r"\batan2f?\s*\(", r"\bl?lrintf?\s*\(",
        r"\bfabsf?\s*\(", r"\bfminf?\s*\(", r"\bfmaxf?\s*\(",
        r"\bpowf?\s*\(", r"\bexpf?\s*\(", r"\blogf?\s*\(",
    ]

    # Check the hard ADC entry, the per-motor control loop, and the important
    # direct callees used for phase, observer, transforms/SVM, sampling and PWM.
    # Slow/task-side helpers are intentionally allowed to use float.
    checks = [
        ("src/motor/mcpwm_foc.c", "void mcpwm_foc_adc_words_isr", "ADC/DMA ISR entry"),
        ("src/motor/mcpwm_foc.c", "static void foc_one_motor_isr", "per-motor FOC ISR"),
        ("src/motor/mcpwm_foc.c", "uint16_t motor_sensor_electrical_phase_u16", "fast phase selector"),
        ("src/motor/mcpwm_foc.c", "static inline void detect_inductance_capture_isr", "inductance capture ISR"),
        ("src/motor/foc_math.c", "void foc_observer_update_fixed", "fixed observer"),
        ("src/motor/foc_math.c", "void foc_pll_run_fixed", "fixed PLL"),
        ("src/motor/foc_math.c", "void foc_fast_sincos_u16_q15", "fixed sin/cos"),
        ("src/motor/foc_math.c", "void foc_svm_q15", "fixed SVM"),
        ("src/debug_sample.c", "static void sample_fill", "ISR sample packer"),
        ("src/debug_sample.c", "void debug_sample_capture_isr", "debug sample ISR"),
        ("src/hwconf/hw.c", "void motor_hw_set_pwm_q15", "fixed PWM writer"),
    ]
    for rel, sig, label in checks:
        fn = strip_c_comments(extract_function(read(rel), sig))
        hits = [pat for pat in forbidden if re.search(pat, fn)]
        if hits:
            fail(f"{label} contains floating-point constructs: " + ", ".join(hits))
    passed("hard ADC/FOC call path and direct fast helpers contain no floating-point constructs")


def verify_forbidden_physical_modules() -> None:
    # COMM_FORWARD_CAN is deliberately a protocol selector for local motor 2;
    # this check is about physical module source files only.
    forbidden = [
        re.compile(r"^(can|comm_can)(?:_|\.).*\.c$", re.I),
        re.compile(r"^imu(?:_|\.).*\.c$", re.I),
        re.compile(r"^bms(?:_|\.).*\.c$", re.I),
        re.compile(r"^bm_if(?:_|\.).*\.c$", re.I),
        re.compile(r"^nrf(?:_|\.).*\.c$", re.I),
        re.compile(r"^ledpwm(?:_|\.).*\.c$", re.I),
        re.compile(r"^(comm_usb|usbcomm|comusb)(?:_|\.).*\.c$", re.I),
        re.compile(r"^lispif(?:_|\.).*\.c$", re.I),
        re.compile(r"^lzo(?:_|\.).*\.c$", re.I),
        re.compile(r"^qml(?:_|\.).*\.c$", re.I),
        re.compile(r"^ntc(?:_|\.).*\.c$", re.I),
    ]
    bad: list[str] = []
    for p in SRC.rglob("*.c"):
        if any(rx.match(p.name) for rx in forbidden):
            bad.append(str(p.relative_to(ROOT)))
    if bad:
        fail("forbidden physical modules present: " + ", ".join(sorted(bad)))
    passed("excluded CAN/IMU/BMS/NRF/USB/Lisp/LZO/QML/LEDPWM/NTC source modules are absent")


def verify_left_encoder_spec() -> None:
    app = read("src/applications/appconf_default.h")
    pp = macro_int(app, "LEFT_POLE_PAIRS")
    ppr = macro_int(app, "LEFT_ENCODER_PPR")
    if pp != 4 or ppr != 1024:
        fail(f"LEFT motor/encoder default changed: pole_pairs={pp}, PPR={ppr}")
    if not re.search(r"LEFT_ENCODER_CPR\s+\(LEFT_ENCODER_PPR\s*\*\s*4U?\)", app):
        fail("LEFT_ENCODER_CPR is not quadrature x4 of LEFT_ENCODER_PPR")
    passed("LEFT encoder defaults are 4 pole-pairs, 1024 PPR, 4096 quadrature counts/rev")


def verify_sensor_policy() -> None:
    defaults = read("src/motor/mcconf_default.h")
    app = read("src/applications/appconf_default.h")
    conf = read("src/confgenerator.c")
    direct = read("src/motor/mc_interface.c")

    required = [
        (defaults, r"MCCONF_FOC_SENSOR_LEFT_DEFAULT\s+FOC_SENSOR_MODE_ENCODER_AB", "left default AB encoder"),
        (defaults, r"MCCONF_FOC_SENSOR_RIGHT_DEFAULT\s+FOC_SENSOR_MODE_HALL", "right default Hall"),
        (app, r"LEFT_SENSOR_BOOT_MODE\s+SENSOR_MODE_ENCODER", "left boot encoder"),
        (app, r"RIGHT_SENSOR_BOOT_MODE\s+SENSOR_MODE_HALL", "right boot Hall"),
        (conf, r"id\s*==\s*MOTOR_RIGHT.*?VESC_FOC_SENSOR_HALL", "wire config right-Hall enforcement"),
        (direct, r"m->id\s*==\s*MOTOR_RIGHT.*?FOC_SENSOR_MODE_HALL", "direct API right-Hall enforcement"),
    ]
    for text, pattern, label in required:
        if not re.search(pattern, text, flags=re.S):
            fail(f"sensor policy invariant missing: {label}")
    passed("physical sensor policy remains LEFT Hall/ABI and RIGHT Hall-only; sensorless is observer mode")


def verify_local_motor2_forwarding() -> None:
    c = read("src/comm/commands.c")
    if "COMM_FORWARD_CAN" not in c:
        fail("COMM_FORWARD_CAN local motor-2 protocol forwarding was removed")
    # No physical CAN implementation should be referenced by ordinary names.
    if re.search(r"\bcomm_can_(send|transmit|forward)\b", c):
        fail("commands.c references a physical CAN transmission backend")
    passed("COMM_FORWARD_CAN is retained as local motor-2 protocol routing without CAN PHY")
    if "case COMM_PING_CAN:" not in c or "{COMM_PING_CAN, VESC_LOCAL_MOTOR2_FORWARD_ID}" not in c:
        fail("VESC Tool COMM_PING_CAN discovery for local motor-2 ID was removed")
    if "target == VESC_LOCAL_MOTOR2_FORWARD_ID" not in c:
        fail("VESC Tool forwarded motor-2 command routing no longer matches advertised ID")
    passed("VESC Tool device scan advertises exactly the locally forwarded motor-2 ID")


def verify_hoverboard_timing_contract() -> None:
    hw = read("src/hwconf/hw.c")
    required_tokens = ["TIM1", "TIM8", "TIM_SLAVEMODE_GATED", "ADC_DUALMODE_REGSIMULT", "DMA1_Channel1_IRQn"]
    missing = [t for t in required_tokens if t not in hw]
    if missing:
        fail("hoverboard timing contract missing tokens: " + ", ".join(missing))
    passed("TIM1/TIM8 + dual-ADC/DMA hoverboard timing architecture remains present")


def main() -> None:
    verify_wire_abi()
    verify_fw_handshake()
    verify_fixed_point_isr()
    verify_forbidden_physical_modules()
    verify_left_encoder_spec()
    verify_sensor_policy()
    verify_local_motor2_forwarding()
    verify_hoverboard_timing_contract()
    print("ALL STATIC PORT INVARIANTS: PASS")


if __name__ == "__main__":
    main()
