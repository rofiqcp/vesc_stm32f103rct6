#!/usr/bin/env python3
from vesc_protocol.buffer import scaled_i32
from vesc_protocol.ids import Command
from vesc_protocol.setter_cli import between, run_motion_script


if __name__ == "__main__":
    run_motion_script(
        "Tes duty cycle (dibatasi waktu dan otomatis stop)",
        "duty", "rasio duty (-0.25..0.25)",
        lambda v: bytes((Command.SET_DUTY,)) + scaled_i32(v, 100_000),
        between(-0.25, 0.25),
    )
