#!/usr/bin/env python3
from vesc_protocol.buffer import scaled_i32
from vesc_protocol.ids import Command
from vesc_protocol.setter_cli import between, run_motion_script


if __name__ == "__main__":
    run_motion_script(
        "Tes target posisi PID",
        "degrees", "derajat (-2147..2147)",
        lambda v: bytes((Command.SET_POS,)) + scaled_i32(v, 1_000_000),
        between(-2147.0, 2147.0),
    )
