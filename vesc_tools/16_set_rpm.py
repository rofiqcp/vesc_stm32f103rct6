#!/usr/bin/env python3
from vesc_protocol.buffer import pack_i32
from vesc_protocol.ids import Command
from vesc_protocol.setter_cli import between, run_motion_script


if __name__ == "__main__":
    run_motion_script(
        "Tes kecepatan electrical RPM",
        "erpm", "electrical RPM (-20000..20000)",
        lambda v: bytes((Command.SET_RPM,)) + pack_i32(round(v)),
        between(-20_000.0, 20_000.0),
    )
