#!/usr/bin/env python3
from vesc_protocol.buffer import scaled_i32
from vesc_protocol.ids import Command
from vesc_protocol.setter_cli import between, run_motion_script


if __name__ == "__main__":
    run_motion_script(
        "Tes arus relatif terhadap limit motor",
        "relative", "rasio arus (-0.25..0.25)",
        lambda v: bytes((Command.SET_CURRENT_REL,)) + scaled_i32(v, 100_000),
        between(-0.25, 0.25),
    )
