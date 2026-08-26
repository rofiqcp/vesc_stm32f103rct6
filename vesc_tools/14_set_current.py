#!/usr/bin/env python3
from vesc_protocol.buffer import scaled_i32
from vesc_protocol.ids import Command
from vesc_protocol.setter_cli import between, run_motion_script


if __name__ == "__main__":
    run_motion_script(
        "Tes arus motor (tetap dibatasi konfigurasi VESC)",
        "amps", "ampere (-10..10)",
        lambda v: bytes((Command.SET_CURRENT,)) + scaled_i32(v, 1_000),
        between(-10.0, 10.0),
    )
