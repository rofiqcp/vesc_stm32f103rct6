#!/usr/bin/env python3
import argparse
import time

from vesc_protocol import Command, VescClient
from vesc_protocol.cli import add_connection_args, connection_values
from vesc_protocol.safety import require_arm
from vesc_protocol.parsers import parse_rotor_position


def main() -> None:
    parser = argparse.ArgumentParser(description="Uji COMM_SET_DETECT/display mode")
    add_connection_args(parser)
    parser.add_argument("mode", type=int, help="nilai mode byte 0..255")
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--arm", action="store_true")
    args = parser.parse_args()
    require_arm(args)
    if not 0 <= args.mode <= 255:
        raise SystemExit("mode harus 0..255")
    if not 0.05 <= args.duration <= 10:
        raise SystemExit("duration harus 0.05..10 detik")
    with VescClient(**connection_values(args)) as client:
        try:
            client.clear_input()
            client.command(bytes((Command.SET_DETECT, args.mode)), args.can_id)
            deadline = time.monotonic() + args.duration
            samples = 0
            while time.monotonic() < deadline:
                reply = client.receive(deadline - time.monotonic())
                if reply is None:
                    break
                if reply and reply[0] == Command.ROTOR_POSITION:
                    print(f"rotor_position={parse_rotor_position(reply):.5f}")
                    samples += 1
        finally:
            client.command(bytes((Command.SET_DETECT, 0)), args.can_id)
    print(f"Mode dikembalikan ke 0; {samples} sampel posisi diterima.")


if __name__ == "__main__":
    main()
