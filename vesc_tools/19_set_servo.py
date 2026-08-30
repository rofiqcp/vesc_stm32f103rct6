#!/usr/bin/env python3
import argparse
import time

from vesc_protocol import VescClient
from vesc_protocol.buffer import scaled_i16
from vesc_protocol.cli import add_connection_args, connection_values
from vesc_protocol.ids import Command
from vesc_protocol.safety import require_arm


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tes output servo lalu kembali ke posisi netral")
    add_connection_args(parser)
    parser.add_argument("position", type=float, help="posisi target 0..1")
    parser.add_argument("--neutral", type=float, default=0.5)
    parser.add_argument("--duration", type=float, default=0.5)
    parser.add_argument("--arm", action="store_true")
    args = parser.parse_args()
    require_arm(args)
    if not 0 <= args.position <= 1 or not 0 <= args.neutral <= 1:
        raise SystemExit("position dan --neutral harus 0..1")
    if not 0.05 <= args.duration <= 10:
        raise SystemExit("--duration harus 0.05..10 detik")
    target = bytes((Command.SET_SERVO_POS,)) + scaled_i16(args.position, 1_000)
    neutral = bytes((Command.SET_SERVO_POS,)) + scaled_i16(args.neutral, 1_000)
    with VescClient(**connection_values(args)) as client:
        try:
            client.command(target, args.can_id)
            time.sleep(args.duration)
        finally:
            client.command(neutral, args.can_id)
    print(f"Selesai; servo dikembalikan ke {args.neutral:g}.")
