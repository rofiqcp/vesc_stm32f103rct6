#!/usr/bin/env python3
import argparse

from vesc_protocol import Command, VescClient
from vesc_protocol.buffer import pack_u32
from vesc_protocol.cli import add_connection_args, connection_values, print_mapping
from vesc_protocol.parsers import parse_values_selective


def main() -> None:
    parser = argparse.ArgumentParser(description="Baca field telemetri berdasarkan bit mask")
    add_connection_args(parser)
    parser.add_argument(
        "--mask", type=lambda value: int(value, 0), default=0x003FFFFF,
        help="bit 0..21; default semua field (0x003FFFFF)",
    )
    args = parser.parse_args()
    if not 0 <= args.mask <= 0xFFFFFFFF:
        raise SystemExit("--mask harus 0..0xFFFFFFFF")
    request = bytes((Command.GET_VALUES_SELECTIVE,)) + pack_u32(args.mask)
    with VescClient(**connection_values(args)) as client:
        data = parse_values_selective(client.request(request, can_id=args.can_id))
    print_mapping(data)


if __name__ == "__main__":
    main()
