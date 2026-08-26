#!/usr/bin/env python3
import argparse

from vesc_protocol import Command, VescClient
from vesc_protocol.buffer import pack_u16
from vesc_protocol.cli import add_connection_args, connection_values, print_mapping
from vesc_protocol.parsers import parse_imu


def main() -> None:
    parser = argparse.ArgumentParser(description="Baca seluruh field IMU yang tersedia")
    add_connection_args(parser)
    parser.add_argument("--mask", type=lambda v: int(v, 0), default=0xFFFF)
    args = parser.parse_args()
    if not 0 <= args.mask <= 0xFFFF:
        raise SystemExit("--mask harus 0..0xFFFF")
    request = bytes((Command.GET_IMU_DATA,)) + pack_u16(args.mask)
    with VescClient(**connection_values(args)) as client:
        data = parse_imu(client.request(request, can_id=args.can_id))
    print_mapping(data)


if __name__ == "__main__":
    main()
