#!/usr/bin/env python3
import argparse

from vesc_protocol import Command, VescClient
from vesc_protocol.cli import add_connection_args, connection_values, print_mapping
from vesc_protocol.parsers import parse_values


def main() -> None:
    parser = argparse.ArgumentParser(description="Baca seluruh COMM_GET_VALUES sekali")
    add_connection_args(parser)
    args = parser.parse_args()
    with VescClient(**connection_values(args)) as client:
        data = parse_values(client.request(bytes((Command.GET_VALUES,)), can_id=args.can_id))
    print_mapping(data)


if __name__ == "__main__":
    main()
