#!/usr/bin/env python3
import argparse

from vesc_protocol import Command, VescClient
from vesc_protocol.cli import add_connection_args, connection_values


def main() -> None:
    parser = argparse.ArgumentParser(description="Kirim COMM_ALIVE satu kali")
    add_connection_args(parser)
    args = parser.parse_args()
    with VescClient(**connection_values(args)) as client:
        client.command(bytes((Command.ALIVE,)), args.can_id)
    print("COMM_ALIVE terkirim (command ini memang tidak membalas).")


if __name__ == "__main__":
    main()
