#!/usr/bin/env python3
import argparse

from vesc_protocol import VescClient
from vesc_protocol.cli import add_connection_args, connection_values
from vesc_protocol.safety import stop_motor


def main() -> None:
    parser = argparse.ArgumentParser(description="Kirim SET_DUTY=0 tiga kali")
    add_connection_args(parser)
    args = parser.parse_args()
    with VescClient(**connection_values(args)) as client:
        stop_motor(client, args.can_id)
    print("Perintah stop SET_DUTY=0 sudah dikirim tiga kali.")


if __name__ == "__main__":
    main()
