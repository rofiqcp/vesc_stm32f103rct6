#!/usr/bin/env python3
import argparse

from vesc_protocol import Command, VescClient
from vesc_protocol.cli import add_connection_args, connection_values, print_mapping
from vesc_protocol.parsers import parse_firmware


def main() -> None:
    parser = argparse.ArgumentParser(description="Tes koneksi dan identitas VESC lokal")
    add_connection_args(parser, can=False)
    args = parser.parse_args()
    with VescClient(**connection_values(args)) as client:
        result = parse_firmware(client.request(bytes((Command.FW_VERSION,))))
    print("Koneksi VESC berhasil.")
    print_mapping(result)


if __name__ == "__main__":
    main()
