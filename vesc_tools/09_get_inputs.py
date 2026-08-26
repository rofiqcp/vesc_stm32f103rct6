#!/usr/bin/env python3
import argparse

from vesc_protocol import Command, VescClient
from vesc_protocol.cli import add_connection_args, connection_values, print_mapping
from vesc_protocol.parsers import parse_adc, parse_chuk, parse_ppm


SOURCES = {
    "ppm": (Command.GET_DECODED_PPM, parse_ppm),
    "adc": (Command.GET_DECODED_ADC, parse_adc),
    "chuk": (Command.GET_DECODED_CHUK, parse_chuk),
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Baca input PPM, ADC, dan CHUK")
    add_connection_args(parser)
    parser.add_argument("--source", choices=("all", *SOURCES), default="all")
    args = parser.parse_args()
    names = SOURCES if args.source == "all" else {args.source: SOURCES[args.source]}
    with VescClient(**connection_values(args)) as client:
        for name, (command, decode) in names.items():
            print(f"[{name.upper()}]")
            try:
                result = decode(client.request(bytes((command,)), can_id=args.can_id))
                print_mapping(result)
            except Exception as exc:
                print(f"gagal/tidak didukung: {exc}")


if __name__ == "__main__":
    main()
