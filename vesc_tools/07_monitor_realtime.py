#!/usr/bin/env python3
import argparse
import sys
import time

from vesc_protocol import Command, VescClient
from vesc_protocol.cli import add_connection_args, connection_values, json_line
from vesc_protocol.parsers import parse_values


def main() -> None:
    parser = argparse.ArgumentParser(description="Telemetri lengkap realtime, satu JSON per baris")
    add_connection_args(parser)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--count", type=int, default=0, help="0=terus sampai Ctrl+C")
    args = parser.parse_args()
    if not 0.2 <= args.hz <= 100:
        raise SystemExit("--hz harus 0.2..100")
    interval = 1.0 / args.hz
    index = 0
    with VescClient(**connection_values(args)) as client:
        try:
            while args.count == 0 or index < args.count:
                started = time.monotonic()
                data = parse_values(
                    client.request(bytes((Command.GET_VALUES,)), can_id=args.can_id)
                )
                data = {
                    "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
                    "target": "local" if args.can_id is None else args.can_id,
                    **data,
                }
                print(json_line(data), flush=True)
                index += 1
                delay = interval - (time.monotonic() - started)
                if delay > 0:
                    time.sleep(delay)
        except KeyboardInterrupt:
            print("\nMonitor dihentikan.", file=sys.stderr)


if __name__ == "__main__":
    main()
