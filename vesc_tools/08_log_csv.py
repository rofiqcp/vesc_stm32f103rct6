#!/usr/bin/env python3
import argparse
import csv
import time

from vesc_protocol import Command, VescClient
from vesc_protocol.cli import add_connection_args, connection_values
from vesc_protocol.parsers import parse_values


def main() -> None:
    parser = argparse.ArgumentParser(description="Simpan telemetri lengkap ke CSV")
    add_connection_args(parser)
    parser.add_argument("--output", default="vesc_telemetry.csv")
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=60.0, help="detik; 0=sampai Ctrl+C")
    args = parser.parse_args()
    if not 0.2 <= args.hz <= 100:
        raise SystemExit("--hz harus 0.2..100")
    if not 0 <= args.duration <= 86400:
        raise SystemExit("--duration harus 0..86400 detik")
    interval = 1.0 / args.hz
    deadline = None if args.duration == 0 else time.monotonic() + args.duration
    writer = None
    count = 0
    with open(args.output, "w", newline="", encoding="utf-8") as csv_file:
        with VescClient(**connection_values(args)) as client:
            try:
                while deadline is None or time.monotonic() < deadline:
                    started = time.monotonic()
                    data = parse_values(
                        client.request(bytes((Command.GET_VALUES,)), can_id=args.can_id)
                    )
                    row = {
                        "unix_time": f"{time.time():.6f}",
                        "target": "local" if args.can_id is None else args.can_id,
                        **data,
                    }
                    if writer is None:
                        writer = csv.DictWriter(csv_file, fieldnames=list(row))
                        writer.writeheader()
                    writer.writerow(row)
                    csv_file.flush()
                    count += 1
                    delay = interval - (time.monotonic() - started)
                    if delay > 0:
                        time.sleep(delay)
            except KeyboardInterrupt:
                pass
    print(f"Tersimpan {count} sampel ke {args.output}")


if __name__ == "__main__":
    main()
