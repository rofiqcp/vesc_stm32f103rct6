#!/usr/bin/env python3
import argparse
import time

from vesc_protocol import Command, VescClient
from vesc_protocol.cli import add_connection_args, connection_values, json_line
from vesc_protocol.parsers import parse_values


def parse_ids(value: str) -> list[int]:
    ids = [int(item.strip()) for item in value.split(",") if item.strip()]
    if not ids or any(not 0 <= item <= 254 for item in ids):
        raise argparse.ArgumentTypeError("format contoh 1,2,10; ID harus 0..254")
    return ids


def main() -> None:
    parser = argparse.ArgumentParser(description="Monitor bergilir semua VESC pada CAN")
    add_connection_args(parser, can=False)
    parser.add_argument("--can-ids", type=parse_ids, help="contoh 1,2,10; kosong=PING_CAN")
    parser.add_argument("--hz", type=float, default=5.0, help="putaran scan per detik")
    parser.add_argument("--count", type=int, default=0, help="jumlah putaran; 0=terus")
    args = parser.parse_args()
    if not 0.1 <= args.hz <= 20:
        raise SystemExit("--hz harus 0.1..20")
    client_args = connection_values(args)
    with VescClient(**client_args) as client:
        ids = args.can_ids
        if ids is None:
            reply = client.request(bytes((Command.PING_CAN,)), timeout=6.0)
            ids = list(reply[1:])
        if not ids:
            raise SystemExit("Tidak ada CAN ID ditemukan.")
        print(f"# target CAN: {','.join(map(str, ids))}", flush=True)
        cycle = 0
        try:
            while args.count == 0 or cycle < args.count:
                started = time.monotonic()
                timestamp = time.strftime("%Y-%m-%dT%H:%M:%S")
                for can_id in ids:
                    try:
                        data = parse_values(
                            client.request(bytes((Command.GET_VALUES,)), can_id=can_id)
                        )
                        print(json_line({"timestamp": timestamp, "can_id": can_id, **data}), flush=True)
                    except Exception as exc:
                        print(json_line({"timestamp": timestamp, "can_id": can_id, "error": str(exc)}), flush=True)
                cycle += 1
                delay = 1.0 / args.hz - (time.monotonic() - started)
                if delay > 0:
                    time.sleep(delay)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
