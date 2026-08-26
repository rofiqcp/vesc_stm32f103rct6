#!/usr/bin/env python3
import argparse

from vesc_protocol import Command, VescClient
from vesc_protocol.cli import add_connection_args, connection_values
from vesc_protocol.parsers import parse_firmware


def main() -> None:
    parser = argparse.ArgumentParser(description="Ping semua CAN ID melalui VESC USB")
    add_connection_args(parser, can=False)
    parser.set_defaults(timeout=6.0, retries=0)
    parser.add_argument("--identify", action="store_true", help="baca firmware tiap ID")
    args = parser.parse_args()
    with VescClient(**connection_values(args)) as client:
        reply = client.request(
            bytes((Command.PING_CAN,)), expected_command=Command.PING_CAN, timeout=args.timeout
        )
        ids = list(reply[1:])
        if not ids:
            print("Tidak ada node CAN lain yang membalas.")
            return
        print("CAN ID terdeteksi:", ", ".join(map(str, ids)))
        if args.identify:
            for can_id in ids:
                try:
                    fw = parse_firmware(
                        client.request(bytes((Command.FW_VERSION,)), can_id=can_id)
                    )
                    print(
                        f"  CAN {can_id:3d}: {fw.get('hardware', '?')} "
                        f"FW {fw.get('major', '?')}.{fw.get('minor', '?')}"
                    )
                except Exception as exc:
                    print(f"  CAN {can_id:3d}: gagal identifikasi: {exc}")


if __name__ == "__main__":
    main()
