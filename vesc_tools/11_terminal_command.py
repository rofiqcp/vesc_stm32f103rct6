#!/usr/bin/env python3
import argparse
import time

from vesc_protocol import Command, VescClient
from vesc_protocol.cli import add_connection_args, connection_values


def main() -> None:
    parser = argparse.ArgumentParser(description="Jalankan satu terminal command VESC")
    add_connection_args(parser)
    parser.add_argument("command", help='contoh: "faults"')
    parser.add_argument("--listen", type=float, default=1.5)
    args = parser.parse_args()
    payload = bytes((Command.TERMINAL_CMD_SYNC,)) + args.command.encode("utf-8")
    with VescClient(**connection_values(args)) as client:
        client.clear_input()
        client.command(payload, args.can_id)
        deadline = time.monotonic() + args.listen
        printed = False
        while time.monotonic() < deadline:
            reply = client.receive(deadline - time.monotonic())
            if reply is None:
                break
            if reply and reply[0] == Command.PRINT:
                print(reply[1:].decode("utf-8", errors="replace"), end="", flush=True)
                printed = True
        if not printed:
            print("Tidak ada COMM_PRINT diterima (command mungkin tidak menghasilkan teks).")


if __name__ == "__main__":
    main()
