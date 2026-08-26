import argparse
import glob
import json
import os


def available_ports() -> list[str]:
    patterns = (
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
        "/dev/serial/by-id/*",
        "/dev/ttyAMA*",
    )
    return sorted({path for pattern in patterns for path in glob.glob(pattern)})


def resolve_port(value: str | None) -> str:
    if value:
        return value
    ports = available_ports()
    if len(ports) == 1:
        return ports[0]
    if not ports:
        raise SystemExit("Tidak ada port serial terdeteksi. Gunakan --port /dev/ttyACM0")
    joined = "\n  ".join(ports)
    raise SystemExit(f"Ada beberapa port. Pilih dengan --port:\n  {joined}")


def add_connection_args(parser: argparse.ArgumentParser, can: bool = True) -> None:
    parser.add_argument("--port", help="contoh: /dev/ttyACM0; otomatis bila hanya satu")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--retries", type=int, default=1)
    if can:
        parser.add_argument("--can-id", type=int, help="kosong=VESC yang terhubung USB")


def connection_values(args) -> dict:
    return {
        "port": resolve_port(args.port),
        "baud": args.baud,
        "timeout": args.timeout,
        "retries": args.retries,
    }


def print_mapping(data: dict) -> None:
    width = max((len(key) for key in data), default=0)
    for key, value in data.items():
        print(f"{key:<{width}} : {value}")


def json_line(data: dict) -> str:
    return json.dumps(data, separators=(",", ":"), ensure_ascii=False)
