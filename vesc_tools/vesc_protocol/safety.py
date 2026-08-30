import argparse
import signal
import time

from .buffer import scaled_i32
from .ids import Command


class StopRequested(Exception):
    pass


def require_arm(args) -> None:
    if not args.arm:
        raise SystemExit(
            "Ditolak demi keselamatan. Pastikan roda terangkat lalu tambahkan --arm."
        )


def add_motion_args(parser: argparse.ArgumentParser, default_duration: float = 1.0) -> None:
    parser.add_argument("--duration", type=float, default=default_duration)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument(
        "--arm",
        action="store_true",
        help="konfirmasi area aman dan motor boleh bergerak",
    )


def validate_timing(duration: float, hz: float) -> None:
    if not 0.05 <= duration <= 30.0:
        raise SystemExit("--duration harus 0.05..30 detik")
    if not 1.0 <= hz <= 100.0:
        raise SystemExit("--hz harus 1..100")


def stop_motor(client, can_id: int | None, repeat: int = 3) -> None:
    zero = bytes((Command.SET_DUTY,)) + scaled_i32(0.0, 100_000.0)
    for _ in range(repeat):
        client.command(zero, can_id)
        time.sleep(0.03)


def run_bounded(client, can_id: int | None, payload: bytes, duration: float, hz: float) -> None:
    validate_timing(duration, hz)
    interrupted = False

    def handle_signal(signum, frame):
        nonlocal interrupted
        interrupted = True

    previous_int = signal.signal(signal.SIGINT, handle_signal)
    previous_term = signal.signal(signal.SIGTERM, handle_signal)
    try:
        interval = 1.0 / hz
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline and not interrupted:
            started = time.monotonic()
            client.command(payload, can_id)
            delay = interval - (time.monotonic() - started)
            if delay > 0:
                time.sleep(delay)
    finally:
        stop_motor(client, can_id)
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)
