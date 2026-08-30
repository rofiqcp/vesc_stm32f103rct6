import argparse

from .cli import add_connection_args, connection_values
from .safety import add_motion_args, require_arm, run_bounded


def run_motion_script(
    description: str,
    value_name: str,
    unit: str,
    build_payload,
    validate_value,
) -> None:
    parser = argparse.ArgumentParser(description=description)
    add_connection_args(parser)
    add_motion_args(parser)
    parser.add_argument(value_name, type=float, help=unit)
    args = parser.parse_args()
    require_arm(args)
    value = getattr(args, value_name.lstrip("-").replace("-", "_"))
    validate_value(value)
    from .client import VescClient

    target = "lokal" if args.can_id is None else f"CAN {args.can_id}"
    print(
        f"ARM aktif: kirim {value:g} {unit} ke {target} selama "
        f"{args.duration:g}s; lalu SET_DUTY=0."
    )
    with VescClient(**connection_values(args)) as client:
        run_bounded(client, args.can_id, build_payload(value), args.duration, args.hz)
    print("Selesai; perintah stop sudah dikirim.")


def between(low: float, high: float):
    def validate(value: float) -> None:
        if not low <= value <= high:
            raise SystemExit(f"nilai harus {low:g}..{high:g}")
    return validate
