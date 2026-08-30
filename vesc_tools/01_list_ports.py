#!/usr/bin/env python3
from vesc_protocol.cli import available_ports


def main() -> None:
    ports = available_ports()
    if not ports:
        print("Tidak ada port serial VESC yang tampak.")
        print("Coba: ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null")
        return
    print("Port serial yang ditemukan:")
    for port in ports:
        print(f"  {port}")


if __name__ == "__main__":
    main()
