#!/usr/bin/env python3
import struct

from vesc_protocol.buffer import Reader, scaled_i16, scaled_i32
from vesc_protocol.crc import crc16
from vesc_protocol.packet import PacketParser, frame


def test_crc() -> None:
    assert crc16(b"123456789") == 0x31C3


def test_packets() -> None:
    for size in (1, 255, 256, 65535, 65536):
        payload = bytes((i * 17 + 3) & 0xFF for i in range(size))
        parser = PacketParser(max_payload=70000)
        wire = frame(payload)
        midpoint = len(wire) // 2
        parser.feed(b"noise" + wire[:midpoint])
        assert parser.pop() is None
        parser.feed(wire[midpoint:])
        assert parser.pop() == payload
        assert parser.pop() is None


def test_bad_crc_resync() -> None:
    bad = bytearray(frame(b"bad"))
    bad[-2] ^= 0x01
    good_payload = b"good"
    parser = PacketParser()
    parser.feed(bytes(bad) + frame(good_payload))
    assert parser.pop() == good_payload


def test_buffers() -> None:
    reader = Reader(scaled_i16(-12.3, 10) + scaled_i32(45.678, 1000))
    assert reader.scaled16(10) == -12.3
    assert abs(reader.scaled32(1000) - 45.678) < 1e-9
    reader = Reader(struct.pack(">f", 1.0) + struct.pack(">f", -2.5))
    assert reader.auto32() == 1.0
    assert reader.auto32() == -2.5


def main() -> None:
    test_crc()
    test_packets()
    test_bad_crc_resync()
    test_buffers()
    print("OK: CRC, framing 2/3/4-byte, resync, dan buffer codec lulus.")


if __name__ == "__main__":
    main()
