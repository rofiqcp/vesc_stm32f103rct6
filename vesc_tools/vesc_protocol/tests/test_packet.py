"""Uji framing paket (frame) dan parser stream terhadap spesifikasi packet.cpp.

Header:
    0x02 <len 1 byte>      untuk len <= 255
    0x03 <len 2 byte BE>   untuk len 256..65535
    0x04 <len 3 byte BE>   untuk len 65536..16777215
diikuti payload, CRC16 BE, dan stop byte 0x03.
"""

import pytest

from vesc_protocol.packet import PacketError, PacketParser, frame


def test_frame_2byte_header_small():
    payload = b"\x04\x10\x20"
    wire = frame(payload)
    assert wire[0] == 0x02
    assert wire[1] == len(payload)
    assert wire[-1] == 0x03
    # CRC = 2 byte sebelum stop
    assert bytes(wire[2 + len(payload): -1]) == _crc_bytes(payload)


def test_frame_3byte_header_boundary():
    # len == 256 harus pakai header 0x03
    payload = bytes(range(256))
    wire = frame(payload)
    assert wire[0] == 0x03
    assert (wire[1] << 8) | wire[2] == 256
    assert wire[-1] == 0x03


def test_frame_4byte_header_boundary():
    # len == 65536 harus pakai header 0x04
    payload = bytes((i & 0xFF) for i in range(65536))
    wire = frame(payload)
    assert wire[0] == 0x04
    assert ((wire[1] << 16) | (wire[2] << 8) | wire[3]) == 65536
    assert wire[-1] == 0x03


def test_frame_rejects_empty():
    with pytest.raises(PacketError):
        frame(b"")


def test_frame_rejects_oversized():
    with pytest.raises(PacketError):
        frame(bytes(16_777_216))


def test_parser_pop_basic():
    payload = b"hello"
    parser = PacketParser()
    parser.feed(frame(payload))
    assert parser.pop() == payload
    assert parser.pop() is None


def test_parser_streaming_split_anywhere():
    payload = bytes((i * 7 + 3) & 0xFF for i in range(300))
    wire = frame(payload)
    parser = PacketParser(max_payload=70000)
    # Suap byte demi byte (termasuk byte pertama) — parser harus tetap sinkron.
    for i, byte in enumerate(wire):
        parser.feed(bytes((byte,)))
        if i < len(wire) - 1:
            assert parser.pop() is None
    assert parser.pop() == payload


def test_parser_noise_before_packet():
    payload = b"good"
    parser = PacketParser()
    # Byte acak yang BUKAN 2/3/4 di awal harus dilewati parser (di sini 0x00,0x01).
    # 0x02 adalah start byte valid, jadi tidak dipakai sebagai "noise murni".
    parser.feed(b"\x00\x01\x05\x06" + frame(payload))
    assert parser.pop() == payload


def test_parser_resync_on_bad_crc():
    bad = bytearray(frame(b"bad"))
    bad[-2] ^= 0x01  # rusak CRC
    good = frame(b"good")
    parser = PacketParser()
    parser.feed(bytes(bad) + good)
    assert parser.pop() == b"good"


def test_parser_multiple_packets_one_feed():
    a, b, c = b"aaa", b"bbbbbb", b"ccccc"
    parser = PacketParser()
    parser.feed(frame(a) + frame(b) + frame(c))
    assert parser.pop() == a
    assert parser.pop() == b
    assert parser.pop() == c
    assert parser.pop() is None


def test_parser_garbage_start_bytes_skipped():
    payload = b"x"
    parser = PacketParser()
    # byte acak yang bukan 2/3/4 di awal harus dilewati parser
    parser.feed(b"\xff\xfe" + frame(payload))
    assert parser.pop() == payload


def test_parser_two_consecutive_bad_then_good():
    bad1 = bytearray(frame(b"one"))
    bad1[-2] ^= 0xFF
    bad2 = bytearray(frame(b"two"))
    bad2[-2] ^= 0xFF
    good = frame(b"three")
    parser = PacketParser()
    parser.feed(bytes(bad1) + bytes(bad2) + good)
    assert parser.pop() == b"three"


def test_parser_clear():
    parser = PacketParser()
    parser.feed(frame(b"partial"))
    parser.clear()
    assert parser.pop() is None


def _crc_bytes(payload: bytes) -> bytes:
    from vesc_protocol.crc import crc16

    c = crc16(payload)
    return bytes((c >> 8, c & 0xFF))


def test_parser_boundary_len_near_255():
    # C++: header 0x03 hanya sah bila len >= 255; len 254 harus tetap 0x02.
    payload = bytes((i & 0xFF) for i in range(254))
    wire = frame(payload)
    assert wire[0] == 0x02  # len 254 -> 2-byte header
    parser = PacketParser()
    parser.feed(wire)
    assert parser.pop() == payload


def test_frame_uses_2byte_header_at_255():
    # PENGIRIM (sendPacket C++ & firmware VESC): len <= 255 -> header 0x02.
    # Ini lah yang dipakai firmware, jadi frame() harus konsisten dg pengirim.
    payload = bytes((i & 0xFF) for i in range(255))
    wire = frame(payload)
    assert wire[0] == 0x02
    assert wire[1] == 255
    parser = PacketParser()
    parser.feed(wire)
    assert parser.pop() == payload


def test_parser_rejects_3byte_header_for_254():
    # PENERIMA (try_decode_packet C++): header 0x03 hanya sah bila len >= 255.
    # Paket sintetik 0x03 + len 254 harus ditolak (canonical=false), lalu
    # start byte 0x03 di-consumed dan resync gagal (tidak ada paket valid).
    payload = b"\x03\x00\xfe" + b"\x00" * 254 + b"\x00\x00\x03"
    parser = PacketParser()
    parser.feed(payload)
    assert parser.pop() is None


def test_parser_len_24b_rejects_65534():
    # Frame sintetik 0x04 dengan len=65534 harus ditolak (perlu >= 65535).
    payload = b"\x04\x00\xff\xfe" + b"\x00" * 100 + b"\x00\x00\x03"
    parser = PacketParser()
    parser.feed(payload)
    assert parser.pop() is None

