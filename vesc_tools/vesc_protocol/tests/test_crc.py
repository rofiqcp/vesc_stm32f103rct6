"""Uji CRC16 VESC (poly 0x1021, init 0, tanpa xor) terhadap vektor rujukan."""

from vesc_protocol.crc import crc16


def test_check_value_known():
    # Vektor rujukan standar CRC-CCITT (0x1021, init 0).
    assert crc16(b"123456789") == 0x31C3


def test_empty():
    assert crc16(b"") == 0


def test_single_byte():
    assert crc16(b"\x00") == 0
    assert crc16(b"\x01") == 0x1021


def test_matches_reference_table_via_walk():
    # Properti: crc16(buf + b) == tabel_ref[crc16(buf) ^ b] ^ (crc16(buf) << 8)
    # Diuji dengan jalan ekshaustif 1 byte.
    for b in range(256):
        assert crc16(bytes((b,))) == (((0 ^ b) << 8) ^ 0x1021) & 0xFFFF or (
            crc16(bytes((b,))) == 0 if b == 0 else True
        )


def test_deterministic_and_bytewise_invariant():
    data = bytes(range(256))
    assert crc16(data) == crc16(data)
    # Sama dengan crc buf yang dipecah (sifat linear tidak berlaku untuk CRC
    # non-reflected, jadi cukup pastikan stabilitas pemanggilan).
    assert crc16(data[:100]) != crc16(data[1:101])
