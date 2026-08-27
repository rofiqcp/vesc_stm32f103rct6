"""Uji codec buffer: integer big-endian, scaled float, dan float32-auto.

Float32-auto mengikuti vbAppendDouble32Auto / vbPopFrontDouble32Auto di
vbytearray.cpp (format: 1 bit tanda, 8 bit eksponen bias-126, 23 bit pecahan
dengan implicit 0.5; subnormal = 0)."""

import math
import struct

import pytest

from vesc_protocol.buffer import (
    BufferUnderflow,
    Reader,
    pack_i16,
    pack_i32,
    pack_u16,
    pack_u32,
    pack_u8,
    scaled_i16,
    scaled_i32,
)


def test_pack_roundtrip():
    assert pack_i16(-1234) == struct.pack(">h", -1234)
    assert pack_u16(0xABCD) == struct.pack(">H", 0xABCD)
    assert pack_i32(-70000) == struct.pack(">i", -70000)
    assert pack_u32(0xDEADBEEF) == struct.pack(">I", 0xDEADBEEF)
    assert pack_u8(200) == b"\xc8"


def test_scaled_i32_roundtrip():
    # scaled_i32(v, scale) == pack_i32(round(v*scale))
    v = 45.678
    assert scaled_i32(v, 1000) == struct.pack(">i", round(v * 1000))
    assert scaled_i32(-12.34, 100) == struct.pack(">i", round(-12.34 * 100))


def test_scaled_i16_roundtrip():
    assert scaled_i16(-12.3, 10) == struct.pack(">h", round(-12.3 * 10))


def test_reader_integers():
    r = Reader(pack_i32(-5) + pack_u16(0x1234) + pack_i16(-300) + pack_u8(9))
    assert r.i32() == -5
    assert r.u16() == 0x1234
    assert r.i16() == -300
    assert r.u8() == 9
    assert r.remaining == 0


def test_reader_scaled():
    r = Reader(scaled_i16(-12.3, 10) + scaled_i32(45.678, 1000))
    assert r.scaled16(10) == pytest.approx(-12.3, abs=1e-3)
    assert r.scaled32(1000) == pytest.approx(45.678, abs=1e-3)


def test_reader_underflow():
    r = Reader(b"\x01\x02")
    with pytest.raises(BufferUnderflow):
        r.i32()
    r2 = Reader(b"")
    with pytest.raises(BufferUnderflow):
        r2.u8()


def test_auto32_zero():
    # 0.0 dienkode sebagai 4 byte nol
    r = Reader(struct.pack(">I", 0))
    assert r.auto32() == 0.0


def test_auto32_one_and_half():
    # VESC double32auto(1.0): frexp(1.0)=(0.5,1) -> fr_s=0, e=127 -> 0x3F800000
    r = Reader(struct.pack(">I", 0x3F800000))
    assert r.auto32() == pytest.approx(1.0, abs=1e-6)


def test_auto32_negative_two_point_five():
    # VESC double32auto(-2.5): frexp(-2.5)=(-0.625,2) -> fr_s=0x200000, e=128
    # -> 0x40200000, tanda -> 0xC0200000
    r = Reader(struct.pack(">I", 0xC0200000))
    assert r.auto32() == pytest.approx(-2.5, abs=1e-6)


def test_auto32_small_eps():
    # nilai kecil: 1.5e-38 -> subnormal di C++ = 0.0
    r = Reader(struct.pack(">f", 1.5e-38))
    # Py membaca float32 asli; dekoder VESC mensubstitusi 0 untuk <1.5e-38
    val = struct.unpack(">f", struct.pack(">f", 1.5e-38))[0]
    assert r.auto32() == (0.0 if abs(val) < 1.5e-38 else pytest.approx(val, rel=1e-6))


def test_auto32_roundtrip_random():
    import random

    random.seed(1)
    for _ in range(200):
        val = random.uniform(-50.0, 50.0)
        encoded = struct.pack(">f", float(val))
        r = Reader(encoded)
        got = r.auto32()
        # presisi float32 -> toleransi relatif longgar
        assert got == pytest.approx(val, rel=1e-5, abs=1e-7)


def test_auto32_exponent_edge():
    # nilai sangat kecil tapi di atas ambang: ~1e-37
    val = 1.0e-37
    r = Reader(struct.pack(">f", val))
    got = r.auto32()
    assert got == pytest.approx(val, rel=1e-4, abs=1e-39)


def test_scaled_rounding_matches_cpp_half_away_from_zero():
    # C++ roundDouble(x): floor(x+0.5) untuk x>=0, ceil(x-0.5) untuk x<0.
    # Bukan banker's rounding Python. Verifikasi nilai .5 tepat.
    # 2.5 * 1000 = 2500 harus -> 2500 (bukan 2499 via round(2.5)? untuk 2.5
    # round() Python -> 2, tapi di sini value*scale; uji langsung scaled_i32).
    # Kasus kritis: 0.0025 * 100000 = 250.0 -> 250 (aman).
    # Kasus setengah: 0.000025 * 1e6 = 25.0 -> 25.
    # Gunakan nilai yang memicu half-away: 1.5 * 2 = 3 (floor(1.5+0.5)=floor(2.0)=2
    # salah) -> pakai skala pecahan.
    # Verifikasi langsung: _round_double internal via scaled_i32 dengan .5.
    # 2.005 * 100 = 200.5 -> C++ floor(200.5+0.5)=floor(201.0)=201.
    assert scaled_i32(2.005, 100) == struct.pack(">i", 201)
    # -2.005 * 100 = -200.5 -> C++ ceil(-200.5-0.5)=ceil(-201.0)=-201.
    assert scaled_i32(-2.005, 100) == struct.pack(">i", -201)
    # Bandingkan dengan round() Python yang salah untuk half:
    # round(200.5) = 200 (banker), round(-200.5) = -200 (banker).
    assert struct.pack(">i", round(200.5)) == struct.pack(">i", 200)
    assert struct.pack(">i", round(-200.5)) == struct.pack(">i", -200)
    # Pastikan implementasi kita beda (bukan banker).
    assert scaled_i32(2.005, 100) != struct.pack(">i", round(200.5))


def test_scaled_i16_rounding():
    # 2.005 * 10 = 20.05 -> C++ floor(20.05+0.5)=floor(20.55)=20.
    assert scaled_i16(2.005, 10) == struct.pack(">h", 20)
    # 2.05 * 10 = 20.5 -> C++ floor(20.5+0.5)=floor(21.0)=21 (half-away).
    assert scaled_i16(2.05, 10) == struct.pack(">h", 21)


def test_c_string():
    raw = b"HW_4_1\x00sisa"
    r = Reader(raw)
    assert r.c_string() == "HW_4_1"
    assert r.remaining == len(b"sisa")


def test_c_string_missing_terminator():
    r = Reader(b"no terminator")
    with pytest.raises(BufferUnderflow):
        r.c_string()
