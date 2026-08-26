import math
import struct


def pack_i16(value: int) -> bytes:
    return struct.pack(">h", value)


def pack_u16(value: int) -> bytes:
    return struct.pack(">H", value)


def pack_i32(value: int) -> bytes:
    return struct.pack(">i", value)


def pack_u32(value: int) -> bytes:
    return struct.pack(">I", value)


def scaled_i16(value: float, scale: float) -> bytes:
    return pack_i16(round(value * scale))


def scaled_i32(value: float, scale: float) -> bytes:
    return pack_i32(round(value * scale))


class BufferUnderflow(ValueError):
    pass


class Reader:
    def __init__(self, data: bytes):
        self.data = data
        self.index = 0

    @property
    def remaining(self) -> int:
        return len(self.data) - self.index

    def take(self, size: int) -> bytes:
        end = self.index + size
        if end > len(self.data):
            raise BufferUnderflow(
                f"butuh {size} byte, tersisa {self.remaining} byte"
            )
        result = self.data[self.index:end]
        self.index = end
        return result

    def u8(self) -> int:
        return self.take(1)[0]

    def i16(self) -> int:
        return struct.unpack(">h", self.take(2))[0]

    def u16(self) -> int:
        return struct.unpack(">H", self.take(2))[0]

    def i32(self) -> int:
        return struct.unpack(">i", self.take(4))[0]

    def u32(self) -> int:
        return struct.unpack(">I", self.take(4))[0]

    def scaled16(self, scale: float) -> float:
        return self.i16() / scale

    def scaled32(self, scale: float) -> float:
        return self.i32() / scale

    def auto32(self) -> float:
        raw = self.u32()
        exponent = (raw >> 23) & 0xFF
        significand_raw = raw & 0x7FFFFF
        negative = bool(raw & 0x80000000)
        if exponent == 0 and significand_raw == 0:
            return 0.0
        significand = significand_raw / 16_777_216.0 + 0.5
        exponent -= 126
        if negative:
            significand = -significand
        return math.ldexp(significand, exponent)

    def c_string(self) -> str:
        end = self.data.find(b"\x00", self.index)
        if end < 0:
            raise BufferUnderflow("string tidak memiliki terminator NUL")
        raw = self.data[self.index:end]
        self.index = end + 1
        return raw.decode("utf-8", errors="replace")
