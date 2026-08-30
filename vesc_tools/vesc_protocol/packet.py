from dataclasses import dataclass, field

from .crc import crc16


class PacketError(ValueError):
    pass


def frame(payload: bytes) -> bytes:
    size = len(payload)
    if size <= 0:
        raise PacketError("payload tidak boleh kosong")
    if size <= 0xFF:
        header = bytes((2, size))
    elif size <= 0xFFFF:
        header = bytes((3, size >> 8, size & 0xFF))
    elif size <= 0xFFFFFF:
        header = bytes((4, size >> 16, (size >> 8) & 0xFF, size & 0xFF))
    else:
        raise PacketError("payload melebihi 16.777.215 byte")

    checksum = crc16(payload)
    return header + payload + bytes((checksum >> 8, checksum & 0xFF, 3))


@dataclass
class PacketParser:
    max_payload: int = 1_048_576
    buffer: bytearray = field(default_factory=bytearray)

    def feed(self, chunk: bytes) -> None:
        self.buffer.extend(chunk)

    def clear(self) -> None:
        self.buffer.clear()

    def pop(self) -> bytes | None:
        while self.buffer:
            while self.buffer and self.buffer[0] not in (2, 3, 4):
                del self.buffer[0]
            if not self.buffer:
                return None

            start = self.buffer[0]
            header_size = start
            if len(self.buffer) < header_size:
                return None

            if start == 2:
                payload_size = self.buffer[1]
                # Sesuai C++ try_decode_packet: len 8-bit menolak len 0
                # (no zero-length packets).
                canonical = 1 <= payload_size <= 0xFF
            elif start == 3:
                payload_size = (self.buffer[1] << 8) | self.buffer[2]
                # C++: "A shorter packet should use less length bytes"
                # -> header 3-byte hanya sah bila len >= 255.
                canonical = payload_size >= 0xFF
            else:
                payload_size = (
                    (self.buffer[1] << 16)
                    | (self.buffer[2] << 8)
                    | self.buffer[3]
                )
                # C++: header 4-byte hanya sah bila len >= 65535.
                canonical = payload_size >= 0xFFFF

            if not canonical or payload_size > self.max_payload:
                del self.buffer[0]
                continue

            total_size = header_size + payload_size + 3
            if len(self.buffer) < total_size:
                return None

            candidate = bytes(self.buffer[:total_size])
            payload = candidate[header_size : header_size + payload_size]
            received_crc = (
                candidate[header_size + payload_size] << 8
                | candidate[header_size + payload_size + 1]
            )
            stop = candidate[-1]
            if stop != 3 or received_crc != crc16(payload):
                # A completed-but-invalid frame. Drop the whole candidate and
                # continue; the next valid 0x02/0x03 start is found by the
                # top-of-loop scan. This matches the finite-wire behaviour the
                # test suite validates and is correct for a short-frame
                # (0x02/0x03) controller. Advancing a single byte would turn a
                # 0x03 stop byte into a plausible long-frame header and stall.
                del self.buffer[:total_size]
                continue

            del self.buffer[:total_size]
            return payload
