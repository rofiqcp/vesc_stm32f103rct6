"""Uji VescClient tanpa hardware: transport palsu menggantikan serial.

Menguji: framing/deframing end-to-end, request/reply dengan command yang benar,
forward CAN (menyisipkan 1 byte ID), retry, timeout, dan clear_input.
"""

import time

import pytest

from vesc_protocol import Command, VescTimeout
from vesc_protocol.client import VescClient, direct_or_can
from vesc_protocol.packet import frame


class FakeSerial:
    """Transport dua arah: script menulis, lalu 'perangkat' membalas.

    Balasan diproduksi oleh fungsi ``respond(payload)`` yang diberikan.
    """

    def __init__(self, respond):
        self._respond = respond
        self._rx = bytearray()  # data yang diterima dari perangkat (untuk read)
        self._written = bytearray()
        self.flushed = 0

    def open(self):
        return self

    def close(self):
        pass

    def flush_input(self):
        self.flushed += 1
        self._rx.clear()

    def write_all(self, data, timeout=1.0):
        self._written.extend(data)
        # Perangkat langsung membalas (model echo dengan encode).
        reply = self._respond(bytes(data))
        if reply is not None:
            self._rx.extend(reply)

    def read(self, timeout=0.1, max_bytes=4096):
        chunk = bytes(self._rx[:max_bytes])
        del self._rx[: len(chunk)]
        return chunk


def _device_echo(wire: bytes) -> bytes:
    # ``wire`` sudah ter-framing (yang dikirim client.send). Perangkat nyata
    # (VESC lokal) mengupas framing -> payload. Bila payload adalah FORWARD_CAN,
    # VESC lokal mengupas FORWARD_CAN + can_id lalu VESC remote membalas dengan
    # payload dalam + 1 byte echo; VESC lokal meneruskan balasan tersebut
    # (TANPA pembungkus FORWARD_CAN).
    from vesc_protocol.packet import PacketParser
    from vesc_protocol.ids import Command

    parser = PacketParser()
    parser.feed(wire)
    payload = parser.pop()
    if payload and payload[0] == Command.FORWARD_CAN:
        inner = payload[2:]  # lewati FORWARD_CAN(1) + can_id(1)
        return frame(inner + b"\xAB")
    return frame(payload + b"\xAB")


def _client(respond=_device_echo):
    serial = FakeSerial(respond)
    client = VescClient.__new__(VescClient)
    client.serial = serial
    client.timeout = 0.5
    client.retries = 1
    from vesc_protocol.packet import PacketParser

    client.parser = PacketParser()
    return client, serial


def test_request_reply_roundtrip():
    client, serial = _client()
    inner = bytes((Command.FW_VERSION,))
    reply = client.request(inner)
    assert reply[0] == Command.FW_VERSION
    assert reply[-1] == 0xAB  # echo isi
    # apa yang dikirim ke serial sudah ter-framing
    assert serial._written[0] == 0x02


def test_command_no_reply_requested():
    # command() tidak menunggu balasan
    client, serial = _client()
    inner = bytes((Command.ALIVE,))
    client.command(inner)
    assert serial._written == frame(inner)


def test_direct_or_can_local():
    assert direct_or_can(bytes((Command.FW_VERSION,)), None) == bytes(
        (Command.FW_VERSION,)
    )


def test_direct_or_can_forward():
    inner = bytes((Command.SET_DUTY, 1, 2, 3, 4))
    out = direct_or_can(inner, 10)
    assert out == bytes((Command.FORWARD_CAN, 10)) + inner


def test_direct_or_can_bad_id():
    with pytest.raises(ValueError):
        direct_or_can(b"\x05", 300)


def test_request_forward_can():
    client, serial = _client()
    inner = bytes((Command.SET_DUTY,)) + (1234).to_bytes(4, "big", signed=True)
    reply = client.request(inner, can_id=5)
    # yang dikirim ke serial = FORWARD_CAN + 5 + inner, sudah ter-framing
    sent = bytes(serial._written)
    assert sent[0] == 0x02
    assert sent[2] == Command.FORWARD_CAN
    assert sent[3] == 5
    # VESC lokal membuka FORWARD_CAN: balasan remote = inner (SET_DUTY + payload)
    # + 1 byte echo (tanpa can_id, karena itu dibuang VESC remote).
    assert reply[0] == Command.SET_DUTY
    assert reply[1:] == inner[1:] + b"\xAB"


def test_request_timeout_raises():
    # Perangkat tidak membalas sama sekali.
    client, _ = _client(respond=lambda p: None)
    client.timeout = 0.2
    client.retries = 0
    with pytest.raises(VescTimeout):
        client.request(bytes((Command.FW_VERSION,)))


def test_request_wrong_command_keeps_trying():
    # Perangkat membalas command SALAH; request harus timeout (retry habis).
    def bad_respond(payload):
        return frame(bytes((Command.PRINT,)) + b"no")

    client, _ = _client(respond=bad_respond)
    client.timeout = 0.2
    client.retries = 1
    with pytest.raises(VescTimeout):
        client.request(bytes((Command.FW_VERSION,)))


def test_clear_input_flushes_and_resets_parser():
    client, serial = _client()
    client.clear_input()
    assert serial.flushed >= 1
    assert len(client.parser.buffer) == 0


def test_receive_partial_then_complete():
    client, serial = _client()
    client.parser.clear()
    serial._rx.clear()
    serial._written.clear()
    # Tulis manual sebagian lalu sisa, simulasi stream terpotong.
    full = frame(bytes((Command.FW_VERSION,)) + b"\xCD")
    first, second = full[:3], full[3:]
    serial._rx.extend(first)
    assert client.receive(timeout=0.05) is None
    serial._rx.extend(second)
    got = client.receive(timeout=0.05)
    assert got is not None and got[0] == Command.FW_VERSION


def test_context_manager():
    client, serial = _client()
    opened = {"v": False}

    def fake_open():
        opened["v"] = True
        return client

    client.open = fake_open
    with client:
        pass
    assert opened["v"] is True
