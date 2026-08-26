import time

from .ids import Command
from .packet import PacketParser, frame
from .serial_linux import LinuxSerial


class VescTimeout(TimeoutError):
    pass


def direct_or_can(inner_payload: bytes, can_id: int | None) -> bytes:
    if can_id is None:
        return inner_payload
    if not 0 <= can_id <= 254:
        raise ValueError("CAN ID harus 0..254")
    return bytes((Command.FORWARD_CAN, can_id)) + inner_payload


class VescClient:
    def __init__(
        self,
        port: str,
        baud: int = 115200,
        timeout: float = 1.0,
        retries: int = 1,
    ):
        self.serial = LinuxSerial(port, baud)
        self.timeout = timeout
        self.retries = retries
        self.parser = PacketParser()

    def open(self) -> "VescClient":
        self.serial.open()
        return self

    def close(self) -> None:
        self.serial.close()

    def clear_input(self) -> None:
        self.parser.clear()
        self.serial.flush_input()

    def send(self, payload: bytes) -> None:
        self.serial.write_all(frame(payload), timeout=self.timeout)

    def receive(self, timeout: float | None = None) -> bytes | None:
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while True:
            payload = self.parser.pop()
            if payload is not None:
                return payload
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            chunk = self.serial.read(timeout=min(remaining, 0.1))
            if chunk:
                self.parser.feed(chunk)

    def request(
        self,
        inner_payload: bytes,
        can_id: int | None = None,
        expected_command: int | None = None,
        timeout: float | None = None,
    ) -> bytes:
        if not inner_payload:
            raise ValueError("inner_payload kosong")
        expected = inner_payload[0] if expected_command is None else expected_command
        wait = self.timeout if timeout is None else timeout
        outer = direct_or_can(inner_payload, can_id)
        last_seen: int | None = None
        for _ in range(self.retries + 1):
            self.clear_input()
            self.send(outer)
            deadline = time.monotonic() + wait
            while time.monotonic() < deadline:
                reply = self.receive(deadline - time.monotonic())
                if reply is None:
                    break
                if reply and reply[0] == expected:
                    return reply
                if reply:
                    last_seen = reply[0]
        detail = "" if last_seen is None else f"; command terakhir={last_seen}"
        target = "lokal" if can_id is None else f"CAN {can_id}"
        raise VescTimeout(
            f"tidak ada balasan command {expected} dari {target} dalam {wait:.2f}s{detail}"
        )

    def command(self, inner_payload: bytes, can_id: int | None = None) -> None:
        self.send(direct_or_can(inner_payload, can_id))

    def __enter__(self) -> "VescClient":
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
