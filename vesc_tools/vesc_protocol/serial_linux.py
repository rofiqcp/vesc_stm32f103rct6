"""Serial port Linux memakai os, select, termios, dan tty dari stdlib."""

import errno
import os
import select
import termios
import time
import tty


class SerialError(OSError):
    pass


def _baud_constant(baud: int) -> int:
    name = f"B{baud}"
    if not hasattr(termios, name):
        raise SerialError(
            f"baud {baud} tidak didukung termios sistem ini; coba 115200"
        )
    return getattr(termios, name)


class LinuxSerial:
    def __init__(self, path: str, baud: int = 115200):
        self.path = path
        self.baud = baud
        self.fd: int | None = None

    def open(self) -> "LinuxSerial":
        if self.fd is not None:
            return self
        try:
            fd = os.open(
                self.path,
                os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK,
            )
        except OSError as exc:
            raise SerialError(f"gagal membuka {self.path}: {exc}") from exc

        try:
            tty.setraw(fd, termios.TCSANOW)
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
            attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
            attrs[3] = 0
            speed = _baud_constant(self.baud)
            attrs[4] = speed
            attrs[5] = speed
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIOFLUSH)
        except Exception:
            os.close(fd)
            raise
        self.fd = fd
        return self

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def flush_input(self) -> None:
        if self.fd is None:
            raise SerialError("port belum dibuka")
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def write_all(self, data: bytes, timeout: float = 1.0) -> None:
        if self.fd is None:
            raise SerialError("port belum dibuka")
        deadline = time.monotonic() + timeout
        offset = 0
        while offset < len(data):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("timeout saat menulis serial")
            _, writable, _ = select.select([], [self.fd], [], remaining)
            if not writable:
                continue
            try:
                offset += os.write(self.fd, data[offset:])
            except BlockingIOError:
                continue

    def read(self, timeout: float = 0.1, max_bytes: int = 4096) -> bytes:
        if self.fd is None:
            raise SerialError("port belum dibuka")
        readable, _, _ = select.select([self.fd], [], [], max(0.0, timeout))
        if not readable:
            return b""
        try:
            return os.read(self.fd, max_bytes)
        except BlockingIOError:
            return b""
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return b""
            raise

    def __enter__(self) -> "LinuxSerial":
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
