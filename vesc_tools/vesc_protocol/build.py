"""Encoder perintah VESC — menghasilkan payload persis seperti ``commands.cpp``.

Tiap fungsi mengembalikan ``bytes`` payload (tanpa framing/CRC). Framing
(0x02/0x03 header + CRC16 + stop 0x03) dilakukan oleh :mod:`vesc_protocol.packet`
saat dikirim lewat :class:`vesc_protocol.client.VescClient`.

Skala (factor) di bawah disalin persis dari C++:
    SET_DUTY            -> vbAppendDouble32(duty, 1e5)
    SET_CURRENT         -> vbAppendDouble32(current, 1e3)
    SET_CURRENT_BRAKE   -> vbAppendDouble32(current, 1e3)
    SET_RPM             -> vbAppendInt32(rpm)
    SET_POS             -> vbAppendDouble32(pos, 1e6)
    SET_HANDBRAKE       -> vbAppendDouble32(current, 1e3)
    SET_SERVO_POS       -> vbAppendDouble16(pos, 1e3)
    SET_CURRENT_REL     -> vbAppendDouble32(current, 1e5)
    SET_DETECT          -> vbAppendInt8(mode)
"""

from .buffer import (
    pack_i16,
    pack_i32,
    pack_u16,
    pack_u32,
    pack_u8,
    scaled_i16,
    scaled_i32,
)
from .ids import Command


def _cmd(code: int) -> bytes:
    return bytes((code,))


def build_fw_version() -> bytes:
    return _cmd(Command.FW_VERSION)


def build_get_values() -> bytes:
    return _cmd(Command.GET_VALUES)


def build_get_values_selective(mask: int) -> bytes:
    return _cmd(Command.GET_VALUES_SELECTIVE) + pack_u32(mask)


def build_get_values_setup() -> bytes:
    return _cmd(Command.GET_VALUES_SETUP)


def build_get_values_setup_selective(mask: int) -> bytes:
    return _cmd(Command.GET_VALUES_SETUP_SELECTIVE) + pack_u32(mask)


def build_get_imu(mask: int) -> bytes:
    return _cmd(Command.GET_IMU_DATA) + pack_u16(mask)


def build_get_decoded_ppm() -> bytes:
    return _cmd(Command.GET_DECODED_PPM)


def build_get_decoded_adc() -> bytes:
    return _cmd(Command.GET_DECODED_ADC)


def build_get_decoded_chuk() -> bytes:
    return _cmd(Command.GET_DECODED_CHUK)


def build_get_decoded_balance() -> bytes:
    return _cmd(Command.GET_DECODED_BALANCE)


def build_get_rotor_position() -> bytes:
    return _cmd(Command.ROTOR_POSITION)


def build_ping_can() -> bytes:
    return _cmd(Command.PING_CAN)


def build_alive() -> bytes:
    return _cmd(Command.ALIVE)


def build_terminal_cmd(command: str) -> bytes:
    return _cmd(Command.TERMINAL_CMD) + command.encode("utf-8", "replace")


def build_terminal_cmd_sync(command: str) -> bytes:
    return _cmd(Command.TERMINAL_CMD_SYNC) + command.encode("utf-8", "replace")


def build_set_duty(duty: float) -> bytes:
    return _cmd(Command.SET_DUTY) + scaled_i32(duty, 100_000.0)


def build_set_current(current: float) -> bytes:
    return _cmd(Command.SET_CURRENT) + scaled_i32(current, 1_000.0)


def build_set_current_brake(current: float) -> bytes:
    return _cmd(Command.SET_CURRENT_BRAKE) + scaled_i32(current, 1_000.0)


def build_set_rpm(rpm: int) -> bytes:
    return _cmd(Command.SET_RPM) + pack_i32(int(round(rpm)))


def build_set_pos(pos: float) -> bytes:
    return _cmd(Command.SET_POS) + scaled_i32(pos, 1_000_000.0)


def build_set_handbrake(current: float) -> bytes:
    return _cmd(Command.SET_HANDBRAKE) + scaled_i32(current, 1_000.0)


def build_set_servo_pos(pos: float) -> bytes:
    return _cmd(Command.SET_SERVO_POS) + scaled_i16(pos, 1_000.0)


def build_set_current_rel(current: float) -> bytes:
    return _cmd(Command.SET_CURRENT_REL) + scaled_i32(current, 100_000.0)


def build_set_detect(mode: int) -> bytes:
    return _cmd(Command.SET_DETECT) + pack_u8(int(mode) & 0xFF)


def build_forward_can(can_id: int, inner_payload: bytes) -> bytes:
    """COMM_FORWARD_CAN: 1 byte id lalu payload perintah asli (seperti C++)."""
    if not 0 <= can_id <= 254:
        raise ValueError("CAN ID harus 0..254")
    return _cmd(Command.FORWARD_CAN) + pack_u8(can_id) + bytes(inner_payload)
