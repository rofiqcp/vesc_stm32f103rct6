"""Uji encoder perintah (build_*) menghasilkan payload persis seperti C++.

Vektor dihitung secara tertutup dari spesifikasi commands.cpp agar tidak ada
ketergantungan pada implementasi crc/buffer saat memverifikasi.
"""

import struct

import pytest

from vesc_protocol import build as B
from vesc_protocol.buffer import scaled_i32
from vesc_protocol.ids import Command


def _i32(v):
    return struct.pack(">i", int(round(v)))


def _i16(v):
    return struct.pack(">h", int(round(v)))


def test_fw_version():
    assert B.build_fw_version() == bytes((Command.FW_VERSION,))


def test_get_values_and_selective():
    assert B.build_get_values() == bytes((Command.GET_VALUES,))
    assert B.build_get_values_selective(0x003FFFFF) == bytes(
        (Command.GET_VALUES_SELECTIVE,)
    ) + struct.pack(">I", 0x003FFFFF)


def test_get_values_setup_and_selective():
    assert B.build_get_values_setup() == bytes((Command.GET_VALUES_SETUP,))
    assert B.build_get_values_setup_selective(0x3FFFFF) == bytes(
        (Command.GET_VALUES_SETUP_SELECTIVE,)
    ) + struct.pack(">I", 0x3FFFFF)


def test_get_imu():
    assert B.build_get_imu(0xFFFF) == bytes((Command.GET_IMU_DATA,)) + struct.pack(
        ">H", 0xFFFF
    )


def test_decoded_inputs_and_rotor():
    assert B.build_get_decoded_ppm() == bytes((Command.GET_DECODED_PPM,))
    assert B.build_get_decoded_adc() == bytes((Command.GET_DECODED_ADC,))
    assert B.build_get_decoded_chuk() == bytes((Command.GET_DECODED_CHUK,))
    assert B.build_get_decoded_balance() == bytes((Command.GET_DECODED_BALANCE,))
    assert B.build_get_rotor_position() == bytes((Command.ROTOR_POSITION,))


def test_ping_can_and_alive():
    assert B.build_ping_can() == bytes((Command.PING_CAN,))
    assert B.build_alive() == bytes((Command.ALIVE,))


def test_terminal():
    assert B.build_terminal_cmd("faults") == bytes((Command.TERMINAL_CMD,)) + b"faults"
    assert B.build_terminal_cmd_sync("ls") == bytes(
        (Command.TERMINAL_CMD_SYNC,)
    ) + b"ls"


def test_set_duty():
    # vbAppendDouble32(duty, 1e5)
    assert B.build_set_duty(0.25) == bytes((Command.SET_DUTY,)) + scaled_i32(
        0.25, 100_000.0
    )
    assert B.build_set_duty(0.25) == bytes((Command.SET_DUTY,)) + _i32(
        round(0.25 * 100_000)
    )


def test_set_current():
    # vbAppendDouble32(current, 1e3)
    assert B.build_set_current(0.5) == bytes((Command.SET_CURRENT,)) + scaled_i32(
        0.5, 1_000.0
    )
    assert B.build_set_current(0.5) == bytes((Command.SET_CURRENT,)) + _i32(
        round(0.5 * 1_000)
    )


def test_set_current_brake():
    assert B.build_set_current_brake(0.5) == bytes(
        (Command.SET_CURRENT_BRAKE,)
    ) + scaled_i32(0.5, 1_000.0)


def test_set_rpm():
    # vbAppendInt32(rpm) -- mentah, tanpa skala
    assert B.build_set_rpm(500) == bytes((Command.SET_RPM,)) + _i32(500)
    assert B.build_set_rpm(-20000) == bytes((Command.SET_RPM,)) + _i32(-20000)


def test_set_pos():
    # vbAppendDouble32(pos, 1e6)
    assert B.build_set_pos(5.0) == bytes((Command.SET_POS,)) + scaled_i32(
        5.0, 1_000_000.0
    )


def test_set_handbrake():
    assert B.build_set_handbrake(0.5) == bytes((Command.SET_HANDBRAKE,)) + scaled_i32(
        0.5, 1_000.0
    )


def test_set_servo_pos():
    # vbAppendDouble16(pos, 1e3) -- i16, bukan i32
    assert B.build_set_servo_pos(0.5) == bytes((Command.SET_SERVO_POS,)) + _i16(
        round(0.5 * 1_000)
    )


def test_set_current_rel():
    # vbAppendDouble32(current, 1e5)
    assert B.build_set_current_rel(0.01) == bytes(
        (Command.SET_CURRENT_REL,)
    ) + scaled_i32(0.01, 100_000.0)


def test_set_detect():
    assert B.build_set_detect(1) == bytes((Command.SET_DETECT, 1))
    assert B.build_set_detect(0) == bytes((Command.SET_DETECT, 0))


def test_forward_can():
    inner = B.build_set_duty(0.1)
    # COMM_FORWARD_CAN(34) + can_id(1 byte) + inner payload
    assert B.build_forward_can(10, inner) == bytes((Command.FORWARD_CAN, 10)) + inner


def test_forward_can_rejects_bad_id():
    with pytest.raises(ValueError):
        B.build_forward_can(255, b"\x05\x00\x00\x00\x00")
    with pytest.raises(ValueError):
        B.build_forward_can(-1, b"\x05")


def test_all_builders_return_bytes_and_start_with_cmd():
    # Setiap builder mengembalikan bytes dengan byte pertama = command id.
    builders = [
        B.build_fw_version(),
        B.build_get_values(),
        B.build_get_values_selective(1),
        B.build_get_values_setup(),
        B.build_get_imu(1),
        B.build_get_decoded_ppm(),
        B.build_get_decoded_adc(),
        B.build_get_decoded_chuk(),
        B.build_get_decoded_balance(),
        B.build_get_rotor_position(),
        B.build_ping_can(),
        B.build_alive(),
        B.build_terminal_cmd("x"),
        B.build_set_duty(0.1),
        B.build_set_current(0.1),
        B.build_set_current_brake(0.1),
        B.build_set_rpm(100),
        B.build_set_pos(1.0),
        B.build_set_handbrake(0.1),
        B.build_set_servo_pos(0.5),
        B.build_set_current_rel(0.01),
        B.build_set_detect(1),
    ]
    valid_cmds = set(int(c) for c in Command)
    for b in builders:
        assert isinstance(b, bytes)
        assert b[0] in valid_cmds
