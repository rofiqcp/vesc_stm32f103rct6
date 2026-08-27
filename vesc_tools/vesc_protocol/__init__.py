"""Implementasi protokol UART VESC tanpa dependensi pihak ketiga.

Sub-modul:
    ids      -- nomor command COMM_PACKET_ID (datatypes.h)
    crc      -- CRC16 poly 0x1021
    packet   -- framing 0x02/0x03 + CRC + parser stream
    buffer   -- integer big-endian, scaled, float32-auto
    build    -- encoder payload perintah (persis commands.cpp)
    parsers  -- dekoder balasan telemetri/firmware
    client   -- request/reply, timeout, retry, forward CAN
    cli      -- helper argparse port/koneksi
    safety   -- arm, batas durasi, stop motor
"""

from .build import (
    build_alive,
    build_forward_can,
    build_fw_version,
    build_get_decoded_adc,
    build_get_decoded_balance,
    build_get_decoded_chuk,
    build_get_decoded_ppm,
    build_get_imu,
    build_get_rotor_position,
    build_get_values,
    build_get_values_selective,
    build_get_values_setup,
    build_get_values_setup_selective,
    build_ping_can,
    build_set_current,
    build_set_current_brake,
    build_set_current_rel,
    build_set_duty,
    build_set_detect,
    build_set_handbrake,
    build_set_pos,
    build_set_rpm,
    build_set_servo_pos,
    build_terminal_cmd,
    build_terminal_cmd_sync,
)
from .client import VescClient, VescTimeout
from .ids import Command

__all__ = [
    "Command",
    "VescClient",
    "VescTimeout",
    "build_fw_version",
    "build_get_values",
    "build_get_values_selective",
    "build_get_values_setup",
    "build_get_values_setup_selective",
    "build_get_imu",
    "build_get_decoded_ppm",
    "build_get_decoded_adc",
    "build_get_decoded_chuk",
    "build_get_decoded_balance",
    "build_get_rotor_position",
    "build_ping_can",
    "build_alive",
    "build_terminal_cmd",
    "build_terminal_cmd_sync",
    "build_set_duty",
    "build_set_current",
    "build_set_current_brake",
    "build_set_rpm",
    "build_set_pos",
    "build_set_handbrake",
    "build_set_servo_pos",
    "build_set_current_rel",
    "build_set_detect",
    "build_forward_can",
]
