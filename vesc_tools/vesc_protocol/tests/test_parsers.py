"""Uji dekoder balasan (parsers.*) dengan vektor bit-exact dari C++.

Setiap tes menyusun payload mentah (byte command id + field) persis seperti yang
dikeluarkan firmware di commands.cpp, lalu memverifikasi parser menghasilkan
field yang selaras. Fokus utama: urutan bit mask GET_VALUES/SETUP/IMU dan
decode firmware (2 byte QML).
"""

import struct

import pytest

from vesc_protocol import parsers as P
from vesc_protocol.buffer import scaled_i32
from vesc_protocol.ids import Command, ImuMask, SetupMask, ValueMask


def _i8(v):
    return struct.pack(">b", v)


def _u8(v):
    return struct.pack(">B", v)


def _i16(v):
    return struct.pack(">h", v)


def _u16(v):
    return struct.pack(">H", v)


def _i32(v):
    return struct.pack(">i", v)


def _u32(v):
    return struct.pack(">I", v)


def _d16(v, scale):
    return _i16(round(v * scale))


def _d32(v, scale):
    return _i32(round(v * scale))


# ---------------------------------------------------------------------------
# Firmware (cmd 0)
# ---------------------------------------------------------------------------
def test_parse_firmware_minimal():
    # major i8, minor i8, hw cstring
    raw = _u8(5) + _u8(3) + b"HW_4_1\x00"
    reply = bytes((Command.FW_VERSION,)) + raw
    out = P.parse_firmware(reply)
    assert out["major"] == 5
    assert out["minor"] == 3
    assert out["hardware"] == "HW_4_1"


def test_parse_firmware_full_alignment():
    # Urutan persis C++: major, minor, hw, uuid(12), pairing, test, hwtype,
    # customcfg, phasefilters, qml_hw, qml_app, nrf_flags, fw_name, crc32
    raw = (
        _u8(6) + _u8(0)
        + b"VESC\x00"
        + bytes(range(12))  # uuid
        + _u8(1)  # pairing_done
        + _u8(0)  # test_version
        + _u8(0)  # hw_type (HW_TYPE_VESC)
        + _u8(2)  # custom_config_count
        + _u8(1)  # phase_filters
        + _u8(2)  # qml_hw fullscreen
        + _u8(1)  # qml_app
        + _u8(3)  # nrf_flags: name|pin
        + b"custom_fw\x00"
        + _u32(0x12345678)
    )
    reply = bytes((Command.FW_VERSION,)) + raw
    out = P.parse_firmware(reply)
    assert out["major"] == 6
    assert out["minor"] == 0
    assert out["hardware"] == "VESC"
    assert out["uuid"] == bytes(range(12)).hex().upper()
    assert out["pairing_done"] is True
    assert out["test_version"] == 0
    assert out["hardware_type"] == 0
    assert out["custom_config_count"] == 2
    assert out["phase_filters"] is True
    assert out["qml_hw"] == 2 and out["qml_hw_fullscreen"] is True
    assert out["qml_app"] == 1 and out["qml_app_fullscreen"] is False
    assert out["nrf_name_supported"] is True
    assert out["nrf_pin_supported"] is True
    assert out["firmware_name"] == "custom_fw"
    assert out["firmware_crc32"] == "0x12345678"


def test_parse_firmware_qml_two_bytes_required():
    # Regresi: 2 byte qml (qml_hw, qml_app) harus dibaca, lalu nrf_flags,
    # sehingga firmware_name & crc32 tidak bergeser. Paket lengkap:
    # major, minor, hw, uuid(12), pairing, test, hwtype, customcfg,
    # phasefilters, qml_hw, qml_app, nrf_flags, fw_name, crc32
    raw = (
        _u8(5) + _u8(1)
        + b"X\x00"
        + bytes(12)  # uuid (12 byte)
        + _u8(0)  # pairing
        + _u8(0)  # test
        + _u8(0)  # hwtype
        + _u8(0)  # customcfg
        + _u8(0)  # phasefilters
        + _u8(0)  # qml_hw
        + _u8(0)  # qml_app  <-- byte ke-2 wajib
        + _u8(0)  # nrf_flags
        + b"NAME\x00"
        + _u32(0xAABBCCDD)
    )
    out = P.parse_firmware(bytes((Command.FW_VERSION,)) + raw)
    assert out["firmware_name"] == "NAME"
    assert out["firmware_crc32"] == "0xAABBCCDD"
    assert "extra_hex" not in out


# ---------------------------------------------------------------------------
# GET_VALUES (cmd 4)
# ---------------------------------------------------------------------------
def test_parse_values_fault_and_status():
    # Susun hanya field yang dipilih oleh mask ALL (bit 0..20):
    # temp_mos i16, temp_motor i16, imotor i32, iin i32, id i32, iq i32,
    # duty i16, rpm i32, vin i16, ah i32, ahc i32, wh i32, whc i32,
    # tacho i32, tachoabs i32, fault u8, pos i32, cid u8,
    # mos1/2/3 i16 x3, vd/vq i32 x2, status u8
    raw = b"".join(
        [
            _d16(25.0, 10),  # temp_fet
            _d16(26.0, 10),  # temp_motor
            _d32(1.5, 100),  # motor current
            _d32(2.0, 100),  # input current
            _d32(0.7, 100),  # id
            _d32(0.8, 100),  # iq
            _d16(0.123, 1000),  # duty
            _i32(1234),  # erpm
            _d16(48.0, 10),  # vin
            _d32(0.5, 10000),  # ah
            _d32(0.1, 10000),  # ahc
            _d32(10.0, 10000),  # wh
            _d32(2.0, 10000),  # whc
            _i32(5000),  # tacho
            _i32(9000),  # tacho_abs
            _u8(0),  # fault NONE
            _d32(12.5, 1_000_000),  # pid pos
            _u8(42),  # controller id
            _d16(24.0, 10),  # mos1
            _d16(24.1, 10),  # mos2
            _d16(24.2, 10),  # mos3
            _d32(1.1, 1000),  # vd
            _d32(2.2, 1000),  # vq
            _u8(0b11),  # status: timeout|kill
        ]
    )
    out = P.parse_values(bytes((Command.GET_VALUES,)) + raw)
    assert out["temp_fet_c"] == pytest.approx(25.0, abs=0.05)
    assert out["motor_current_a"] == pytest.approx(1.5, abs=0.01)
    assert out["duty"] == pytest.approx(0.123, abs=1e-3)
    assert out["erpm"] == 1234
    assert out["input_voltage_v"] == pytest.approx(48.0, abs=0.05)
    assert out["fault_code"] == 0
    assert out["fault"] == "NONE"
    assert out["pid_position_deg"] == pytest.approx(12.5, abs=1e-3)
    assert out["controller_id"] == 42
    assert out["temp_mos1_c"] == pytest.approx(24.0, abs=0.05)
    assert out["temp_mos3_c"] == pytest.approx(24.2, abs=0.05)
    assert out["vd_v"] == pytest.approx(1.1, abs=1e-3)
    assert out["vq_v"] == pytest.approx(2.2, abs=1e-3)
    assert out["timeout_active"] is True
    assert out["kill_switch_active"] is True


def test_parse_values_optional_truncation_safe():
    # Firmware tua yang tidak mengirim mos1/2/3, vd/vq, status -> parser
    # berhenti aman tanpa error.
    raw = b"".join(
        [
            _d16(25.0, 10),
            _d16(26.0, 10),
            _d32(1.5, 100),
            _d32(2.0, 100),
            _d32(0.7, 100),
            _d32(0.8, 100),
            _d16(0.123, 1000),
            _i32(1234),
            _d16(48.0, 10),
            _d32(0.5, 10000),
            _d32(0.1, 10000),
            _d32(10.0, 10000),
            _d32(2.0, 10000),
            _i32(5000),
            _i32(9000),
            _u8(0),
            _d32(12.5, 1_000_000),
            _u8(42),
        ]
    )
    out = P.parse_values(bytes((Command.GET_VALUES,)) + raw)
    assert "temp_mos1_c" not in out
    assert "vd_v" not in out
    assert out["fault"] == "NONE"


def test_parse_values_fault_str():
    # Susun payload GET_VALUES penuh, lalu set fault_code = 3 (DRV).
    raw = (
        _d16(0, 10) + _d16(0, 10)
        + _d32(0, 100) + _d32(0, 100) + _d32(0, 100) + _d32(0, 100)
        + _d16(0, 1000) + _i32(0) + _d16(0, 10)
        + _d32(0, 10000) * 4
        + _i32(0) + _i32(0)
        + _u8(3)  # fault_code = DRV
        + _d32(0, 1_000_000) + _u8(0)
        + _d16(0, 10) * 3
        + _d32(0, 1000) * 2
        + _u8(0)
    )
    out = P.parse_values(bytes((Command.GET_VALUES,)) + raw)
    assert out["fault_code"] == 3
    assert out["fault"] == "DRV"


# ---------------------------------------------------------------------------
# GET_VALUES_SELECTIVE (cmd 50) — bit alignment kritis
# ---------------------------------------------------------------------------
def test_parse_values_selective_only_rpm_and_duty():
    # mask = bit7 (rpm) | bit6 (duty). Urutan C++ mengikuti BIT ascending:
    # duty(i16) dibaca lebih dulu, lalu rpm(i32).
    mask = ValueMask.RPM | ValueMask.DUTY_NOW
    raw = _u32(mask) + _d16(0.5, 1000) + _i32(3000)
    out = P.parse_values_selective(bytes((Command.GET_VALUES_SELECTIVE,)) + raw)
    assert out["mask"] == f"0x{mask:08X}"
    assert out["duty"] == pytest.approx(0.5, abs=1e-3)
    assert out["erpm"] == 3000
    assert "motor_current_a" not in out  # tidak dipilih
    assert "fault_code" not in out


def test_parse_values_selective_mos_trio_single_bit():
    # bit 18 mencakup 3 field i16 (mos1/2/3) sekaligus.
    mask = ValueMask.TEMP_MOS_1_2_3
    raw = _u32(mask) + _d16(20.0, 10) + _d16(21.0, 10) + _d16(22.0, 10)
    out = P.parse_values_selective(bytes((Command.GET_VALUES_SELECTIVE,)) + raw)
    assert out["temp_mos1_c"] == pytest.approx(20.0, abs=0.05)
    assert out["temp_mos2_c"] == pytest.approx(21.0, abs=0.05)
    assert out["temp_mos3_c"] == pytest.approx(22.0, abs=0.05)


def test_parse_values_selective_vd_vq_independent_bits():
    mask = ValueMask.VD | ValueMask.VQ
    raw = _u32(mask) + _d32(3.3, 1000) + _d32(4.4, 1000)
    out = P.parse_values_selective(bytes((Command.GET_VALUES_SELECTIVE,)) + raw)
    assert out["vd_v"] == pytest.approx(3.3, abs=1e-3)
    assert out["vq_v"] == pytest.approx(4.4, abs=1e-3)


def test_parse_values_selective_vd_only():
    mask = ValueMask.VD
    raw = _u32(mask) + _d32(3.3, 1000)
    out = P.parse_values_selective(bytes((Command.GET_VALUES_SELECTIVE,)) + raw)
    assert out["vd_v"] == pytest.approx(3.3, abs=1e-3)
    assert "vq_v" not in out


def test_parse_values_selective_vq_only():
    mask = ValueMask.VQ
    raw = _u32(mask) + _d32(4.4, 1000)
    out = P.parse_values_selective(bytes((Command.GET_VALUES_SELECTIVE,)) + raw)
    assert out["vq_v"] == pytest.approx(4.4, abs=1e-3)
    assert "vd_v" not in out


def test_parse_values_selective_status_bit():
    mask = ValueMask.STATUS
    raw = _u32(mask) + _u8(0b10)  # kill switch active
    out = P.parse_values_selective(bytes((Command.GET_VALUES_SELECTIVE,)) + raw)
    assert out["status"] == 0b10
    assert out["kill_switch_active"] is True
    assert out["timeout_active"] is False


def test_parse_values_selective_bad_bit_raises():
    # meminta field yang tidak ada di payload -> BufferUnderflow
    from vesc_protocol.buffer import BufferUnderflow

    mask = ValueMask.RPM
    raw = _u32(mask) + _i16(0)  # rpm butuh i32, hanya ada i16 -> terpotong
    with pytest.raises(BufferUnderflow):
        P.parse_values_selective(bytes((Command.GET_VALUES_SELECTIVE,)) + raw)


# ---------------------------------------------------------------------------
# GET_VALUES_SETUP (cmd 47 / 51)
# ---------------------------------------------------------------------------
def test_parse_setup_values_all():
    raw = b"".join(
        [
            _d16(25.0, 10),  # temp mos
            _d16(26.0, 10),  # temp motor
            _d32(1.5, 100),  # imotor
            _d32(2.0, 100),  # iin
            _d16(0.123, 1000),  # duty
            _i32(1234),  # rpm
            _d32(3.5, 1000),  # speed
            _d16(48.0, 10),  # vin
            _d16(0.8, 1000),  # battery level
            _d32(0.5, 10000),  # ah
            _d32(0.1, 10000),  # ahc
            _d32(10.0, 10000),  # wh
            _d32(2.0, 10000),  # whc
            _d32(5000, 1000),  # distance
            _d32(9000, 1000),  # distance abs
            _d32(12.5, 1_000_000),  # pid pos
            _u8(0),  # fault
            _u8(1),  # controller id
            _u8(2),  # vesc count
            _d32(55.0, 1000),  # battery wh
            _u32(12345),  # odometer
            _u32(6789),  # uptime
        ]
    )
    out = P.parse_setup_values(bytes((Command.GET_VALUES_SETUP,)) + raw)
    assert out["speed_m_s"] == pytest.approx(3.5, abs=1e-3)
    assert out["battery_level"] == pytest.approx(0.8, abs=1e-3)
    assert out["vesc_count"] == 2
    assert out["odometer_m"] == 12345
    assert out["uptime_ms"] == 6789
    assert out["fault"] == "NONE"


def test_parse_setup_values_selective():
    mask = SetupMask.SPEED | SetupMask.BATTERY_LEVEL
    raw = _u32(mask) + _d32(3.5, 1000) + _d16(0.8, 1000)
    out = P.parse_setup_values_selective(
        bytes((Command.GET_VALUES_SETUP_SELECTIVE,)) + raw
    )
    assert out["speed_m_s"] == pytest.approx(3.5, abs=1e-3)
    assert out["battery_level"] == pytest.approx(0.8, abs=1e-3)
    assert "rpm" not in out


# ---------------------------------------------------------------------------
# IMU (cmd 65)
# ---------------------------------------------------------------------------
def test_parse_imu_roll_pitch_only():
    mask = ImuMask.ROLL | ImuMask.PITCH
    # VESC double32auto: 1.0 -> 0x3F800000, -2.5 -> 0xC0200000
    raw = _u16(mask) + _u32(0x3F800000) + _u32(0xC0200000)
    out = P.parse_imu(bytes((Command.GET_IMU_DATA,)) + raw)
    assert out["mask"] == f"0x{mask:04X}"
    assert out["roll_rad"] == pytest.approx(1.0, abs=1e-6)
    assert out["pitch_rad"] == pytest.approx(-2.5, abs=1e-6)
    assert "yaw_rad" not in out


def test_parse_imu_quaternion_and_cid():
    mask = ImuMask.Q0 | ImuMask.Q1 | ImuMask.Q2 | ImuMask.Q3
    raw = _u16(mask)
    for f in (0.5, -0.5, 0.25, -0.25):
        raw += struct.pack(">f", f)
    raw += _u8(7)  # controller id
    out = P.parse_imu(bytes((Command.GET_IMU_DATA,)) + raw)
    assert out["q0"] == pytest.approx(0.5, abs=1e-6)
    assert out["q3"] == pytest.approx(-0.25, abs=1e-6)
    assert out["controller_id"] == 7


# ---------------------------------------------------------------------------
# Decoded inputs, rotor, ping_can, balance
# ---------------------------------------------------------------------------
def test_parse_ppm():
    raw = _d32(0.75, 1e6) + _d32(0.0015, 1e6)
    out = P.parse_ppm(bytes((Command.GET_DECODED_PPM,)) + raw)
    assert out["decoded_level"] == pytest.approx(0.75, abs=1e-6)
    assert out["pulse_length_s"] == pytest.approx(0.0015, abs=1e-9)


def test_parse_adc():
    raw = _d32(0.1, 1e6) + _d32(1.2, 1e6) + _d32(0.2, 1e6) + _d32(2.4, 1e6)
    out = P.parse_adc(bytes((Command.GET_DECODED_ADC,)) + raw)
    assert out["decoded_level_1"] == pytest.approx(0.1, abs=1e-6)
    assert out["voltage_1_v"] == pytest.approx(1.2, abs=1e-6)
    assert out["decoded_level_2"] == pytest.approx(0.2, abs=1e-6)
    assert out["voltage_2_v"] == pytest.approx(2.4, abs=1e-6)


def test_parse_chuk():
    out = P.parse_chuk(bytes((Command.GET_DECODED_CHUK,)) + _d32(-0.5, 1e6))
    assert out["decoded_y"] == pytest.approx(-0.5, abs=1e-6)


def test_parse_rotor_position():
    out = P.parse_rotor_position(
        bytes((Command.ROTOR_POSITION,)) + _d32(123.456, 100_000.0)
    )
    assert out == pytest.approx(123.456, abs=1e-4)


def test_parse_ping_can():
    reply = bytes((Command.PING_CAN, 1, 2, 10, 254))
    assert P.parse_ping_can(reply) == [1, 2, 10, 254]


def test_parse_ping_can_wrong_cmd():
    with pytest.raises(ValueError):
        P.parse_ping_can(bytes((Command.FW_VERSION,)))


def test_parse_firmware_empty_reply_no_underflow():
    # Balasan hanya command id (tanpa field) tidak boleh raise.
    out = P.parse_firmware(bytes((Command.FW_VERSION,)))
    assert out == {}


def test_parse_values_partial_no_position_or_id():
    # Firmware versi lama yang berhenti SEBELUM field pos/cid (bit16/17).
    # C++ guard vb.size()>=4 / >=1; Python harus skip tanpa BufferUnderflow.
    # Field wajib (bit0..15) lengkap, lalu berhenti.
    raw = b"".join(
        [
            _d16(25.0, 10),
            _d16(26.0, 10),
            _d32(1.5, 100),
            _d32(2.0, 100),
            _d32(0.7, 100),
            _d32(0.8, 100),
            _d16(0.123, 1000),
            _i32(1234),
            _d16(48.0, 10),
            _d32(0.5, 10000),
            _d32(0.1, 10000),
            _d32(10.0, 10000),
            _d32(2.0, 10000),
            _i32(5000),
            _i32(9000),
            _u8(0),  # fault NONE
        ]
    )
    out = P.parse_values(bytes((Command.GET_VALUES,)) + raw)
    assert "pid_position_deg" not in out
    assert "controller_id" not in out
    assert out["fault"] == "NONE"
    assert out["erpm"] == 1234


def test_parse_values_selective_partial_no_position_or_id():
    # Selective dengan bit16/17 SET tapi buffer pendek -> C++ skip (tidak raise).
    mask = ValueMask.RPM | ValueMask.POSITION | ValueMask.VESC_ID
    raw = _u32(mask) + _i32(3000)  # hanya rpm; pos/cid tidak ada
    out = P.parse_values_selective(bytes((Command.GET_VALUES_SELECTIVE,)) + raw)
    assert out["erpm"] == 3000
    assert "pid_position_deg" not in out
    assert "controller_id" not in out



def test_parse_unexpected_command_raises():
    with pytest.raises(ValueError):
        P.parse_values(bytes((Command.FW_VERSION, 0, 0)))
