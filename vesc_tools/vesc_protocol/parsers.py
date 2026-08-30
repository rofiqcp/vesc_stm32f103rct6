"""Dekoder balasan VESC, persis urutan field menurut ``commands.cpp``.

Tiap field ``GET_VALUES`` / ``GET_VALUES_SETUP`` / ``GET_IMU_DATA`` dikodekan
sebagai tupel ``(nama, method, scale, bit)`` di mana ``bit`` adalah nomor bit
mask persis seperti di C++ (``mask & (1 << bit)``). Hal ini penting: beberapa
field berbagi satu bit (``temp_mos_1/2/3`` = bit 18, ``vd/vq`` = bit 19), jadi
mengindeks dengan ``enumerate()`` akan salah menyelaraskan field untuk mask
selective. Semua balasan diawali 1 byte command id; ``_start`` memverifikasi dan
membuangnya.
"""

from .buffer import BufferUnderflow, Reader
from .ids import Command, ImuMask, SetupMask, ValueMask


FAULTS = {
    0: "NONE",
    1: "OVER_VOLTAGE",
    2: "UNDER_VOLTAGE",
    3: "DRV",
    4: "ABS_OVER_CURRENT",
    5: "OVER_TEMP_FET",
    6: "OVER_TEMP_MOTOR",
    7: "GATE_DRIVER_OVER_VOLTAGE",
    8: "GATE_DRIVER_UNDER_VOLTAGE",
    9: "MCU_UNDER_VOLTAGE",
    10: "BOOTING_FROM_WATCHDOG_RESET",
    11: "ENCODER_SPI",
    12: "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
    13: "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE",
    14: "FLASH_CORRUPTION",
    15: "HIGH_OFFSET_CURRENT_SENSOR_1",
    16: "HIGH_OFFSET_CURRENT_SENSOR_2",
    17: "HIGH_OFFSET_CURRENT_SENSOR_3",
    18: "UNBALANCED_CURRENTS",
    19: "BRK",
    20: "RESOLVER_LOT",
    21: "RESOLVER_DOS",
    22: "RESOLVER_LOS",
    23: "FLASH_CORRUPTION_APP_CFG",
    24: "FLASH_CORRUPTION_MC_CFG",
    25: "ENCODER_NO_MAGNET",
    26: "ENCODER_MAGNET_TOO_STRONG",
    27: "PHASE_FILTER",
    28: "ENCODER_MAGNET_TOO_WEAK",
    29: "ENCODER_FAULT",
    30: "LORA",
    31: "ENCODER_SPI_2",
    32: "RESOLVER_LOS_PHASE_1",
    33: "RESOLVER_LOS_PHASE_2",
    34: "ABS_OVERSPEED",
}

# fault code 35..255 dianggap UNKNOWN_<n> saat didecode.


def _start(reply: bytes, expected: int) -> Reader:
    if not reply or reply[0] != expected:
        got = None if not reply else reply[0]
        raise ValueError(f"command balasan {got}, seharusnya {expected}")
    return Reader(reply[1:])


def _fault_str(code: int) -> str:
    return FAULTS.get(code, f"UNKNOWN_{code}")


def parse_firmware(reply: bytes) -> dict:
    """Balasan COMM_FW_VERSION (cmd 0).

    Urutan field persis ``case COMM_FW_VERSION`` di commands.cpp::

        major(i8) minor(i8) hw(cstring)
        uuid(12) pairing_done(i8) test_version(i8) hw_type(i8)
        custom_config_count(i8) phase_filters(i8)
        qml_hw(i8) qml_app(i8) nrf_flags(i8)
        fw_name(cstring) hw_conf_crc32(u32)
    """
    r = _start(reply, 0)
    # Persis C++: ``if (vb.size() >= 2)`` — balasan sangat pendek (mis. hanya
    # command id) tidak boleh mengakibatkan underflow.
    result: dict = {}
    if r.remaining >= 2:
        result["major"] = r.u8()
        result["minor"] = r.u8()
        result["hardware"] = r.c_string()
    if r.remaining >= 12:
        result["uuid"] = r.take(12).hex().upper()
    if r.remaining:
        result["pairing_done"] = bool(r.u8())
    if r.remaining:
        result["test_version"] = r.u8()
    if r.remaining:
        result["hardware_type"] = r.u8()
    if r.remaining:
        result["custom_config_count"] = r.u8()
    if r.remaining:
        result["phase_filters"] = bool(r.u8())
    if r.remaining >= 2:
        qml_hw = r.u8()
        qml_app = r.u8()
        result["qml_hw"] = qml_hw
        result["qml_hw_fullscreen"] = qml_hw == 2
        result["qml_app"] = qml_app
        result["qml_app_fullscreen"] = qml_app == 2
    if r.remaining:
        nrf = r.u8()
        result["nrf_name_supported"] = bool(nrf & 1)
        result["nrf_pin_supported"] = bool(nrf & 2)
    if r.remaining:
        try:
            result["firmware_name"] = r.c_string()
        except BufferUnderflow:
            result["extra_hex"] = r.take(r.remaining).hex()
            return result
    if r.remaining >= 4:
        result["firmware_crc32"] = f"0x{r.u32():08X}"
    if r.remaining:
        result["extra_hex"] = r.take(r.remaining).hex()
    return result


_VALUE_SIMPLE = (
    ("temp_fet_c", "scaled16", 10.0),
    ("temp_motor_c", "scaled16", 10.0),
    ("motor_current_a", "scaled32", 100.0),
    ("input_current_a", "scaled32", 100.0),
    ("id_current_a", "scaled32", 100.0),
    ("iq_current_a", "scaled32", 100.0),
    ("duty", "scaled16", 1000.0),
    ("erpm", "i32", None),
    ("input_voltage_v", "scaled16", 10.0),
    ("amp_hours", "scaled32", 10000.0),
    ("amp_hours_charged", "scaled32", 10000.0),
    ("watt_hours", "scaled32", 10000.0),
    ("watt_hours_charged", "scaled32", 10000.0),
    ("tachometer", "i32", None),
    ("tachometer_abs", "i32", None),
    ("fault_code", "u8", None),
)


def _read(result: dict, r: Reader, name, method, scale) -> None:
    fn = getattr(r, method)
    result[name] = fn() if scale is None else fn(scale)


def parse_values(reply: bytes) -> dict:
    """Balasan COMM_GET_VALUES (cmd 4). Field yang ada selalu didekode;
    field tambahan (mos1/2/3, vd/vq, status, position, vesc_id) bersifat
    opsional dan hanya dibaca bila cukup byte tersisa -- sama seperti C++
    (guard ``vb.size() >=``). Khususnya position (bit16) dan vesc_id (bit17)
    di C++ diguard ``vb.size() >= 4`` dan ``>= 1``; firmware lama yang tidak
    mengirim field itu tidak boleh memicu BufferUnderflow.
    """
    r = _start(reply, 4)
    result: dict = {}
    for name, method, scale in _VALUE_SIMPLE:
        _read(result, r, name, method, scale)
    if "fault_code" in result:
        result["fault"] = _fault_str(result["fault_code"])
    # Bit 16 (POSITION): i32, hanya bila >= 4 byte (sesuai C++).
    if r.remaining >= 4:
        result["pid_position_deg"] = r.scaled32(1_000_000.0)
    # Bit 17 (VESC_ID): u8, hanya bila >= 1 byte (sesuai C++).
    if r.remaining >= 1:
        result["controller_id"] = r.u8()
    # Bit 18 (TEMP_MOS_1_2_3): 3 field i16, hanya bila >= 6 byte.
    if r.remaining >= 6:
        result["temp_mos1_c"] = r.scaled16(10.0)
        result["temp_mos2_c"] = r.scaled16(10.0)
        result["temp_mos3_c"] = r.scaled16(10.0)
    # Full GET_VALUES appends both bit-19 Vd and bit-20 Vq in order.
    if r.remaining >= 8:
        result["vd_v"] = r.scaled32(1000.0)
        result["vq_v"] = r.scaled32(1000.0)
    # Bit 20 (STATUS): 1 byte.
    if r.remaining >= 1:
        status = r.u8()
        result["status"] = status
        result["timeout_active"] = bool(status & 0x01)
        result["kill_switch_active"] = bool(status & 0x02)
    if r.remaining:
        result["extra_hex"] = r.take(r.remaining).hex()
    return result


def parse_values_selective(reply: bytes) -> dict:
    """Balasan COMM_GET_VALUES_SELECTIVE (cmd 50). Byte 1..4 = mask u32.

    Field dipilih oleh mask; ``temp_mos_1/2/3`` berbagi bit 18 dan ``vd/vq``
    berbagi bit 19. Bila field terpilih tapi terpotong -> BufferUnderflow
    (sesuai C++ yang meng-assert ukuran).

    Khusus bit 16 (POSITION) dan bit 17 (VESC_ID): di C++ diguard ``vb.size()>=4``
    dan ``>=1`` dan **tidak** memicu underflow bila pendek -- cukup dilewati
    (position=-1, vesc_id=255). Maka dua field ini ditangani terpisah di bawah,
    di luar loop utama.
    """
    r = _start(reply, 50)
    mask = r.u32()
    result = {"mask": f"0x{mask:08X}"}
    for name, method, scale in _VALUE_SIMPLE:
        bit = _VALUE_BITS[name]
        if bit in (ValueMask.POSITION, ValueMask.VESC_ID):
            continue  # ditangani terpisah dengan guard seperti C++
        if not (mask & bit):
            continue
        if r.remaining < _field_size(method):
            raise BufferUnderflow(f"field '{name}' (bit {bit}) terpotong")
        _read(result, r, name, method, scale)
    if "fault_code" in result:
        result["fault"] = _fault_str(result["fault_code"])
    # Bit 16 (POSITION): i32, hanya bila >= 4 byte (sesuai C++).
    if mask & ValueMask.POSITION and r.remaining >= 4:
        result["pid_position_deg"] = r.scaled32(1_000_000.0)
    # Bit 17 (VESC_ID): u8, hanya bila >= 1 byte (sesuai C++).
    if mask & ValueMask.VESC_ID and r.remaining >= 1:
        result["controller_id"] = r.u8()
    if mask & ValueMask.TEMP_MOS_1_2_3:
        if r.remaining < 6:
            raise BufferUnderflow("temp_mos1/2/3 terpotong")
        result["temp_mos1_c"] = r.scaled16(10.0)
        result["temp_mos2_c"] = r.scaled16(10.0)
        result["temp_mos3_c"] = r.scaled16(10.0)
    if mask & ValueMask.VD:
        if r.remaining < 4:
            raise BufferUnderflow("vd terpotong")
        result["vd_v"] = r.scaled32(1000.0)
    if mask & ValueMask.VQ:
        if r.remaining < 4:
            raise BufferUnderflow("vq terpotong")
        result["vq_v"] = r.scaled32(1000.0)
    if mask & ValueMask.STATUS:
        if r.remaining < 1:
            raise BufferUnderflow("status terpotong")
        status = r.u8()
        result["status"] = status
        result["timeout_active"] = bool(status & 0x01)
        result["kill_switch_active"] = bool(status & 0x02)
    if r.remaining:
        result["extra_hex"] = r.take(r.remaining).hex()
    return result


_VALUE_BITS = {
    "temp_fet_c": ValueMask.TEMP_MOS,
    "temp_motor_c": ValueMask.TEMP_MOTOR,
    "motor_current_a": ValueMask.CURRENT_MOTOR,
    "input_current_a": ValueMask.CURRENT_IN,
    "id_current_a": ValueMask.ID,
    "iq_current_a": ValueMask.IQ,
    "duty": ValueMask.DUTY_NOW,
    "erpm": ValueMask.RPM,
    "input_voltage_v": ValueMask.V_IN,
    "amp_hours": ValueMask.AMP_HOURS,
    "amp_hours_charged": ValueMask.AMP_HOURS_CHARGED,
    "watt_hours": ValueMask.WATT_HOURS,
    "watt_hours_charged": ValueMask.WATT_HOURS_CHARGED,
    "tachometer": ValueMask.TACHOMETER,
    "tachometer_abs": ValueMask.TACHOMETER_ABS,
    "fault_code": ValueMask.FAULT_CODE,
}


def parse_rotor_position(reply: bytes) -> float:
    """Balasan COMM_ROTOR_POSITION (cmd 22): double32(1e5)."""
    r = _start(reply, 22)
    return r.scaled32(100_000.0)


def _field_size(method: str) -> int:
    return 1 if method == "u8" else 2 if method in ("i16", "u16", "scaled16") else 4


def _decode_masked(r: Reader, mask: int, fields, result: dict) -> None:
    for name, method, scale, bit in fields:
        if not (mask & bit):
            continue
        if r.remaining < _field_size(method):
            raise BufferUnderflow(f"field '{name}' (bit {bit}) terpotong")
        _read(result, r, name, method, scale)
    if "fault_code" in result:
        result["fault"] = _fault_str(result["fault_code"])
    if "status" in result:
        status = result["status"]
        result["timeout_active"] = bool(status & 0x01)
        result["kill_switch_active"] = bool(status & 0x02)



_SETUP_SIMPLE = (
    ("temp_fet_c", "scaled16", 10.0, SetupMask.TEMP_MOS),
    ("temp_motor_c", "scaled16", 10.0, SetupMask.TEMP_MOTOR),
    ("motor_current_a", "scaled32", 100.0, SetupMask.CURRENT_MOTOR),
    ("input_current_a", "scaled32", 100.0, SetupMask.CURRENT_IN),
    ("duty", "scaled16", 1000.0, SetupMask.DUTY_NOW),
    ("erpm", "i32", None, SetupMask.RPM),
    ("speed_m_s", "scaled32", 1000.0, SetupMask.SPEED),
    ("input_voltage_v", "scaled16", 10.0, SetupMask.V_IN),
    ("battery_level", "scaled16", 1000.0, SetupMask.BATTERY_LEVEL),
    ("amp_hours", "scaled32", 10000.0, SetupMask.AMP_HOURS),
    ("amp_hours_charged", "scaled32", 10000.0, SetupMask.AMP_HOURS_CHARGED),
    ("watt_hours", "scaled32", 10000.0, SetupMask.WATT_HOURS),
    ("watt_hours_charged", "scaled32", 10000.0, SetupMask.WATT_HOURS_CHARGED),
    ("distance_m", "scaled32", 1000.0, SetupMask.TACHOMETER),
    ("distance_abs_m", "scaled32", 1000.0, SetupMask.TACHOMETER_ABS),
    ("pid_position_deg", "scaled32", 1_000_000.0, SetupMask.POSITION),
    ("fault_code", "u8", None, SetupMask.FAULT_CODE),
    ("controller_id", "u8", None, SetupMask.VESC_ID),
)


def parse_setup_values(reply: bytes) -> dict:
    """Balasan COMM_GET_VALUES_SETUP (cmd 47).

    Field wajib didekode; field tambahan hanya dibaca bila cukup byte
    (guard ``vb.size() >=`` seperti C++): NUM_VESCS(>=1), BATTERY_WH(>=4),
    ODOMETER(>=4), UPTIME_MS(>=4).
    """
    r = _start(reply, 47)
    result: dict = {}
    for name, method, scale, _bit in _SETUP_SIMPLE:
        _read(result, r, name, method, scale)
    if "fault_code" in result:
        result["fault"] = _fault_str(result["fault_code"])
    if r.remaining >= 1:
        result["vesc_count"] = r.u8()
    if r.remaining >= 4:
        result["battery_wh_left"] = r.scaled32(1000.0)
    if r.remaining >= 4:
        result["odometer_m"] = r.u32()
    if r.remaining >= 4:
        result["uptime_ms"] = r.u32()
    if r.remaining:
        result["extra_hex"] = r.take(r.remaining).hex()
    return result


def parse_setup_values_selective(reply: bytes) -> dict:
    """Balasan COMM_GET_VALUES_SETUP_SELECTIVE (cmd 51). Byte 1..4 = mask u32."""
    r = _start(reply, 51)
    mask = r.u32()
    result: dict = {}
    for name, method, scale, bit in _SETUP_SIMPLE:
        if not (mask & bit):
            continue
        if r.remaining < _field_size(method):
            raise BufferUnderflow(f"field '{name}' (bit {bit}) terpotong")
        _read(result, r, name, method, scale)
    if "fault_code" in result:
        result["fault"] = _fault_str(result["fault_code"])
    if mask & SetupMask.NUM_VESCS:
        if r.remaining < 1:
            raise BufferUnderflow("num_vescs terpotong")
        result["vesc_count"] = r.u8()
    if mask & SetupMask.BATTERY_WH:
        if r.remaining < 4:
            raise BufferUnderflow("battery_wh terpotong")
        result["battery_wh_left"] = r.scaled32(1000.0)
    if mask & SetupMask.ODOMETER:
        if r.remaining < 4:
            raise BufferUnderflow("odometer terpotong")
        result["odometer_m"] = r.u32()
    if mask & SetupMask.UPTIME_MS:
        if r.remaining < 4:
            raise BufferUnderflow("uptime_ms terpotong")
        result["uptime_ms"] = r.u32()
    if r.remaining:
        result["extra_hex"] = r.take(r.remaining).hex()
    return result


def parse_ppm(reply: bytes) -> dict:
    """COMM_GET_DECODED_PPM (cmd 31): dua double32(1e6)."""
    r = _start(reply, 31)
    return {"decoded_level": r.scaled32(1e6), "pulse_length_s": r.scaled32(1e6)}


def parse_adc(reply: bytes) -> dict:
    """COMM_GET_DECODED_ADC (cmd 32): empat double32(1e6)."""
    r = _start(reply, 32)
    return {
        "decoded_level_1": r.scaled32(1e6),
        "voltage_1_v": r.scaled32(1e6),
        "decoded_level_2": r.scaled32(1e6),
        "voltage_2_v": r.scaled32(1e6),
    }


def parse_chuk(reply: bytes) -> dict:
    """COMM_GET_DECODED_CHUK (cmd 33): satu double32(1e6)."""
    r = _start(reply, 33)
    return {"decoded_y": r.scaled32(1e6)}


def parse_balance(reply: bytes) -> dict:
    """COMM_GET_DECODED_BALANCE (cmd 79): field skala menurut commands.cpp."""
    r = _start(reply, 79)
    # Urutan persis case COMM_GET_DECODED_BALANCE di commands.cpp (double32 1e6).
    names = (
        "pid_output", "pid_setpoint", "current_filter", "fault_code_now",
        "fault_code_rec", "balance_conf", "balance_now", "v_in",
        "enc_angle_now", "enc_angle_set", "state", "switch_state",
        "adc1", "adc2", "current2", "current3",
    )
    result = {}
    for name in names:
        if r.remaining < 4:
            break
        result[name] = r.scaled32(1e6)
    if r.remaining:
        result["extra_hex"] = r.take(r.remaining).hex()
    return result


_IMU_NAMES = (
    "roll_rad", "pitch_rad", "yaw_rad",
    "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z",
    "mag_x", "mag_y", "mag_z",
    "q0", "q1", "q2", "q3",
)


def parse_imu(reply: bytes) -> dict:
    """COMM_GET_IMU_DATA (cmd 65). Byte 1..2 = mask u16, lalu double32auto."""
    r = _start(reply, 65)
    mask = r.u16()
    result = {"mask": f"0x{mask:04X}"}
    for bit, name in enumerate(_IMU_NAMES):
        if mask & (1 << bit):
            result[name] = r.auto32()
    if r.remaining:
        result["controller_id"] = r.u8()
    if r.remaining:
        result["extra_hex"] = r.take(r.remaining).hex()
    return result


def parse_ping_can(reply: bytes) -> list[int]:
    """COMM_PING_CAN (cmd 62): daftar byte ID (tanpa command id depan)."""
    if not reply or reply[0] != Command.PING_CAN:
        got = None if not reply else reply[0]
        raise ValueError(f"command balasan {got}, seharusnya {Command.PING_CAN}")
    return list(reply[1:])
