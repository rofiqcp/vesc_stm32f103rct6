from .buffer import BufferUnderflow, Reader


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


def _start(reply: bytes, expected: int) -> Reader:
    if not reply or reply[0] != expected:
        got = None if not reply else reply[0]
        raise ValueError(f"command balasan {got}, seharusnya {expected}")
    return Reader(reply[1:])


def parse_firmware(reply: bytes) -> dict:
    r = _start(reply, 0)
    result = {
        "major": r.u8(),
        "minor": r.u8(),
        "hardware": r.c_string(),
    }
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
    if r.remaining:
        result["qml_flags"] = r.u8()
    if r.remaining:
        result["nrf_flags"] = r.u8()
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


VALUE_FIELDS = (
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
    ("pid_position_deg", "scaled32", 1_000_000.0),
    ("controller_id", "u8", None),
    ("temp_mos1_c", "scaled16", 10.0),
    ("temp_mos2_c", "scaled16", 10.0),
    ("temp_mos3_c", "scaled16", 10.0),
    ("vd_v", "scaled32", 1000.0),
    ("vq_v", "scaled32", 1000.0),
    ("status", "u8", None),
)


def _field_size(method: str) -> int:
    return 1 if method == "u8" else 2 if method in ("i16", "u16", "scaled16") else 4


def parse_values(reply: bytes) -> dict:
    r = _start(reply, 4)
    result = {}
    for name, method, scale in VALUE_FIELDS:
        if r.remaining < _field_size(method):
            break
        fn = getattr(r, method)
        result[name] = fn() if scale is None else fn(scale)
    if "fault_code" in result:
        result["fault"] = FAULTS.get(
            result["fault_code"], f"UNKNOWN_{result['fault_code']}"
        )
    if "status" in result:
        status = result["status"]
        result["timeout_active"] = bool(status & 0x01)
        result["kill_switch_active"] = bool(status & 0x02)
    if r.remaining:
        result["extra_hex"] = r.take(r.remaining).hex()
    return result


def parse_values_selective(reply: bytes) -> dict:
    r = _start(reply, 50)
    mask = r.u32()
    result = {"mask": f"0x{mask:08X}"}
    for bit, (name, method, scale) in enumerate(VALUE_FIELDS):
        if not (mask & (1 << bit)):
            continue
        if r.remaining < _field_size(method):
            raise BufferUnderflow(f"field bit {bit} ({name}) terpotong")
        fn = getattr(r, method)
        result[name] = fn() if scale is None else fn(scale)
    if "fault_code" in result:
        result["fault"] = FAULTS.get(
            result["fault_code"], f"UNKNOWN_{result['fault_code']}"
        )
    if "status" in result:
        status = result["status"]
        result["timeout_active"] = bool(status & 0x01)
        result["kill_switch_active"] = bool(status & 0x02)
    return result


def parse_rotor_position(reply: bytes) -> float:
    r = _start(reply, 22)
    return r.scaled32(100_000.0)


def parse_setup_values(reply: bytes) -> dict:
    r = _start(reply, 47)
    fields = (
        ("temp_fet_c", "scaled16", 10.0),
        ("temp_motor_c", "scaled16", 10.0),
        ("motor_current_a", "scaled32", 100.0),
        ("input_current_a", "scaled32", 100.0),
        ("duty", "scaled16", 1000.0),
        ("erpm", "i32", None),
        ("speed_m_s", "scaled32", 1000.0),
        ("input_voltage_v", "scaled16", 10.0),
        ("battery_level", "scaled16", 1000.0),
        ("amp_hours", "scaled32", 10000.0),
        ("amp_hours_charged", "scaled32", 10000.0),
        ("watt_hours", "scaled32", 10000.0),
        ("watt_hours_charged", "scaled32", 10000.0),
        ("distance_m", "scaled32", 1000.0),
        ("distance_abs_m", "scaled32", 1000.0),
        ("pid_position_deg", "scaled32", 1_000_000.0),
        ("fault_code", "u8", None),
        ("controller_id", "u8", None),
        ("vesc_count", "u8", None),
        ("battery_wh_left", "scaled32", 1000.0),
        ("odometer_m", "u32", None),
        ("uptime_ms", "u32", None),
    )
    result = {}
    for name, method, scale in fields:
        if r.remaining < _field_size(method):
            break
        fn = getattr(r, method)
        result[name] = fn() if scale is None else fn(scale)
    if "fault_code" in result:
        result["fault"] = FAULTS.get(
            result["fault_code"], f"UNKNOWN_{result['fault_code']}"
        )
    return result


def parse_ppm(reply: bytes) -> dict:
    r = _start(reply, 31)
    return {"decoded_level": r.scaled32(1e6), "pulse_length_s": r.scaled32(1e6)}


def parse_adc(reply: bytes) -> dict:
    r = _start(reply, 32)
    return {
        "decoded_level_1": r.scaled32(1e6),
        "voltage_1_v": r.scaled32(1e6),
        "decoded_level_2": r.scaled32(1e6),
        "voltage_2_v": r.scaled32(1e6),
    }


def parse_chuk(reply: bytes) -> dict:
    r = _start(reply, 33)
    return {"decoded_y": r.scaled32(1e6)}


IMU_NAMES = (
    "roll_rad", "pitch_rad", "yaw_rad",
    "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z",
    "mag_x", "mag_y", "mag_z",
    "q0", "q1", "q2", "q3",
)


def parse_imu(reply: bytes) -> dict:
    r = _start(reply, 65)
    mask = r.u16()
    result = {"mask": f"0x{mask:04X}"}
    for bit, name in enumerate(IMU_NAMES):
        if mask & (1 << bit):
            result[name] = r.auto32()
    if r.remaining:
        result["controller_id"] = r.u8()
    if r.remaining:
        result["extra_hex"] = r.take(r.remaining).hex()
    return result
