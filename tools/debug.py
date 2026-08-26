#!/usr/bin/env python3
"""Debug/commissioning tool for STM32F103 dual-FOC firmware.

Only pyserial is required for live use:
    python3 -m pip install pyserial

Passive tests never command PWM. Motor-moving tests require --yes explicitly.
"""
from __future__ import annotations

import argparse
import contextlib
import csv
from datetime import datetime
import math
import struct
import sys
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

try:
    import serial  # type: ignore
except Exception:
    serial = None

COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM = 8
COMM_SET_POS = 9
COMM_SET_HANDBRAKE = 10
COMM_SET_DETECT = 11
COMM_SET_MCCONF = 13
COMM_GET_MCCONF = 14
COMM_GET_MCCONF_DEFAULT = 15
COMM_SET_APPCONF = 16
COMM_GET_APPCONF = 17
COMM_GET_APPCONF_DEFAULT = 18
COMM_SAMPLE_PRINT = 19
COMM_ROTOR_POSITION = 22
COMM_DETECT_MOTOR_R_L = 25
COMM_DETECT_MOTOR_FLUX_LINKAGE = 26
COMM_DETECT_ENCODER = 27
COMM_DETECT_HALL_FOC = 28
COMM_ALIVE = 30
COMM_GET_DECODED_ADC = 31
COMM_FORWARD_CAN = 34
COMM_CUSTOM_APP_DATA = 36
COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP = 57
COMM_DETECT_APPLY_ALL_FOC = 58
COMM_PING_CAN = 62
COMM_SET_CURRENT_REL = 84

VESC6_MCCONF_WIRE_SIZE = 481
VESC6_APPCONF_WIRE_SIZE = 493
VESC6_MCCONF_SIGNATURE = 776184161
VESC6_APPCONF_SIGNATURE = 486554156

CUSTOM_SELECT_MOTOR = 0xA0
CUSTOM_DUAL_SUMMARY = 0xA1
CUSTOM_CLEAR_FAULT = 0xA2
CUSTOM_STOP = 0xA3
CUSTOM_SENSOR_SELECT = 0xA4
CUSTOM_SENSOR_DETECT = 0xA5
CUSTOM_CURRENT_CAL = 0xA6
CUSTOM_SAMPLE_START = 0xA7
CUSTOM_EXT_TELEMETRY = 0xA8
CUSTOM_SENSOR_INFO = 0xA9
CUSTOM_COMM_DIAG = 0xAA
CUSTOM_CONFIG_SAVE = 0xAB
CUSTOM_CONFIG_STATUS = 0xAC

SENSOR_AUTO = 0
SENSOR_HALL = 1
SENSOR_ENCODER = 2

SENSOR_NAMES = {0: "AUTO", 1: "HALL", 2: "ENCODER", 3: "FORCED"}
DETECT_NAMES = {
    0: "IDLE", 1: "PREPARE", 2: "HALL_LOCK", 3: "HALL_FWD",
    4: "HALL_REV", 5: "HALL_EVAL", 6: "ENC_PREP", 7: "ENC_LOCK0",
    8: "ENC_SWEEP", 9: "ENC_EVAL", 10: "ENC_RETURN0", 11: "ENC_ALIGN",
    12: "DONE", 13: "FAILED",
}
DETECT_DONE = 12
DETECT_FAILED = 13
DISPLAY_MODES = {
    "stop": 0, "inductance": 1, "observer": 2, "encoder": 3,
    "pid": 4, "pid-error": 5, "obs-enc": 6, "obs-hall": 7,
}

FAULT_NAMES = {
    0: "NONE", 1: "OVER_VOLTAGE", 2: "UNDER_VOLTAGE", 3: "DRV/HW",
    4: "ABS_OVER_CURRENT", 15: "CURRENT_OFFSET", 27: "SENSOR/ENCODER",
}
# Port-native MotorRuntime enum used only by CUSTOM_EXT_TELEMETRY.
NATIVE_FAULT_NAMES = {
    0: "NONE", 1: "ADC_DMA", 2: "ABS_OVER_CURRENT", 3: "OVER_VOLTAGE",
    4: "UNDER_VOLTAGE", 5: "HALL_INVALID", 6: "FOC_ISR_OVERRUN",
    7: "COMMAND_TIMEOUT", 8: "CURRENT_OFFSET", 9: "SENSOR_DETECT",
}


def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def frame(payload: bytes) -> bytes:
    if not payload:
        raise ValueError("empty payload")
    if len(payload) <= 255:
        head = bytes((2, len(payload)))
    elif len(payload) <= 65535:
        head = bytes((3, (len(payload) >> 8) & 0xFF, len(payload) & 0xFF))
    else:
        raise ValueError("payload too long")
    c = crc16(payload)
    return head + payload + bytes((c >> 8, c & 0xFF, 3))


class FrameParser:
    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        self.buf.extend(data)
        out: list[bytes] = []
        while True:
            while self.buf and self.buf[0] not in (2, 3):
                del self.buf[0]
            if len(self.buf) < 2:
                break
            if self.buf[0] == 2:
                plen = self.buf[1]
                start = 2
                total = plen + 5
            else:
                if len(self.buf) < 3:
                    break
                plen = (self.buf[1] << 8) | self.buf[2]
                start = 3
                total = plen + 6
            if plen == 0:
                del self.buf[0]
                continue
            if len(self.buf) < total:
                break
            packet = bytes(self.buf[:total])
            del self.buf[:total]
            payload = packet[start:start + plen]
            crc_rx = (packet[start + plen] << 8) | packet[start + plen + 1]
            if packet[-1] == 3 and crc_rx == crc16(payload):
                out.append(payload)
        return out


def be_i16(v: int) -> bytes:
    return struct.pack(">h", int(v))


def be_u16(v: int) -> bytes:
    return struct.pack(">H", int(v) & 0xFFFF)


def be_i32(v: int) -> bytes:
    return struct.pack(">i", int(v))


class Reader:
    def __init__(self, data: bytes, offset: int = 0):
        self.data = data
        self.i = offset

    def _take(self, n: int) -> bytes:
        if self.i + n > len(self.data):
            raise ValueError(f"short payload at {self.i}, need {n}, len={len(self.data)}")
        b = self.data[self.i:self.i+n]
        self.i += n
        return b

    def u8(self) -> int: return self._take(1)[0]
    def i16(self) -> int: return struct.unpack(">h", self._take(2))[0]
    def u16(self) -> int: return struct.unpack(">H", self._take(2))[0]
    def i32(self) -> int: return struct.unpack(">i", self._take(4))[0]
    def u32(self) -> int: return struct.unpack(">I", self._take(4))[0]
    def f32_auto(self) -> float: return struct.unpack(">f", self._take(4))[0]

    def cstring(self) -> str:
        end = self.data.find(b"\x00", self.i)
        if end < 0:
            raise ValueError("string firmware tidak terminated NUL")
        raw = self.data[self.i:end]
        self.i = end + 1
        return raw.decode("ascii", errors="replace")


def parse_fw_version(p: bytes) -> dict:
    if len(p) < 3 or p[0] != COMM_FW_VERSION:
        raise ValueError("bukan COMM_FW_VERSION")
    r = Reader(p, 1)
    d = {"major": r.u8(), "minor": r.u8()}
    d["hw_name"] = r.cstring()
    d["uuid"] = r._take(12).hex()
    d["pairing_done"] = r.u8() if r.i < len(p) else None
    d["test_version"] = r.u8() if r.i < len(p) else None
    d["hw_type"] = r.u8() if r.i < len(p) else None
    d["custom_config_num"] = r.u8() if r.i < len(p) else None
    d["phase_filters"] = r.u8() if r.i < len(p) else None
    d["qml_hw"] = r.u8() if r.i < len(p) else None
    d["qml_app"] = r.u8() if r.i < len(p) else None
    d["nrf_flags"] = r.u8() if r.i < len(p) else None
    d["fw_name"] = r.cstring() if r.i < len(p) else ""
    d["hw_crc"] = r.u32() if len(p) - r.i >= 4 else None
    d["remaining"] = len(p) - r.i
    return d


class Link:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.05):
        if serial is None:
            raise RuntimeError("pyserial belum terpasang. Jalankan: python3 -m pip install pyserial")
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        self.parser = FrameParser()
        self.pending: list[bytes] = []
        self.ser.reset_input_buffer()

    def close(self) -> None:
        self.ser.close()

    def send(self, payload: bytes) -> None:
        self.ser.write(frame(payload))
        self.ser.flush()

    def recv(self, timeout: float = 1.0, pred: Optional[Callable[[bytes], bool]] = None) -> bytes:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for idx, p in enumerate(self.pending):
                if pred is None or pred(p):
                    return self.pending.pop(idx)
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                for p in self.parser.feed(data):
                    if pred is None or pred(p):
                        return p
                    self.pending.append(p)
        raise TimeoutError("timeout menunggu frame")

    def request(self, payload: bytes, pred: Callable[[bytes], bool], timeout: float = 1.0) -> bytes:
        self.send(payload)
        return self.recv(timeout=timeout, pred=pred)

    @staticmethod
    def standard_route(motor: int, payload: bytes) -> bytes:
        if motor == 0:
            return payload
        if motor == 1:
            return bytes((COMM_FORWARD_CAN, 2)) + payload
        raise ValueError(f"motor invalid: {motor}")

    def send_std(self, motor: int, payload: bytes) -> None:
        self.send(self.standard_route(motor, payload))

    def request_std(self, motor: int, payload: bytes, pred: Callable[[bytes], bool], timeout: float = 1.0) -> bytes:
        # Upstream dual-motor COMM_FORWARD_CAN replies with the inner command
        # directly, without wrapping the reply in COMM_FORWARD_CAN.
        return self.request(self.standard_route(motor, payload), pred, timeout)

    def select_motor(self, motor: int) -> None:
        # Legacy F103 diagnostic only. It does NOT alter standard UART routing.
        self.send(bytes((COMM_CUSTOM_APP_DATA, CUSTOM_SELECT_MOTOR, motor)))
        time.sleep(0.02)

    def stop(self, motor: int) -> None:
        self.send(bytes((COMM_CUSTOM_APP_DATA, CUSTOM_STOP, motor)))

    def clear_fault(self, motor: int) -> None:
        self.send(bytes((COMM_CUSTOM_APP_DATA, CUSTOM_CLEAR_FAULT, motor)))


@dataclass
class Values:
    temp_fet: float
    temp_motor: float
    imotor: float
    ibatt: float
    id: float
    iq: float
    duty: float
    erpm: int
    vin: float
    ah: float
    ah_charged: float
    wh: float
    wh_charged: float
    tach: int
    tach_abs: int
    fault: int
    position: float
    controller_id: int
    vd: float
    vq: float
    timeout_status: int


def parse_values(p: bytes) -> Values:
    if not p or p[0] != COMM_GET_VALUES:
        raise ValueError("bukan COMM_GET_VALUES")
    r = Reader(p, 1)
    temp_fet = r.i16()/10
    temp_motor = r.i16()/10
    imotor = r.i32()/100
    ibatt = r.i32()/100
    id_ = r.i32()/100
    iq = r.i32()/100
    duty = r.i16()/1000
    erpm = r.i32()
    vin = r.i16()/10
    ah = r.i32()/10000
    ahc = r.i32()/10000
    wh = r.i32()/10000
    whc = r.i32()/10000
    tach = r.i32(); tach_abs = r.i32(); fault = r.u8()
    pos = r.i32()/1_000_000
    cid = r.u8()
    # three MOS temperatures (present in current VESC GET_VALUES mask)
    _ = r.i16(); _ = r.i16(); _ = r.i16()
    vd = r.i32()/1000
    vq = r.i32()/1000
    status = r.u8()
    return Values(temp_fet,temp_motor,imotor,ibatt,id_,iq,duty,erpm,vin,ah,ahc,wh,whc,tach,tach_abs,fault,pos,cid,vd,vq,status)


def get_values(link: Link, motor: int, timeout: float = 1.0) -> Values:
    p = link.request_std(motor, bytes((COMM_GET_VALUES,)),
                         lambda x: bool(x) and x[0] == COMM_GET_VALUES,
                         timeout)
    return parse_values(p)


def parse_decoded_adc(p: bytes) -> dict:
    if len(p) != 17 or p[0] != COMM_GET_DECODED_ADC:
        raise ValueError("bukan COMM_GET_DECODED_ADC")
    r = Reader(p, 1)
    return {
        "decoded1": r.i32() / 1_000_000.0,
        "voltage1": r.i32() / 1_000_000.0,
        "decoded2": r.i32() / 1_000_000.0,
        "voltage2": r.i32() / 1_000_000.0,
    }


def get_decoded_adc(link: Link, motor: int, timeout: float = 1.0) -> dict:
    p = link.request_std(
        motor,
        bytes((COMM_GET_DECODED_ADC,)),
        lambda x: len(x) == 17 and x[0] == COMM_GET_DECODED_ADC,
        timeout,
    )
    return parse_decoded_adc(p)


def parse_cal(p: bytes) -> dict:
    if len(p) < 2 or p[:2] != bytes((COMM_CUSTOM_APP_DATA, CUSTOM_CURRENT_CAL)):
        raise ValueError("bukan current-cal reply")
    r=Reader(p,2)
    d = {
        "done": bool(r.u8()), "valid": bool(r.u8()), "count": r.u32(), "target": r.u32(),
        "left_u": r.i32(), "left_v": r.i32(), "left_dc": r.i32(),
        "right_u": r.i32(), "right_v": r.i32(), "right_dc": r.i32(),
    }
    if len(p) - r.i >= 12:
        d["adc_isr_count"] = r.u32()
        d["dma_cndtr"] = r.u16()
        d["tim2_cnt"] = r.u16()
        d["tim1_dir"] = r.u8()
        d["tim1_running"] = bool(r.u8())
        d["tim8_running"] = bool(r.u8())
        d["tim2_running"] = bool(r.u8())
    if len(p) - r.i >= 3:
        d["adc1_enabled"] = bool(r.u8())
        d["adc2_enabled"] = bool(r.u8())
        d["dma1_ch1_enabled"] = bool(r.u8())
    # V12 detailed calibration diagnostics, appended after all legacy fields.
    if len(p) - r.i >= 7:
        d["cal_diag_revision"] = r.u8()
        d["warn_mask"] = r.u16()
        d["fail_range_mask"] = r.u16()
        d["fail_noise_mask"] = r.u16()
        names=["left_u","left_v","left_dc","right_u","right_v","right_dc"]
        channels={}
        for name in names:
            if len(p) - r.i < 12: break
            mean=r.i32(); mn=r.u16(); mx=r.u16(); var_x100=r.u32()
            channels[name]={"mean":mean,"min":mn,"max":mx,"spread":mx-mn,
                            "variance":var_x100/100.0,"stddev":math.sqrt(max(0.0,var_x100/100.0))}
        d["channels"] = channels
        reg_names=[
            "rcc_cfgr",
            "adc1_cr1","adc1_cr2","adc1_sqr1","adc1_sqr3",
            "adc2_cr1","adc2_cr2","adc2_sqr1","adc2_sqr3",
            "dma1_ch1_ccr","dma1_ch1_cndtr32","dma1_isr",
            "tim1_cr1","tim1_arr","tim1_cnt","tim1_bdtr",
            "tim8_cr1","tim8_arr","tim8_cnt","tim8_bdtr",
            "tim2_cr1","tim2_smcr","tim2_ccr2","tim2_cnt32",
        ]
        regs={}
        for name in reg_names:
            if len(p)-r.i < 4: break
            regs[name]=r.u32()
        d["registers"] = regs
        dma_words=[]
        for _ in range(6):
            if len(p)-r.i < 4: break
            w=r.u32(); dma_words.append({"word":w,"adc1":w & 0xFFFF,"adc2":(w>>16)&0xFFFF})
        if dma_words: d["dma_words"] = dma_words

        # V14 appended fields: driven/undriven offset split and first fault snapshot.
        if d.get("cal_diag_revision",0) >= 14 and len(p)-r.i >= 51:
            d["cal_stage"] = r.u8()
            d["shift_warn_mask"] = r.u16()
            names=["left_u","left_v","left_dc","right_u","right_v","right_dc"]
            d["undriven_mean"] = {name:r.i32() for name in names}
            d["driven_mean"] = {name:r.i32() for name in names}
            if len(p)-r.i >= 60:
                fs={}
                fs["valid"]=bool(r.u8()); fs["motor"]=r.u8(); fs["fault"]=r.u8(); fs["cal_stage"]=r.u8()
                fs["raw_u"]=r.u16(); fs["raw_v"]=r.u16(); fs["raw_dc"]=r.u16()
                fs["offset_u"]=r.i32(); fs["offset_v"]=r.i32(); fs["offset_dc"]=r.i32()
                fs["ia_q15"]=r.i32(); fs["ib_q15"]=r.i32(); fs["ic_q15"]=r.i32()
                fs["trip_q15"]=r.i32(); fs["id_target_q15"]=r.i32(); fs["iq_target_q15"]=r.i32()
                fs["ccr1"]=r.u16(); fs["ccr2"]=r.u16(); fs["ccr3"]=r.u16()
                fs["tim_cnt"]=r.u16(); fs["dma_cndtr"]=r.u16(); fs["adc_isr_count"]=r.u32()
                # Q15 current base in this firmware is 64 A.
                for qn in ("ia_q15","ib_q15","ic_q15","trip_q15","id_target_q15","iq_target_q15"):
                    fs[qn.replace("_q15","_a")] = fs[qn] * 64.0 / 32768.0
                d["fault_snapshot"] = fs

            # V15 appends synchronized-enable state and the immutable ADC/PWM schedule.
            if d.get("cal_diag_revision",0) >= 15 and len(p)-r.i >= 22:
                fs=d.setdefault("fault_snapshot",{})
                fs["blank_cycles"]=r.u16(); fs["pwm_enabled"]=bool(r.u8()); fs["moe"]=bool(r.u8())
                fs["pending_events"]=r.u8(); fs["reserved"]=r.u8()
                d["adc_motor_phase_offset_ticks"]=r.u16()
                d["foc_isr_event_hz"]=r.u32(); d["foc_isr_slot_cycles"]=r.u32()
                d["tim8_rcr"]=r.u16(); d["tim1_cnt_v15"]=r.u16(); d["tim8_cnt_v15"]=r.u16()

            # V16 appends the independent ADC3/DMA2 DCLINK acquisition state.
            if d.get("cal_diag_revision",0) >= 16 and len(p)-r.i >= 42:
                d["adc3_enabled"] = bool(r.u8())
                d["dma2_ch5_enabled"] = bool(r.u8())
                d["dma2_ch5_cndtr"] = r.u16()
                d["adc3_vbus_raw0"] = r.u16(); d["adc3_vbus_raw1"] = r.u16()
                d["vbus_dma_stale_count"] = r.u8(); _ = r.u8()
                d["vbus_dma_stale_events"] = r.u32()
                for name in ("adc3_cr1","adc3_cr2","adc3_sqr1","adc3_sqr3",
                             "dma2_ch5_ccr","dma2_ch5_cndtr32","dma2_isr"):
                    d.setdefault("registers",{})[name] = r.u32()

            # V17 appends robust-filter outlier counts and per-motor MOE trace.
            if d.get("cal_diag_revision",0) >= 17 and len(p)-r.i >= 38:
                names = ["left_u", "left_v", "left_dc",
                         "right_u", "right_v", "right_dc"]
                d["outlier_count"] = {name: r.u16() for name in names}
                d["moe_fail_mask"] = r.u8()
                d["moe_confirmed_mask"] = r.u8()
                d["moe_request_adc"] = [r.u32(), r.u32()]
                d["moe_confirm_adc"] = [r.u32(), r.u32()]
                d["first_sample_adc"] = [r.u32(), r.u32()]
            # V18 appends power-stage fault latch and live PVD status.
            if d.get("cal_diag_revision",0) >= 18 and len(p)-r.i >= 5:
                d["powerstage_fault_flags"] = r.u32()
                d["pvd_low"] = bool(r.u8())
    return d


def get_cal(link: Link, trigger: bool = False) -> dict:
    payload=bytes((COMM_CUSTOM_APP_DATA,CUSTOM_CURRENT_CAL,1 if trigger else 0))
    p=link.request(payload, lambda x: len(x)>=2 and x[:2]==bytes((COMM_CUSTOM_APP_DATA,CUSTOM_CURRENT_CAL)), 1.0)
    return parse_cal(p)


def parse_sensor(p: bytes) -> dict:
    if len(p)<2 or p[:2] != bytes((COMM_CUSTOM_APP_DATA,CUSTOM_SENSOR_INFO)):
        raise ValueError("bukan sensor-info")
    r=Reader(p,2)
    motor=r.u8(); controller_id=r.u8(); mode=r.u8(); request=r.u8(); state=r.u8(); success=bool(r.u8()); pp=r.u8(); inv=bool(r.u8()); off=r.u16()
    hall=[r.u8() for _ in range(8)]
    angles=[r.u16() for _ in range(8)]
    d={"motor":motor,"controller_id":controller_id,"mode":mode,"request":request,"state":state,"success":success,"pole_pairs":pp,
       "encoder_inverted":inv,"encoder_offset_u16":off,"hall_table":hall,"hall_angles_u16":angles}
    if r.i < len(p):
        d["diag_revision"]=r.u8()
        if d["diag_revision"] >= 13:
            d["detect_current_a"]=r.i32()/1000.0
            d["id_target_a"]=r.i32()/1000.0
            d["iq_target_a"]=r.i32()/1000.0
            d["sweep_index"]=r.u32()
            d["hall_samples"]=[r.u32() for _ in range(8)]
            d["detect_hall_table"]=[r.u8() for _ in range(8)]
            d["raw_current_1"]=r.u16(); d["raw_current_2"]=r.u16(); d["raw_dc"]=r.u16()
            d["ia_a"]=r.i32()/1000.0; d["ib_a"]=r.i32()/1000.0; d["ic_a"]=r.i32()/1000.0
            d["pwm_enabled"]=bool(r.u8()); d["moe"]=bool(r.u8())
            d["current_scale_a_per_count"]=r.i32()/1000000.0
            if d["diag_revision"] >= 15 and len(p)-r.i >= 11:
                d["pwm_enable_pending_events"]=r.u8()
                d["pwm_enable_blank_cycles"]=r.u16()
                d["pwm_tim_cnt"]=r.u16(); d["dma_cndtr"]=r.u16()
                d["tim8_rcr"]=r.u16(); d["adc_motor_phase_offset_ticks"]=r.u16()
    return d


def get_sensor(link: Link, motor: int) -> dict:
    p=link.request(bytes((COMM_CUSTOM_APP_DATA,CUSTOM_SENSOR_INFO,motor)),
                   lambda x: len(x)>=3 and x[:2]==bytes((COMM_CUSTOM_APP_DATA,CUSTOM_SENSOR_INFO)) and x[2]==motor,1.0)
    return parse_sensor(p)


def parse_extended(p: bytes) -> dict:
    if len(p)<2 or p[:2] != bytes((COMM_CUSTOM_APP_DATA,CUSTOM_EXT_TELEMETRY)):
        raise ValueError("bukan extended telemetry")
    r=Reader(p,2)
    d={}
    d["motor"]=r.u8(); d["revision"]=r.u8(); d["controller_id"]=r.u8(); d["sensor_mode"]=r.u8(); d["native_fault"]=r.u8(); d["native_fault_name"]=NATIVE_FAULT_NAMES.get(d["native_fault"],"?"); d["detect_state"]=r.u8()
    d["cal_done"]=bool(r.u8()); d["cal_valid"]=bool(r.u8()); d["hall_raw"]=r.u8(); d["pole_pairs"]=r.u8(); d["encoder_inverted"]=bool(r.u8())
    if d["revision"] >= 6:
        d["phase_current_a"]=r.i32()/1000.0
        d["phase_current_b"]=r.i32()/1000.0
        d["phase_current_c"]=r.i32()/1000.0
    names_scales=[("id",1000),("iq",1000),("id_filter",1000),("iq_filter",1000),("vd",1000),("vq",1000),
                  ("imotor",1000),("ibatt",1000),("erpm",1),("mech_rpm",10),("position_deg",1000),("rotor_elec_deg",1000),
                  ("vbus",1000),("duty",100000)]
    for name,scale in names_scales: d[name]=r.i32()/scale
    d["offset_u"]=r.i32(); d["offset_v"]=r.i32(); d["offset_dc"]=r.i32()
    d["cal_count"]=r.u32(); d["cal_target"]=r.u32(); d["isr_max_cycles"]=r.u32(); d["isr_overruns"]=r.u32()
    d["encoder_count"]=r.i32(); d["encoder_offset_u16"]=r.u16()
    if d["revision"] >= 5 and len(p)-r.i >= 55:
        d["observer_valid"]=bool(r.u8()); d["using_encoder"]=bool(r.u8()); d["encoder_synced"]=bool(r.u8())
        d["observer_phase_deg"]=r.i32()/1000.0; d["observer_erpm"]=r.i32(); d["observer_quality"]=r.i32()/100000.0
        d["foc_motor_r_ohm"]=r.i32()/1000000.0; d["foc_motor_l_h"]=r.i32()/1000000000.0
        d["foc_motor_ld_lq_diff_h"]=r.i32()/1000000000.0; d["foc_motor_flux_linkage_wb"]=r.i32()/10000000.0
        d["foc_sl_erpm_start"]=r.i32(); d["foc_sl_erpm"]=r.i32(); d["foc_openloop_rpm"]=r.i32(); d["foc_openloop_rpm_low"]=r.i32()
        d["current_loop_hz"]=r.u32(); d["telemetry_snapshot_hz"]=r.u32()
        if d["revision"] >= 7 and len(p)-r.i >= 1:
            d["foc_sensor_mode"]=r.u8()
    return d


def get_extended(link: Link, motor: int) -> dict:
    p=link.request(bytes((COMM_CUSTOM_APP_DATA,CUSTOM_EXT_TELEMETRY,motor)),
                   lambda x: len(x)>=3 and x[:2]==bytes((COMM_CUSTOM_APP_DATA,CUSTOM_EXT_TELEMETRY)) and x[2]==motor,1.0)
    return parse_extended(p)


def print_values(motor: int, v: Values) -> None:
    name="LEFT" if motor==0 else "RIGHT"
    print(f"{name}: Vbus={v.vin:6.2f} V  Imotor={v.imotor:+7.2f} A  Ibatt={v.ibatt:+7.2f} A  "
          f"Id={v.id:+7.2f} A  Iq={v.iq:+7.2f} A  Vd={v.vd:+7.2f} V  Vq={v.vq:+7.2f} V")
    print(f"      ERPM={v.erpm:+8d}  duty={v.duty:+.4f}  pos={v.position:8.3f} deg  "
          f"fault={v.fault}({FAULT_NAMES.get(v.fault,'?')}) timeout={v.timeout_status}")


def cmd_info(link: Link, _args: argparse.Namespace) -> int:
    p=link.request(bytes((COMM_FW_VERSION,)),lambda x: bool(x) and x[0]==COMM_FW_VERSION,1.0)
    d=parse_fw_version(p)
    print(f"FW: {d['major']}.{d['minor']}  name={d['fw_name']}  HW={d['hw_name']}")
    print(f"UUID={d['uuid']} HW_TYPE={d['hw_type']} custom={d['custom_config_num']} phase_filters={d['phase_filters']} hw_crc={d['hw_crc']}")
    return 0


def _validate_vesc6_wire(name: str, payload: bytes, command: int, wire_size: int, signature: int) -> bytes:
    if not payload or payload[0] != command:
        raise ValueError(f"{name}: command reply salah")
    if len(payload) != 1 + wire_size:
        raise ValueError(f"{name}: ukuran reply {len(payload)} != {1 + wire_size}")
    wire = payload[1:]
    sig = struct.unpack(">I", wire[:4])[0]
    if sig != signature:
        raise ValueError(f"{name}: signature 0x{sig:08X} != VESC 6.00 0x{signature:08X}")
    return wire


def cmd_vesc_tool_check(link: Link, args: argparse.Namespace) -> int:
    """Exercise the same core FW/config path used by VESC Tool 6.00."""
    motor = args.motor
    fw = link.request_std(motor, bytes((COMM_FW_VERSION,)),
                          lambda x: bool(x) and x[0] == COMM_FW_VERSION, args.timeout)
    info = parse_fw_version(fw)
    if (info['major'], info['minor']) != (6, 0):
        raise ValueError(f"FW ABI bukan 6.00: {info['major']}.{info['minor']}")

    mc_payload = link.request_std(motor, bytes((COMM_GET_MCCONF,)),
                                  lambda x: bool(x) and x[0] == COMM_GET_MCCONF, args.timeout)
    mc = _validate_vesc6_wire("MCCONF", mc_payload, COMM_GET_MCCONF,
                              VESC6_MCCONF_WIRE_SIZE, VESC6_MCCONF_SIGNATURE)

    app_payload = link.request_std(motor, bytes((COMM_GET_APPCONF,)),
                                   lambda x: bool(x) and x[0] == COMM_GET_APPCONF, args.timeout)
    app = _validate_vesc6_wire("APPCONF", app_payload, COMM_GET_APPCONF,
                               VESC6_APPCONF_WIRE_SIZE, VESC6_APPCONF_SIGNATURE)

    controller_id = app[4]
    expected_id = 1 if motor == 0 else 2
    if controller_id != expected_id:
        raise ValueError(f"APPCONF controller_id={controller_id}, expected {expected_id}")

    print("=== VESC TOOL 6.00 CORE PATH ===")
    print(f"motor             : {motor} ({'LEFT' if motor == 0 else 'RIGHT/local-forward-id2'})")
    print(f"FW                : {info['major']}.{info['minor']}  {info['hw_name']}  {info['fw_name']}")
    print(f"MCCONF            : PASS {len(mc)} bytes signature=0x{VESC6_MCCONF_SIGNATURE:08X}")
    print(f"APPCONF           : PASS {len(app)} bytes signature=0x{VESC6_APPCONF_SIGNATURE:08X} controller_id={controller_id}")

    if args.write_back:
        require_yes(args, "menulis kembali MCCONF/APPCONF yang baru saja dibaca ke flash")
        # Writing the exact image back is intentionally non-motor-moving, but
        # it exercises the asynchronous SET + ACK + persistent-store path used
        # by VESC Tool. Firmware rejects MCCONF writes while the motor is active.
        ack_mc = link.request_std(motor, bytes((COMM_SET_MCCONF,)) + mc,
                                  lambda x: x == bytes((COMM_SET_MCCONF,)), args.write_timeout)
        if ack_mc != bytes((COMM_SET_MCCONF,)):
            raise ValueError("MCCONF SET ACK salah")
        ack_app = link.request_std(motor, bytes((COMM_SET_APPCONF,)) + app,
                                   lambda x: x == bytes((COMM_SET_APPCONF,)), args.write_timeout)
        if ack_app != bytes((COMM_SET_APPCONF,)):
            raise ValueError("APPCONF SET ACK salah")
        print("SET_MCCONF ACK    : PASS")
        print("SET_APPCONF ACK   : PASS")

        # Read back and require byte-exact persistence semantics. For motor 2
        # APPCONF is exposed with controller_id=2 on the wire by design.
        mc2 = _validate_vesc6_wire("MCCONF readback",
            link.request_std(motor, bytes((COMM_GET_MCCONF,)), lambda x: bool(x) and x[0] == COMM_GET_MCCONF, args.timeout),
            COMM_GET_MCCONF, VESC6_MCCONF_WIRE_SIZE, VESC6_MCCONF_SIGNATURE)
        app2 = _validate_vesc6_wire("APPCONF readback",
            link.request_std(motor, bytes((COMM_GET_APPCONF,)), lambda x: bool(x) and x[0] == COMM_GET_APPCONF, args.timeout),
            COMM_GET_APPCONF, VESC6_APPCONF_WIRE_SIZE, VESC6_APPCONF_SIGNATURE)
        if mc2 != mc or app2 != app:
            raise ValueError("readback config tidak byte-exact setelah SET")
        print("READBACK          : PASS byte-exact")

    print("PASS: jalur inti handshake/read-config VESC Tool 6.00 valid")
    return 0


def _raw_handshake(link: Link, timeout: float = 1.0) -> tuple[bytes, list[bytes]]:
    tx = frame(bytes((COMM_FW_VERSION,)))
    link.ser.reset_input_buffer()
    link.parser = FrameParser()
    link.pending.clear()
    print("TX:", tx.hex(" "))
    link.ser.write(tx); link.ser.flush()
    raw=bytearray(); parser=FrameParser(); frames=[]
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        chunk=link.ser.read(link.ser.in_waiting or 1)
        if chunk:
            raw.extend(chunk); frames.extend(parser.feed(chunk))
            if any(p and p[0]==COMM_FW_VERSION for p in frames): break
    return bytes(raw), frames


def cmd_handshake(link: Link, args: argparse.Namespace) -> int:
    all_raw=bytearray()
    for attempt in range(1, args.attempts + 1):
        print(f"\n--- handshake attempt {attempt}/{args.attempts} ---")
        raw,frames=_raw_handshake(link,args.timeout)
        all_raw.extend(raw)
        print("RX raw:", raw.hex(" ") if raw else "<no bytes>")
        fw=next((p for p in frames if p and p[0]==COMM_FW_VERSION),None)
        if fw is not None:
            print("RX payload:", fw.hex(" "))
            print(parse_fw_version(fw))
            print("PASS: framing + CRC + COMM_FW_VERSION reply valid")
            return 0
        time.sleep(0.05)

    if not all_raw:
        print("FAIL/LEVEL-1: MCU mengirim 0 byte. Fokus ke boot 64MHz, USART3 RXNE IRQ/ring, uartcomm packet thread, TXE/TC IRQ, wiring PB10/PB11, GND, dan baud.")
    else:
        print("FAIL/LEVEL-2: ada byte dari MCU tetapi tidak terbentuk frame VESC valid. Fokus ke baud/clock 64MHz, USART3 TXE/TC IRQ, framing packet, atau CRC.")
        print("ALL RX raw:", bytes(all_raw).hex(" "))
    return 2


def cmd_baud_scan(link: Link, args: argparse.Namespace) -> int:
    rates=[int(x.strip()) for x in args.bauds.split(',') if x.strip()]
    old=link.ser.baudrate
    try:
        for rate in rates:
            link.ser.baudrate=rate; time.sleep(0.05)
            print(f"\n=== {rate} baud ===")
            raw,frames=_raw_handshake(link,args.timeout)
            fw=next((p for p in frames if p and p[0]==COMM_FW_VERSION),None)
            if fw is not None:
                print(parse_fw_version(fw))
                print(f"FOUND VESC handshake at {rate} baud")
                return 0
            print(f"no valid reply ({len(raw)} raw bytes)")
    finally:
        link.ser.baudrate=old
    return 3


def parse_comm_diag(p: bytes) -> dict:
    if len(p)<3 or p[:2]!=bytes((COMM_CUSTOM_APP_DATA,CUSTOM_COMM_DIAG)):
        raise ValueError("bukan COMM_DIAG")
    r=Reader(p,2); revision=r.u8()
    d={'revision':revision}
    if revision >= 6:
        names=['rx_bytes','rx_ring_overruns','rx_frames_ok','tx_bytes','uart_errors','tx_frames',
               'tx_ring_overruns','tx_complete_count','blocking_busy_drops',
               'motor2_forwards','unsupported_forward_ids','baud']
        for name in names: d[name]=r.u32()
        d['blocking_queue_depth']=r.u8()
        d['timeout_active']=bool(r.u8())
        d['config_valid']=bool(r.u8())
        if revision >= 8 and len(p)-r.i >= 35:
            d['rx_dma_irq_count']=r.u32(); d['tx_dma_irq_count']=r.u32()
            d['idle_irq_count']=r.u32(); d['dma_errors']=r.u32(); d['reset_flags']=r.u32()
            d['had_iwdg_reset']=bool(r.u8()); d['watchdog_started']=bool(r.u8()); d['watchdog_healthy']=bool(r.u8())
            if revision >= 9 and len(p)-r.i >= 27:
                d['config_integrity_ok']=bool(r.u8())
                d['config_integrity_checks']=r.u32(); d['config_integrity_failures']=r.u32()
                d['power_hold']=bool(r.u8()); d['shutdown_latched']=bool(r.u8())
            if len(p)-r.i >= 16:
                d['watchdog_required_mask']=r.u32()
                d['heartbeat_foc']=r.u32(); d['heartbeat_motor_service']=r.u32(); d['heartbeat_comm']=r.u32()
                if revision >= 12 and len(p)-r.i >= 4:
                    d['heartbeat_fault']=r.u32()
            if revision >= 10 and len(p)-r.i >= 28:
                d['watchdog_unhealthy_mask']=r.u32()
                d['watchdog_miss_foc']=r.u32(); d['watchdog_miss_motor_service']=r.u32(); d['watchdog_miss_comm']=r.u32()
                if revision >= 12 and len(p)-r.i >= 5:
                    d['watchdog_miss_fault']=r.u32(); d['config_boot_status']=r.u8()
                d['sample_clamp_left']=r.u32(); d['sample_clamp_right']=r.u32()
                d['sample_margin_left_q15']=r.u16(); d['sample_margin_right_q15']=r.u16()
                if revision >= 11 and len(p)-r.i >= 20:
                    d['app_adc_raw1']=r.u16(); d['app_adc_raw2']=r.u16()
                    d['app_adc_mv1']=r.u16(); d['app_adc_mv2']=r.u16()
                    d['app_adc_decoded1']=r.i16()/1000.0; d['app_adc_decoded2']=r.i16()/1000.0
                    d['app_adc_command']=r.i16()/1000.0
                    d['app_adc_fault_flags']=r.u8(); d['app_adc_range_ok']=bool(r.u8())
                    d['app_adc_armed_left']=bool(r.u8()); d['app_adc_armed_right']=bool(r.u8())
                    d['app_cmd_source_left']=r.u8(); d['app_cmd_source_right']=r.u8()
                    if revision >= 13 and len(p)-r.i >= 46:
                        d['sampling_contract_flags']=r.u32()
                        d['isr_total_max_cycles']=r.u32(); d['isr_near_deadline_count']=r.u32()
                        d['isr_period_min_cycles']=r.u32(); d['isr_period_max_cycles']=r.u32()
                        d['heap_free_bytes']=r.u32(); d['heap_min_ever_bytes']=r.u32()
                        d['stack_motor_free_bytes']=r.u16(); d['stack_sample_free_bytes']=r.u16()
                        d['stack_fault_free_bytes']=r.u16(); d['stack_status_free_bytes']=r.u16()
                        d['stack_packet_free_bytes']=r.u16(); d['stack_block_free_bytes']=r.u16()
                        d['tx_queue_high_water']=r.u16(); d['tx_queue_busy_drops']=r.u32()
                        if revision >= 14 and len(p)-r.i >= 4:
                            d['tx_low_priority_drops']=r.u32()
    elif revision >= 5:
        names=['rx_bytes','rx_ring_overruns','rx_frames_ok','tx_bytes','uart_errors','tx_frames','tx_ring_overruns','tx_complete_count','blocking_busy_drops','baud']
        for name in names: d[name]=r.u32()
        d['tx_queue_depth']=r.u8(); d['blocking_queue_depth']=r.u8()
    else:
        names=['rx_dma_bytes','rx_ring_overruns','rx_frames_ok','uart_idle_events','uart_errors','tx_frames','tx_queue_drops','tx_dma_errors','blocking_busy_drops','baud']
        for name in names: d[name]=r.u32()
        d['tx_queue_depth']=r.u8(); d['blocking_queue_depth']=r.u8()
    return d

def cmd_comm_diag(link: Link, _args: argparse.Namespace) -> int:
    p=link.request(bytes((COMM_CUSTOM_APP_DATA,CUSTOM_COMM_DIAG)),
                   lambda x: len(x)>=2 and x[:2]==bytes((COMM_CUSTOM_APP_DATA,CUSTOM_COMM_DIAG)),1.0)
    d=parse_comm_diag(p)
    for k,v in d.items(): print(f"{k:22s}: {v}")
    return 0


def cmd_status(link: Link, args: argparse.Namespace) -> int:
    motors=[args.motor] if args.motor is not None else [0,1]
    for m in motors:
        print_values(m,get_values(link,m))
        e=get_extended(link,m)
        phase_txt = ""
        if e.get("revision", 0) >= 6:
            phase_txt=(f" Ia/Ib/Ic={e['phase_current_a']:+.2f}/{e['phase_current_b']:+.2f}/"
                       f"{e['phase_current_c']:+.2f} A")
        print(f"      sensor={SENSOR_NAMES.get(e['sensor_mode'],e['sensor_mode'])} hall={e['hall_raw']:03b} "
              f"rotor={e['rotor_elec_deg']:.2f} deg mechRPM={e['mech_rpm']:.1f}{phase_txt} "
              f"offsets={e['offset_u']}/{e['offset_v']}/{e['offset_dc']} ISRmax={e['isr_max_cycles']} overrun={e['isr_overruns']}")
    return 0


def _calibration_channel_status(d: dict, idx: int, name: str) -> str:
    bit=1<<idx
    if d.get("fail_range_mask",0) & bit: return "FAIL_RANGE"
    if d.get("fail_noise_mask",0) & bit: return "FAIL_NOISE"
    if d.get("warn_mask",0) & bit: return "WARN_NOISE"
    return "PASS"


def _print_calibration_diagnostics(d: dict) -> None:
    print("\n=== CURRENT CALIBRATION RESULT ===")
    for k in ("done","valid","count","target","left_u","left_v","left_dc","right_u","right_v","right_dc"):
        if k in d: print(f"{k:24s}: {d[k]}")
    print("\n=== HARDWARE LIVENESS ===")
    for k in ("adc_isr_count","dma_cndtr","tim2_cnt","tim1_dir","tim1_running","tim8_running","tim2_running",
              "adc1_enabled","adc2_enabled","dma1_ch1_enabled"):
        if k in d: print(f"{k:24s}: {d[k]}")
    if "cal_diag_revision" in d:
        print("\n=== CURRENT CALIBRATION STATISTICS ===")
        print(f"diag_revision           : {d['cal_diag_revision']}")
        print(f"warn_mask               : 0x{d.get('warn_mask',0):04X}")
        print(f"fail_range_mask         : 0x{d.get('fail_range_mask',0):04X}")
        print(f"fail_noise_mask         : 0x{d.get('fail_noise_mask',0):04X}")
        print("channel      mean   min   max spread  stddev outlier      status")
        names=["left_u","left_v","left_dc","right_u","right_v","right_dc"]
        for idx,name in enumerate(names):
            c=d.get("channels",{}).get(name)
            if not c: continue
            outliers=d.get("outlier_count",{}).get(name,0)
            print(f"{name:10s} {c['mean']:5d} {c['min']:5d} {c['max']:5d} "
                  f"{c['spread']:6d} {c['stddev']:8.3f} {outliers:7d}  "
                  f"{_calibration_channel_status(d,idx,name)}")
        print("\nThreshold policy:")
        print("  undriven pass         : only reject ADC rail / gross hardware fault")
        print("  driven offset source  : 50%/50%/50% zero-vector switching, 1000 samples/motor")
        print("  hard mean range       : 128..3967 ADC counts")
        if d.get("cal_diag_revision",0) >= 17:
            print("  robust inlier window  : +/-256 counts around undriven mean")
            print("  warning noise         : any outlier OR raw spread >160 OR clean stddev >16")
            print("  hard driven-noise fail: >10 outliers, <990 inliers, OR clean stddev >80")
        else:
            print("  warning noise         : spread >160 OR stddev >16 counts")
            print("  hard driven-noise fail: spread >800 OR stddev >80 counts")
        print("  PWM start blanking    : 8 driven samples at 50% zero vector; DC trip 17 A")
        print("  warning does NOT inhibit PWM; hard failure does.")
        if d.get("cal_diag_revision",0) >= 14:
            stage_names={0:"UNDRIVEN",1:"WAIT_LEFT_DRIVEN",2:"LEFT_WARMUP",3:"LEFT_DRIVEN",
                         4:"WAIT_RIGHT_DRIVEN",5:"RIGHT_WARMUP",6:"RIGHT_DRIVEN",
                         7:"WAIT_FINALIZE",8:"DONE",9:"FAILED"}
            st=d.get("cal_stage",-1)
            print("\n=== DRIVEN/OFFSET SPLIT ===")
            print(f"cal_stage               : {st} {stage_names.get(st,'?')}")
            print(f"shift_warn_mask         : 0x{d.get('shift_warn_mask',0):04X}")
            print("channel      undriven driven  delta")
            for name in ["left_u","left_v","left_dc","right_u","right_v","right_dc"]:
                u=d.get("undriven_mean",{}).get(name,0); v=d.get("driven_mean",{}).get(name,0)
                print(f"{name:10s} {u:8d} {v:6d} {v-u:6d}")
            fs=d.get("fault_snapshot",{})
            print("\n=== FIRST ACTIVE-DRIVE CURRENT-FAULT SNAPSHOT ===")
            if fs.get("valid"):
                for k,v in fs.items(): print(f"{k:24s}: {v}")
            else:
                print("valid                   : False (no active-drive current fault captured)")
        if d.get("cal_diag_revision",0) >= 15:
            print("\n=== ADC/PWM SCHEDULE ===")
            print(f"adc_phase_offset_ticks  : {d.get('adc_motor_phase_offset_ticks','?')}")
            print(f"foc_isr_event_hz        : {d.get('foc_isr_event_hz','?')}")
            print(f"foc_isr_slot_cycles     : {d.get('foc_isr_slot_cycles','?')}")
            print(f"tim8_rcr                : {d.get('tim8_rcr','?')}")
            print(f"tim1_cnt/tim8_cnt       : {d.get('tim1_cnt_v15','?')} / {d.get('tim8_cnt_v15','?')}")
            print("rank1                   : ADC1 RIGHT_DC | ADC2 LEFT_DC")
            print("rank2                   : ADC1 LEFT_A   | ADC2 LEFT_B")
            print("rank3                   : ADC1 RIGHT_B  | ADC2 RIGHT_C")
        if d.get("cal_diag_revision",0) >= 16:
            print("\n=== ADC3 DCLINK PATH ===")
            for k in ("adc3_enabled","dma2_ch5_enabled","dma2_ch5_cndtr",
                      "adc3_vbus_raw0","adc3_vbus_raw1","vbus_dma_stale_count",
                      "vbus_dma_stale_events"):
                if k in d: print(f"{k:24s}: {d[k]}")
            print("trigger                  : TIM8_TRGO (same PWM frame as current scan)")
            print("DCLINK                   : PC2 / ADC3_IN12 / DMA2 Channel 5")
        if d.get("cal_diag_revision",0) >= 17:
            print("\n=== PWM MOE CALIBRATION TRACE ===")
            print(f"moe_fail_mask           : 0x{d.get('moe_fail_mask',0):02X}")
            print(f"moe_confirmed_mask      : 0x{d.get('moe_confirmed_mask',0):02X}")
            request=d.get("moe_request_adc",[0,0])
            confirm=d.get("moe_confirm_adc",[0,0])
            first=d.get("first_sample_adc",[0,0])
            print("motor        request_adc confirm_adc first_sample delta_confirm delta_sample")
            for idx,name in enumerate(("LEFT", "RIGHT")):
                req=request[idx]; con=confirm[idx]; fst=first[idx]
                print(f"{name:10s} {req:11d} {con:11d} {fst:12d} "
                      f"{(con-req) if con else -1:13d} {(fst-con) if fst and con else -1:12d}")
    regs=d.get("registers",{})
    if regs:
        print("\n=== RAW PERIPHERAL REGISTERS ===")
        for k,v in regs.items(): print(f"{k:24s}: 0x{v:08X} ({v})")
    if d.get("dma_words"):
        print("\n=== ADC DUAL DMA RAW WORDS ===")
        for idx,w in enumerate(d["dma_words"],1):
            print(f"rank{idx}: word=0x{w['word']:08X} ADC1={w['adc1']:4d} ADC2={w['adc2']:4d}")


def _cmd_calibrate_body(link: Link,args: argparse.Namespace)->int:
    print(f"timestamp               : {datetime.now().isoformat(timespec='seconds')}")
    print(f"port                    : {link.ser.port}")
    print(f"baud                    : {link.ser.baudrate}")
    link.stop(0); link.stop(1); time.sleep(0.1)
    d=get_cal(link,trigger=True)
    print("Calibration started; both motor PWM commands stopped.")
    deadline=time.monotonic()+args.timeout
    history=[]
    while time.monotonic()<deadline:
        time.sleep(0.1)
        d=get_cal(link,False)
        history.append((time.monotonic(),d.get('adc_isr_count',0),d.get('dma_cndtr',-1),d.get('tim8_cnt_v15',d.get('tim2_cnt',-1)),d.get('count',0)))
        if d['done']: break
    _print_calibration_diagnostics(d)
    print("\n=== LIVENESS SAMPLES DURING CALIBRATION ===")
    for j,(ts,isrc,cndtr,timcnt,cnt) in enumerate(history[-12:]):
        print(f"sample[{j:02d}] cal={cnt:4d} adc_isr={isrc:10d} dma_cndtr={cndtr:3d} tim8_cnt={timcnt:5d}")
    if len(history)>=2:
        delta=history[-1][1]-history[0][1]
        print(f"adc_isr_delta            : {delta}")
    if not d['done'] or not d['valid']:
        print("\nRESULT: FAIL - calibration hard sanity check failed.")
        return 2
    time.sleep(0.4)
    ok=True
    print("\n=== ZERO-CURRENT TELEMETRY AFTER CALIBRATION ===")
    for m in (0,1):
        try:
            v=get_values(link,m); print_values(m,v)
            e=get_extended(link,m); print("EXT:",e)
            if abs(v.id)>args.zero_limit or abs(v.iq)>args.zero_limit or abs(v.imotor)>args.zero_limit*1.5:
                ok=False
        except Exception as exc:
            ok=False; print(f"M{m} telemetry error: {exc}")
    print("\nRESULT:", "PASS zero-current" if ok else "WARN/FAIL zero-current residual/telemetry")
    return 0 if ok else 3


def _default_txt(prefix: str) -> Path:
    return Path(f"{prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")


def cmd_calibrate(link: Link,args: argparse.Namespace)->int:
    path=Path(getattr(args,"out",None) or _default_txt("vesc_f103_calibration"))
    path.parent.mkdir(parents=True,exist_ok=True)
    rc=1
    with path.open("w",encoding="utf-8") as f, contextlib.redirect_stdout(f), contextlib.redirect_stderr(f):
        try:
            rc=_cmd_calibrate_body(link,args)
        except Exception:
            print("\n=== EXCEPTION ===")
            traceback.print_exc()
            rc=1
    print(f"DEBUG TXT: {path.resolve()}")
    print(f"RESULT CODE: {rc}")
    return rc


# Full passive diagnostic report including observer/model and hardware timing fields.
def cmd_diagnose(link: Link,args: argparse.Namespace)->int:
    path=Path(getattr(args,"out",None) or _default_txt("vesc_f103_full_debug"))
    path.parent.mkdir(parents=True,exist_ok=True)
    rc=0
    with path.open("w",encoding="utf-8") as f, contextlib.redirect_stdout(f), contextlib.redirect_stderr(f):
        try:
            print("STM32F103RCT6 VESC FOC FULL PASSIVE DEBUG REPORT")
            print("timestamp:",datetime.now().isoformat(timespec='seconds'))
            print("port:",link.ser.port,"baud:",link.ser.baudrate)
            print("\n=== FIRMWARE ===")
            cmd_info(link,args)
            print("\n=== COMM USART3 IRQ/RING ===")
            cmd_comm_diag(link,args)
            print("\n=== PRE-CAL HARDWARE SNAPSHOT ===")
            pre=get_cal(link,False); _print_calibration_diagnostics(pre)
            print("\n=== RECALIBRATION ===")
            cal_args=argparse.Namespace(timeout=args.timeout,zero_limit=args.zero_limit,out=None)
            # Call the body directly so the whole report stays in this one TXT.
            cr=_cmd_calibrate_body(link,cal_args)
            if cr != 0: rc=cr
            print("\n=== SENSOR STATE ===")
            for m in (0,1):
                try:
                    print(("LEFT" if m==0 else "RIGHT"),get_sensor(link,m))
                except Exception as exc: print(f"sensor M{m} error: {exc}")
            print("\n=== FINAL TELEMETRY SNAPSHOTS ===")
            for sample in range(5):
                print(f"-- snapshot {sample} --")
                for m in (0,1):
                    try:
                        print_values(m,get_values(link,m)); print("EXT:",get_extended(link,m))
                    except Exception as exc: print(f"M{m}: {exc}")
                time.sleep(0.2)
            print("\n=== FINAL COMM USART3 IRQ/RING ===")
            cmd_comm_diag(link,args)
        except Exception:
            print("\n=== EXCEPTION ===")
            traceback.print_exc(); rc=1
    print(f"DEBUG TXT: {path.resolve()}")
    print(f"RESULT CODE: {rc}")
    return rc


def cmd_sensor_info(link: Link,args: argparse.Namespace)->int:
    motors=[args.motor] if args.motor is not None else [0,1]
    for m in motors:
        d=get_sensor(link,m)
        print(("LEFT" if m==0 else "RIGHT"), d)
        print(" hall table:",d['hall_table'])
        print(" hall deg  :",[round(a*360/65536,1) for a in d['hall_angles_u16']])
    return 0


def require_yes(args: argparse.Namespace, what: str) -> None:
    if not getattr(args,"yes",False):
        raise RuntimeError(f"{what} dapat menggerakkan motor. Ulangi dengan --yes setelah power-stage siap.")


def cmd_sensor_select(link: Link,args: argparse.Namespace)->int:
    mode={"hall":SENSOR_HALL,"encoder":SENSOR_ENCODER}[args.mode]
    if args.motor==1 and mode==SENSOR_ENCODER:
        raise RuntimeError("RIGHT pada PCB ini hanya Hall")
    link.stop(args.motor); time.sleep(0.05)
    p=link.request(bytes((COMM_CUSTOM_APP_DATA,CUSTOM_SENSOR_SELECT,args.motor,mode)),
                   lambda x: len(x)>=3 and x[:2]==bytes((COMM_CUSTOM_APP_DATA,CUSTOM_SENSOR_INFO)) and x[2]==args.motor,1.0)
    print(parse_sensor(p)); return 0


def _cmd_sensor_detect_body(link: Link,args: argparse.Namespace)->int:
    require_yes(args,"Auto-detect sensor")
    mode={"auto":SENSOR_AUTO,"hall":SENSOR_HALL,"encoder":SENSOR_ENCODER}[args.mode]
    if args.motor==1 and mode==SENSOR_ENCODER: raise RuntimeError("RIGHT tidak memiliki encoder")
    cal=get_cal(link,False)
    if not cal['done'] or not cal['valid']: raise RuntimeError("Current-zero calibration belum valid. Jalankan 'calibrate' dulu.")
    print("VESC F103 SENSOR DETECT TRACE")
    print("timestamp:",datetime.now().isoformat(timespec='seconds'))
    print("motor:","LEFT" if args.motor==0 else "RIGHT","mode:",args.mode,"current_A:",args.current)
    print("NOTE: Hall detect follows VESC 1s Id ramp + 1deg/5ms forward/reverse sweeps.")
    link.stop(args.motor); link.clear_fault(args.motor); time.sleep(0.1)
    print("PRE:",get_sensor(link,args.motor))
    print("PRE-EXT:",get_extended(link,args.motor))
    ma=int(round(args.current*1000.0))
    if ma < 200 or ma > 2000: raise ValueError("--current harus 0.2..2.0 A")
    link.send(bytes((COMM_CUSTOM_APP_DATA,CUSTOM_SENSOR_DETECT,args.motor,mode))+be_i32(ma))
    deadline=time.monotonic()+args.timeout
    last=None; seq=0
    while time.monotonic()<deadline:
        d=get_sensor(link,args.motor); state=d['state']
        if state!=last:
            print(f"\nSTATE {state} {DETECT_NAMES.get(state,'?')} mode={SENSOR_NAMES.get(d['mode'],d['mode'])}")
            last=state
        if seq % 5 == 0 or state in (10, 11, DETECT_DONE, DETECT_FAILED):
            print("SENSOR:",d)
            try: print("EXT   :",get_extended(link,args.motor))
            except Exception as exc: print("EXT error:",exc)
        if state in (DETECT_DONE,DETECT_FAILED):
            print("FINAL:",d)
            try:
                cal_after=get_cal(link,False)
                print("\n=== CURRENT/FAULT SNAPSHOT AT DETECT END ===")
                _print_calibration_diagnostics(cal_after)
            except Exception as exc:
                print("fault snapshot read error:",exc)
            return 0 if state==DETECT_DONE and d['success'] else 4
        seq+=1; time.sleep(0.1)
    raise TimeoutError("sensor detect timeout")


def cmd_sensor_detect(link: Link,args: argparse.Namespace)->int:
    path=Path(getattr(args,"out",None) or _default_txt("vesc_f103_sensor_detect"))
    path.parent.mkdir(parents=True,exist_ok=True)
    rc=1
    with path.open("w",encoding="utf-8") as f, contextlib.redirect_stdout(f), contextlib.redirect_stderr(f):
        try:
            rc=_cmd_sensor_detect_body(link,args)
        except Exception:
            print("\n=== EXCEPTION ==="); traceback.print_exc(); rc=1
    print(f"DEBUG TXT: {path.resolve()}")
    print(f"RESULT CODE: {rc}")
    return rc


def _read_mcconf_wire(link: Link, motor: int, timeout: float) -> bytes:
    payload=link.request_std(motor,bytes((COMM_GET_MCCONF,)),
        lambda x: bool(x) and x[0]==COMM_GET_MCCONF,timeout)
    return _validate_vesc6_wire("MCCONF",payload,COMM_GET_MCCONF,
                                VESC6_MCCONF_WIRE_SIZE,VESC6_MCCONF_SIGNATURE)


def parse_standard_encoder_detect(p: bytes) -> dict:
    if len(p) != 10 or p[0] != COMM_DETECT_ENCODER:
        raise ValueError(f"COMM_DETECT_ENCODER reply size/command invalid: {len(p)}")
    r=Reader(p,1)
    offset=r.i32()/1_000_000.0
    ratio=r.i32()/1_000_000.0
    inverted=bool(r.u8())
    ok=abs(offset-1001.0)>1e-6 and ratio>0.0
    return {"ok":ok,"offset_deg":offset,"ratio":ratio,"inverted":inverted}


def parse_standard_hall_detect(p: bytes) -> dict:
    if len(p) != 10 or p[0] != COMM_DETECT_HALL_FOC:
        raise ValueError(f"COMM_DETECT_HALL_FOC reply size/command invalid: {len(p)}")
    table=list(p[1:9])
    ok=bool(p[9])
    return {"ok":ok,"hall_table":table}


def cmd_detect_encoder_standard(link: Link,args: argparse.Namespace)->int:
    require_yes(args,"Standard VESC encoder detection")
    if args.motor != 0:
        raise RuntimeError("Encoder A/B fisik hanya tersedia pada LEFT/motor 0")
    if not 0.2 <= args.current <= 5.0:
        raise ValueError("--current harus 0.2..5.0 A")
    before=_read_mcconf_wire(link,args.motor,args.timeout)
    link.stop(args.motor); link.clear_fault(args.motor); time.sleep(0.1)
    req=bytes((COMM_DETECT_ENCODER,))+be_i32(int(round(args.current*1000.0)))
    p=link.request_std(args.motor,req,
        lambda x: len(x)==10 and x[0]==COMM_DETECT_ENCODER,args.timeout)
    d=parse_standard_encoder_detect(p)
    after=_read_mcconf_wire(link,args.motor,2.0)
    unchanged=(before==after)
    print("COMM_DETECT_ENCODER:",d)
    print("MCCONF unchanged    :", "PASS" if unchanged else "FAIL")
    print("NOTE: command standar hanya mengembalikan hasil; VESC Tool yang memilih Apply/Write.")
    return 0 if d['ok'] and unchanged else 3


def cmd_detect_hall_standard(link: Link,args: argparse.Namespace)->int:
    require_yes(args,"Standard VESC Hall detection")
    if not 0.2 <= args.current <= 5.0:
        raise ValueError("--current harus 0.2..5.0 A")
    before=_read_mcconf_wire(link,args.motor,args.timeout)
    link.stop(args.motor); link.clear_fault(args.motor); time.sleep(0.1)
    req=bytes((COMM_DETECT_HALL_FOC,))+be_i32(int(round(args.current*1000.0)))
    p=link.request_std(args.motor,req,
        lambda x: len(x)==10 and x[0]==COMM_DETECT_HALL_FOC,args.timeout)
    d=parse_standard_hall_detect(p)
    after=_read_mcconf_wire(link,args.motor,2.0)
    unchanged=(before==after)
    valid_states=sum(1 for v in d['hall_table'] if v != 255)
    print("COMM_DETECT_HALL_FOC:",d,"valid_states=",valid_states)
    print("MCCONF unchanged       :", "PASS" if unchanged else "FAIL")
    print("NOTE: command standar hanya mengembalikan Hall table; VESC Tool yang memilih Apply/Write.")
    return 0 if d['ok'] and valid_states==6 and unchanged else 3


def cmd_rotor(link: Link,args: argparse.Namespace)->int:
    mode=DISPLAY_MODES[args.mode]
    link.send_std(args.motor, bytes((COMM_SET_DETECT,mode)))
    end=time.monotonic()+args.seconds
    try:
        while time.monotonic()<end:
            p=link.recv(timeout=0.5,pred=lambda x: len(x)>=5 and x[0]==COMM_ROTOR_POSITION)
            pos=struct.unpack(">i",p[1:5])[0]/100000
            print(f"rotor/position = {pos:10.5f} deg")
    finally:
        link.send_std(args.motor, bytes((COMM_SET_DETECT,0)))
    return 0


def parse_sample_packet(p: bytes) -> tuple[int, dict]:
    if len(p) < 1 or p[0] != COMM_SAMPLE_PRINT:
        raise ValueError("not COMM_SAMPLE_PRINT")
    r=Reader(p,1)
    index16=r.i16()
    row={
        "index": index16,
        "current0": r.f32_auto(), "current1": r.f32_auto(), "current2": r.f32_auto(),
        "phase1": r.f32_auto(), "phase2": r.f32_auto(), "phase3": r.f32_auto(),
        "vzero": r.f32_auto(), "current_fir": r.f32_auto(),
        "switching_frequency": r.f32_auto(),
        "status": r.u8(),
        "phase": r.u8(),
        "index_full": r.i32(),
    }
    return index16,row


def cmd_sample(link: Link,args: argparse.Namespace)->int:
    count=max(1,min(args.count,256)); dec=max(1,min(args.decimation,255))
    # mode=1 is immediate/NOW sampling in the F103 port; wire format follows
    # COMM_SAMPLE_PRINT: mode, uint16 sample_len, uint8 decimation, raw.
    req=bytes((COMM_SAMPLE_PRINT,1))+be_u16(count)+bytes((dec,1 if getattr(args,'raw',False) else 0))
    link.send_std(args.motor,req)
    collected:dict[int,dict]={}; deadline=time.monotonic()+args.timeout
    while time.monotonic()<deadline and len(collected)<count:
        p=link.recv(timeout=1.0,pred=lambda x: len(x)>=1 and x[0]==COMM_SAMPLE_PRINT)
        idx,row=parse_sample_packet(p)
        collected[idx]=row
        print(f"\rreceived {len(collected)}/{count}",end='',flush=True)
    print()
    rows=[collected[k] for k in sorted(collected)]
    if args.csv:
        path=Path(args.csv)
        with path.open("w",newline="") as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0].keys()) if rows else [])
            if rows: w.writeheader(); w.writerows(rows)
        print("CSV:",path)
    for row in rows[:min(12,len(rows))]: print(row)
    return 0 if len(rows)==count else 5

def command_payload(mode:str,value:float)->bytes:
    if mode=="current": return bytes((COMM_SET_CURRENT,))+be_i32(round(value*1000))
    if mode=="brake": return bytes((COMM_SET_CURRENT_BRAKE,))+be_i32(round(abs(value)*1000))
    if mode=="rpm": return bytes((COMM_SET_RPM,))+be_i32(round(value))
    if mode=="duty": return bytes((COMM_SET_DUTY,))+be_i32(round(value*100000))
    if mode=="position": return bytes((COMM_SET_POS,))+be_i32(round(value*1_000_000))
    raise ValueError(mode)



def cmd_detect_rl(link: Link,args: argparse.Namespace)->int:
    require_yes(args,"R/L detection")
    p=link.request_std(args.motor,bytes((COMM_DETECT_MOTOR_R_L,)),
        lambda x: len(x)>=9 and x[0]==COMM_DETECT_MOTOR_R_L,args.timeout)
    r=Reader(p,1); resistance=r.i32()/1_000_000.0; inductance_uH=r.i32()/1000.0
    ld_lq_uH=(r.i32()/1000.0) if len(p)-r.i>=4 else 0.0
    inductance=inductance_uH*1.0e-6; ld_lq=ld_lq_uH*1.0e-6
    print(f"R={resistance:.8f} ohm  L={inductance_uH:.3f} uH ({inductance:.9f} H)  "
          f"Ld-Lq={ld_lq_uH:.3f} uH ({ld_lq:.9f} H)")
    return 0 if resistance>0 and inductance>0 else 3


def cmd_detect_flux(link: Link,args: argparse.Namespace)->int:
    require_yes(args,"Flux linkage detection")
    current=int(round(args.current*1000.0))
    resistance=int(round(args.resistance*1_000_000.0))
    if args.openloop:
        payload=(bytes((COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,))+be_i32(current)+
                 be_i32(int(round(args.erpm_per_sec*1000.0)))+
                 be_i32(int(round(args.duty*1000.0)))+be_i32(resistance)+
                 be_i32(int(round(args.inductance*100_000_000.0))))
        cmd=COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP
    else:
        payload=(bytes((COMM_DETECT_MOTOR_FLUX_LINKAGE,))+be_i32(current)+
                 be_i32(int(round(args.min_erpm*1000.0)))+
                 be_i32(int(round(args.duty*1000.0)))+be_i32(resistance))
        cmd=COMM_DETECT_MOTOR_FLUX_LINKAGE
    p=link.request_std(args.motor,payload,lambda x: len(x)>=5 and x[0]==cmd,args.timeout)
    flux=Reader(p,1).i32()/10_000_000.0
    print(f"flux_linkage={flux:.9f} Wb")
    return 0 if flux>0 else 3


def cmd_detect_all_foc(link: Link,args: argparse.Namespace)->int:
    require_yes(args,"Full FOC auto-detection")
    # Port ini tidak memiliki physical CAN. Byte detect_can tetap ada pada wire
    # command VESC, tetapi selalu nol agar tidak mengiklankan backend yang tidak ada.
    payload=(bytes((COMM_DETECT_APPLY_ALL_FOC,0))+
             be_i32(int(round(args.max_power_loss*1000.0)))+
             be_i32(int(round(args.min_input_current*1000.0)))+
             be_i32(int(round(args.max_input_current*1000.0)))+
             be_i32(int(round(args.openloop_erpm*1000.0)))+
             be_i32(int(round(args.sl_erpm*1000.0))))
    p=link.request_std(args.motor,payload,lambda x: len(x)>=3 and x[0]==COMM_DETECT_APPLY_ALL_FOC,args.timeout)
    result=Reader(p,1).i16(); print(f"detect_apply_all_foc result={result}")
    try: print("EXT:",get_extended(link,args.motor))
    except Exception as exc: print("extended telemetry after detect error:",exc)
    return 0 if result==0 else 3

def cmd_motor_test(link: Link,args: argparse.Namespace)->int:
    require_yes(args,"Motor test")
    cal=get_cal(link,False)
    if not cal['valid']: raise RuntimeError("current calibration tidak valid")
    s=get_sensor(link,args.motor)
    if not s['success'] and not args.force:
        raise RuntimeError("sensor belum lulus auto-detect. Jalankan sensor-detect atau gunakan --force dengan risiko sendiri.")
    link.clear_fault(args.motor); time.sleep(0.05)
    payload=command_payload(args.mode,args.value)
    end=time.monotonic()+args.seconds; next_t=0.0
    try:
        while time.monotonic()<end:
            now=time.monotonic()
            if now>=next_t:
                link.send_std(args.motor,payload); next_t=now+0.02  # 50 Hz command refresh
            try:
                v=get_values(link,args.motor); print_values(args.motor,v)
                if v.fault!=0: return 6
            except TimeoutError:
                pass
            time.sleep(0.05)
    finally:
        link.stop(args.motor)
        link.send_std(args.motor,bytes((COMM_SET_CURRENT,))+be_i32(0))
    return 0


def _measured_rate_hz(timestamps: list[float]) -> float:
    if len(timestamps) < 2:
        return 0.0
    span = timestamps[-1] - timestamps[0]
    return (len(timestamps) - 1) / span if span > 0.0 else 0.0


def cmd_speed_test(link: Link, args: argparse.Namespace) -> int:
    """Active ERPM test with independently measured RT and APP rates."""
    require_yes(args, "Speed test")
    if args.seconds <= 0.0:
        raise ValueError("--seconds harus > 0")

    cal = get_cal(link, False)
    if not cal["valid"]:
        raise RuntimeError("current calibration tidak valid")
    sensor = get_sensor(link, args.motor)
    if not sensor["success"] and not args.force:
        raise RuntimeError(
            "sensor belum lulus auto-detect. Jalankan sensor-detect atau "
            "gunakan --force dengan risiko sendiri."
        )

    initial = get_values(link, args.motor)
    if initial.fault != 0:
        raise RuntimeError(
            f"fault aktif sebelum test: {initial.fault} "
            f"({FAULT_NAMES.get(initial.fault, '?')})"
        )

    rt_period = 1.0 / 50.0
    app_period = 1.0 / 20.0
    payload = command_payload("rpm", args.erpm)
    command_times: list[float] = []
    rt_times: list[float] = []
    app_times: list[float] = []
    rt_jitter_ms: list[float] = []
    app_jitter_ms: list[float] = []
    rows: list[dict] = []
    rt_timeouts = 0
    app_timeouts = 0
    missed_rt_slots = 0
    missed_app_slots = 0
    fault_seen = 0
    latest_app: dict = {}
    latest_values: Optional[Values] = None

    link.clear_fault(args.motor)
    time.sleep(0.05)
    started = time.monotonic()
    stop_at = started + args.seconds
    next_rt = started
    next_app = started

    try:
        while True:
            event_deadline = min(next_rt, next_app)
            if event_deadline >= stop_at:
                break
            now = time.monotonic()
            if now < event_deadline:
                time.sleep(event_deadline - now)
                now = time.monotonic()

            event = "rt" if next_rt <= next_app else "app"
            rt_ok = False
            app_updated = False
            if event == "rt":
                if now - next_rt >= rt_period:
                    skipped = int((now - next_rt) // rt_period)
                    missed_rt_slots += skipped
                    next_rt += skipped * rt_period
                scheduled = next_rt
                rt_jitter_ms.append((now - scheduled) * 1000.0)
                link.send_std(args.motor, payload)
                command_times.append(time.monotonic())
                try:
                    latest_values = get_values(link, args.motor, timeout=0.04)
                    rt_times.append(time.monotonic())
                    rt_ok = True
                except TimeoutError:
                    rt_timeouts += 1
                next_rt += rt_period
            else:
                if now - next_app >= app_period:
                    skipped = int((now - next_app) // app_period)
                    missed_app_slots += skipped
                    next_app += skipped * app_period
                scheduled = next_app
                app_jitter_ms.append((now - scheduled) * 1000.0)
                try:
                    latest_app = get_decoded_adc(link, args.motor, timeout=0.04)
                    app_times.append(time.monotonic())
                    app_updated = True
                except TimeoutError:
                    app_timeouts += 1
                next_app += app_period

            row = {
                "elapsed_s": time.monotonic() - started,
                "scheduled_s": scheduled - started,
                "event": event,
                "jitter_ms": ((rt_jitter_ms[-1] if event == "rt" else
                                app_jitter_ms[-1])),
                "motor": args.motor,
                "target_erpm": args.erpm,
                "rt_ok": int(rt_ok),
                "app_updated": int(app_updated),
                "erpm": latest_values.erpm if latest_values else "",
                "duty": latest_values.duty if latest_values else "",
                "imotor_a": latest_values.imotor if latest_values else "",
                "ibatt_a": latest_values.ibatt if latest_values else "",
                "id_a": latest_values.id if latest_values else "",
                "iq_a": latest_values.iq if latest_values else "",
                "vin_v": latest_values.vin if latest_values else "",
                "fault": latest_values.fault if latest_values else "",
                "app_decoded1": latest_app.get("decoded1", ""),
                "app_voltage1_v": latest_app.get("voltage1", ""),
                "app_decoded2": latest_app.get("decoded2", ""),
                "app_voltage2_v": latest_app.get("voltage2", ""),
            }
            rows.append(row)

            if rt_ok and latest_values is not None and latest_values.fault != 0:
                fault_seen = latest_values.fault
                break
    finally:
        link.stop(args.motor)
        link.send_std(args.motor, bytes((COMM_SET_CURRENT,)) + be_i32(0))

    rt_hz = _measured_rate_hz(rt_times)
    app_hz = _measured_rate_hz(app_times)
    command_hz = _measured_rate_hz(command_times)
    rt_max_jitter = max(rt_jitter_ms, default=0.0)
    rt_mean_jitter = (sum(rt_jitter_ms) / len(rt_jitter_ms)
                      if rt_jitter_ms else 0.0)
    app_max_jitter = max(app_jitter_ms, default=0.0)
    app_mean_jitter = (sum(app_jitter_ms) / len(app_jitter_ms)
                       if app_jitter_ms else 0.0)

    print("\n=== SPEED TEST RATE RESULT ===")
    print(f"motor                   : {'LEFT' if args.motor == 0 else 'RIGHT'}")
    print(f"target_erpm             : {args.erpm}")
    print(f"command_rate_hz         : {command_hz:.2f} (target 50)")
    print(f"rt_data_rate_hz         : {rt_hz:.2f} (target 50)")
    print(f"app_data_rate_hz        : {app_hz:.2f} (target 20)")
    print(f"rt/app_timeouts         : {rt_timeouts} / {app_timeouts}")
    print(f"missed_rt/app_slots     : {missed_rt_slots} / {missed_app_slots}")
    print(f"mean/max_jitter_ms RT   : {rt_mean_jitter:.3f} / {rt_max_jitter:.3f}")
    print(f"mean/max_jitter_ms APP  : {app_mean_jitter:.3f} / {app_max_jitter:.3f}")
    print(f"fault                   : {fault_seen} ({FAULT_NAMES.get(fault_seen, '?')})")

    if args.csv:
        path = Path(args.csv)
        with path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else [])
            if rows:
                writer.writeheader()
                writer.writerows(rows)
        print(f"CSV                     : {path}")

    rates_ok = (command_hz >= args.min_rt_hz and
                rt_hz >= args.min_rt_hz and
                app_hz >= args.min_app_hz)
    if fault_seen != 0:
        print("RESULT: FAIL - motor fault during speed test")
        return 6
    if not rates_ok or rt_timeouts or app_timeouts:
        print("RESULT: FAIL - telemetry rate/timeout requirement not met")
        return 8
    print("RESULT: PASS - RT 50 Hz and APP 20 Hz schedules verified")
    return 0


def cmd_monitor(link: Link,args: argparse.Namespace)->int:
    end=time.monotonic()+args.seconds if args.seconds>0 else math.inf
    period=1/max(1.0,args.hz)
    while time.monotonic()<end:
        t0=time.monotonic()
        for m in (0,1): print_values(m,get_values(link,m))
        print("-")
        dt=time.monotonic()-t0
        if dt<period: time.sleep(period-dt)
    return 0


def cmd_test_all(link: Link,args: argparse.Namespace)->int:
    print("PASS UART/VESC frame: FW response")
    cmd_info(link,args)
    c=get_cal(link,False); print("CAL:",c)
    failures=0
    if not c['done'] or not c['valid']:
        print("FAIL startup current calibration"); failures+=1
    for m in (0,1):
        try:
            v=get_values(link,m); print_values(m,v)
            e=get_extended(link,m); s=get_sensor(link,m)
            print(" EXT:",e); print(" SENSOR:",s)
            if v.fault: failures+=1
            if c['valid'] and abs(v.id)>args.zero_limit and abs(v.iq)>args.zero_limit:
                print("WARN residual Id/Iq tinggi saat passive test")
        except Exception as exc:
            print(f"FAIL motor {m}: {exc}"); failures+=1
    print("PASS passive test-all" if failures==0 else f"FAILURES={failures}")
    return 0 if failures==0 else 7



def _wait_sensor_done(link: Link, motor: int, timeout: float) -> dict:
    deadline=time.monotonic()+timeout
    last=None
    while time.monotonic()<deadline:
        d=get_sensor(link,motor)
        if d['state'] != last:
            print(f"M{motor} detect: {DETECT_NAMES.get(d['state'], d['state'])} mode={SENSOR_NAMES.get(d['mode'],d['mode'])}")
            last=d['state']
        if d['state'] in (DETECT_DONE,DETECT_FAILED):
            return d
        time.sleep(0.1)
    raise TimeoutError(f"sensor detect M{motor} timeout")

def _run_command_stage(link: Link, motor: int, mode: str, value: float, seconds: float) -> Values:
    payload=command_payload(mode,value)
    end=time.monotonic()+seconds
    next_cmd=0.0
    last=None
    try:
        while time.monotonic()<end:
            now=time.monotonic()
            if now>=next_cmd:
                link.send_std(motor,payload)
                next_cmd=now+0.02
            try:
                last=get_values(link,motor)
                print_values(motor,last)
                if last.fault != 0:
                    raise RuntimeError(f"fault {last.fault} {FAULT_NAMES.get(last.fault,'?')}")
            except TimeoutError:
                pass
            time.sleep(0.05)
    finally:
        link.stop(motor)
        time.sleep(0.05)
    if last is None:
        raise TimeoutError(f"tidak ada telemetry saat stage {mode} M{motor}")
    return last

def cmd_full_test(link: Link,args: argparse.Namespace)->int:
    require_yes(args,"FULL ACTIVE COMMISSIONING TEST")
    print("=== 0. STOP + CLEAR FAULT ===")
    for m in (0,1):
        link.stop(m); link.clear_fault(m)
    time.sleep(0.1)

    print("=== 1. CURRENT ZERO CALIBRATION ===")
    cal_args=argparse.Namespace(timeout=args.cal_timeout,zero_limit=args.zero_limit)
    rc=cmd_calibrate(link,cal_args)
    if rc != 0:
        return rc

    print("=== 2. RUNTIME SENSOR AUTO-DETECT ===")
    for m in (0,1):
        link.clear_fault(m)
        link.send(bytes((COMM_CUSTOM_APP_DATA,CUSTOM_SENSOR_DETECT,m,SENSOR_AUTO)))
        d=_wait_sensor_done(link,m,args.detect_timeout)
        if not d['success'] or d['state'] != DETECT_DONE:
            print(f"FAIL sensor auto-detect M{m}: {d}")
            return 8
        print(f"PASS M{m}: sensor={SENSOR_NAMES.get(d['mode'],d['mode'])}, pole_pairs={d['pole_pairs']}")

    print("=== 3. PASSIVE TELEMETRY ===")
    for m in (0,1):
        v=get_values(link,m); e=get_extended(link,m)
        print_values(m,v); print(" EXT:",e)
        if v.fault != 0:
            return 9

    print("=== 4. FAST ISR SAMPLE BUFFER ===")
    for m in (0,1):
        tmp=argparse.Namespace(motor=m,count=64,decimation=max(1,args.sample_decimation),timeout=5.0,csv=None)
        rc=cmd_sample(link,tmp)
        if rc != 0:
            return rc

    print("=== 5. LOW CURRENT + / - ===")
    for m in (0,1):
        for val in (abs(args.current),-abs(args.current)):
            print(f"M{m} CURRENT {val:+.3f} A")
            _run_command_stage(link,m,"current",val,args.stage_seconds)

    if not args.skip_rpm:
        print("=== 6. LOW RPM + / - ===")
        for m in (0,1):
            for val in (abs(args.erpm),-abs(args.erpm)):
                print(f"M{m} RPM {val:+.0f} eRPM")
                _run_command_stage(link,m,"rpm",val,args.stage_seconds)

    if args.position_step != 0.0:
        print("=== 7. POSITION (LEFT only when encoder selected) ===")
        sd=get_sensor(link,0)
        if sd['mode']==SENSOR_ENCODER and sd['success']:
            v0=get_values(link,0)
            target=(v0.position+args.position_step)%360.0
            print(f"LEFT POS {v0.position:.2f} -> {target:.2f} deg")
            _run_command_stage(link,0,"position",target,args.stage_seconds)
        else:
            print("SKIP position: LEFT sensor hasil detect bukan encoder")

    print("=== 8. FINAL STOP / FAULT CHECK ===")
    failures=0
    for m in (0,1):
        link.stop(m); time.sleep(0.05)
        v=get_values(link,m); print_values(m,v)
        if v.fault != 0: failures += 1
    if failures:
        print(f"FULL TEST FAIL: faults={failures}")
        return 10
    print("FULL ACTIVE TEST PASS (protocol/firmware-level; verify waveforms/current physically with scope/clamp)")
    return 0

def parse_ping_can(p: bytes) -> list[int]:
    if not p or p[0] != COMM_PING_CAN:
        raise ValueError("bukan COMM_PING_CAN")
    return list(p[1:])


def cmd_can_scan(link: Link, _args: argparse.Namespace) -> int:
    p=link.request(bytes((COMM_PING_CAN,)), lambda x: bool(x) and x[0]==COMM_PING_CAN, 2.0)
    devs=parse_ping_can(p)
    print("VESC Tool-compatible device scan:", devs)
    if devs != [2]:
        raise ValueError(f"expected exactly local motor-2 ID [2], got {devs}")
    print("PASS: direct controller=M1/ID1, discovered local forwarded controller=M2/ID2")
    return 0


def cmd_vesc_tool_dual_basic(link: Link, _args: argparse.Namespace) -> int:
    print("=== VESC TOOL DUAL BASIC PASSIVE CHECK ===")
    # Direct/root controller.
    fw1=link.request(bytes((COMM_FW_VERSION,)), lambda x: bool(x) and x[0]==COMM_FW_VERSION, 2.0)
    i1=parse_fw_version(fw1)
    if (i1['major'],i1['minor']) != (6,0):
        raise ValueError(f"root FW ABI {i1['major']}.{i1['minor']} != 6.00")
    v1=get_values(link,0)
    if v1.controller_id != 1:
        raise ValueError(f"root GET_VALUES controller_id={v1.controller_id}, expected 1")

    # Discovery is exactly what VESC Tool CAN Scan uses.
    ping=link.request(bytes((COMM_PING_CAN,)), lambda x: bool(x) and x[0]==COMM_PING_CAN, 2.0)
    devs=parse_ping_can(ping)
    if devs != [2]:
        raise ValueError(f"device scan expected [2], got {devs}")

    # VESC Tool switches to the discovered node by wrapping requests in
    # COMM_FORWARD_CAN, ID 2. Replies remain normal inner replies.
    fw2=link.request_std(1,bytes((COMM_FW_VERSION,)), lambda x: bool(x) and x[0]==COMM_FW_VERSION, 2.0)
    i2=parse_fw_version(fw2)
    if (i2['major'],i2['minor']) != (6,0):
        raise ValueError(f"M2 FW ABI {i2['major']}.{i2['minor']} != 6.00")
    if i2['uuid'] == i1['uuid']:
        raise ValueError("M1/M2 UUID must differ for VESC Tool backup/identity")
    v2=get_values(link,1)
    if v2.controller_id != 2:
        raise ValueError(f"forwarded GET_VALUES controller_id={v2.controller_id}, expected 2")

    # Both configuration read paths must be independently addressable.
    for motor,expected in ((0,1),(1,2)):
        mc_payload=link.request_std(motor,bytes((COMM_GET_MCCONF,)),lambda x: bool(x) and x[0]==COMM_GET_MCCONF,2.0)
        _validate_vesc6_wire(f"M{motor+1} MCCONF",mc_payload,COMM_GET_MCCONF,VESC6_MCCONF_WIRE_SIZE,VESC6_MCCONF_SIGNATURE)
        app_payload=link.request_std(motor,bytes((COMM_GET_APPCONF,)),lambda x: bool(x) and x[0]==COMM_GET_APPCONF,2.0)
        app=_validate_vesc6_wire(f"M{motor+1} APPCONF",app_payload,COMM_GET_APPCONF,VESC6_APPCONF_WIRE_SIZE,VESC6_APPCONF_SIGNATURE)
        if app[4] != expected:
            raise ValueError(f"M{motor+1} APPCONF controller_id={app[4]}, expected {expected}")

    print(f"M1 FW/UUID/ID : PASS  {i1['major']}.{i1['minor']}  {i1['uuid']}  ID=1")
    print(f"SCAN          : PASS  remote IDs={devs}")
    print(f"M2 FW/UUID/ID : PASS  {i2['major']}.{i2['minor']}  {i2['uuid']}  ID=2")
    print("M1/M2 VALUES  : PASS")
    print("M1/M2 MCCONF  : PASS")
    print("M1/M2 APPCONF : PASS")
    print("PASS: basic VESC Tool dual-controller discovery/routing path is coherent")
    return 0


def cmd_motor2_forward(link: Link, _args: argparse.Namespace) -> int:
    # Board ini tidak memiliki CAN PHY. COMM_FORWARD_CAN ID 2 dipakai seperti
    # cabang local second-motor pada VESC dual-motor: request diproses oleh
    # runtime motor kanan di MCU yang sama. Motor-2 tetap diiklankan ke VESC
    # Tool melalui COMM_PING_CAN agar muncul pada device scan.
    fw=link.request_std(1,bytes((COMM_FW_VERSION,)),lambda x: bool(x) and x[0]==COMM_FW_VERSION,1.0)
    d=parse_fw_version(fw)
    print("RIGHT FW via local COMM_FORWARD_CAN ID 2:",d)
    return 0


def parse_config_status(p: bytes) -> dict:
    if len(p)<4 or p[:2]!=bytes((COMM_CUSTOM_APP_DATA,CUSTOM_CONFIG_STATUS)):
        raise ValueError("bukan config status")
    r=Reader(p,2)
    return {"valid":bool(r.u8()),"last_save_ok":bool(r.u8()),"sequence":r.u32(),"timeout_ms":r.u32()}


def cmd_config_status(link: Link, _args: argparse.Namespace) -> int:
    p=link.request(bytes((COMM_CUSTOM_APP_DATA,CUSTOM_CONFIG_STATUS)),
                   lambda x: len(x)>=2 and x[:2]==bytes((COMM_CUSTOM_APP_DATA,CUSTOM_CONFIG_STATUS)),1.0)
    print(parse_config_status(p)); return 0


def cmd_config_save(link: Link, _args: argparse.Namespace) -> int:
    p=link.request(bytes((COMM_CUSTOM_APP_DATA,CUSTOM_CONFIG_SAVE)),
                   lambda x: len(x)>=2 and x[:2]==bytes((COMM_CUSTOM_APP_DATA,CUSTOM_CONFIG_STATUS)),2.0)
    d=parse_config_status(p); print(d); return 0 if d['last_save_ok'] else 3


def self_test() -> int:
    payload=bytes((COMM_SET_CURRENT,))+be_i32(1234)
    fr=frame(payload)
    parser=FrameParser(); out=[]
    for chunk in (fr[:2],fr[2:5],fr[5:]): out.extend(parser.feed(chunk))
    assert out==[payload]
    assert crc16(b"123456789")==0x31C3
    assert Link.standard_route(0,payload)==payload
    assert Link.standard_route(1,payload)==bytes((COMM_FORWARD_CAN,2))+payload
    assert parse_ping_can(bytes((COMM_PING_CAN,2))) == [2]
    assert parse_ping_can(bytes((COMM_PING_CAN,))) == []
    assert abs(_measured_rate_hz([i * 0.02 for i in range(100)]) - 50.0) < 1e-9
    assert abs(_measured_rate_hz([i * 0.05 for i in range(40)]) - 20.0) < 1e-9
    fw=(bytes((COMM_FW_VERSION,6,0))+b"HOVERBOARD_DUAL_FOC\x00"+bytes(range(12))+
        bytes((1,0,0,0,0,0,0,0))+b"vesc-f103-hoverboard-v24\x00")
    fwd=parse_fw_version(fw)
    assert fwd["major"]==6 and fwd["minor"]==0
    assert fwd["hw_name"]=="HOVERBOARD_DUAL_FOC"
    assert fwd["fw_name"].endswith("v24") and fwd["hw_crc"] is None

    mc_wire=struct.pack(">I",VESC6_MCCONF_SIGNATURE)+bytes(VESC6_MCCONF_WIRE_SIZE-4)
    app_wire=struct.pack(">I",VESC6_APPCONF_SIGNATURE)+bytes((1,))+bytes(VESC6_APPCONF_WIRE_SIZE-5)
    assert _validate_vesc6_wire("MCCONF-selftest",bytes((COMM_GET_MCCONF,))+mc_wire,
                                COMM_GET_MCCONF,VESC6_MCCONF_WIRE_SIZE,VESC6_MCCONF_SIGNATURE)==mc_wire
    assert _validate_vesc6_wire("APPCONF-selftest",bytes((COMM_GET_APPCONF,))+app_wire,
                                COMM_GET_APPCONF,VESC6_APPCONF_WIRE_SIZE,VESC6_APPCONF_SIGNATURE)==app_wire

    vals=[1.0,-2.0,3.25,4.0,5.0,6.0,7.0,8.0,16000.0]
    samp=bytes((COMM_SAMPLE_PRINT,))+be_i16(3)+b''.join(struct.pack(">f",x) for x in vals)+bytes((0,123))+be_i32(3)
    idx,row=parse_sample_packet(samp)
    assert idx==3 and abs(row['current1']+2.0)<1e-6 and row['index_full']==3

    adc_payload = bytes((COMM_GET_DECODED_ADC,)) + b"".join(
        be_i32(v) for v in (250_000, 825_000, 750_000, 2_475_000)
    )
    adc = parse_decoded_adc(adc_payload)
    assert adc == {
        "decoded1": 0.25, "voltage1": 0.825,
        "decoded2": 0.75, "voltage2": 2.475,
    }

    # Current-cal revision 17 remains exactly within the 512-byte firmware
    # payload and exposes robust-filter/MOE state without changing its prefix.
    cpay = bytearray((COMM_CUSTOM_APP_DATA, CUSTOM_CURRENT_CAL, 1, 1))
    cpay += struct.pack(">II", 6096, 6096)
    for value in (2669, 2659, 1954, 2616, 2604, 1985):
        cpay += be_i32(value)
    cpay += struct.pack(">IHH", 1_659_009, 6, 0)
    cpay += bytes((0, 1, 1, 0, 1, 1, 1))
    cpay += bytes((17,)) + struct.pack(">HHH", 0x18, 0, 0)
    channel_values = (
        (2669, 2631, 4074, 625_300),
        (2659, 2344, 4076, 977_600),
        (1954, 1948, 1961, 286),
        (2616, 2587, 2941, 31_260),
        (2604, 2575, 2929, 31_220),
        (1985, 1979, 1990, 397),
    )
    for mean, minimum, maximum, variance in channel_values:
        cpay += be_i32(mean) + be_u16(minimum) + be_u16(maximum)
        cpay += struct.pack(">I", variance)
    for value in range(24):
        cpay += struct.pack(">I", value)
    for value in range(6):
        cpay += struct.pack(">I", 0x08000800 + value)
    cpay += bytes((8,)) + be_u16(0)
    for value in (2680, 2672, 1954, 2632, 2620, 1984,
                  2669, 2659, 1954, 2616, 2604, 1985):
        cpay += be_i32(value)
    cpay += bytes((0, 0, 0, 8))
    cpay += be_u16(0) + be_u16(0) + be_u16(0)
    for _ in range(3):
        cpay += be_i32(0)
    for _ in range(6):
        cpay += be_i32(0)
    for _ in range(5):
        cpay += be_u16(0)
    cpay += struct.pack(">I", 0)
    cpay += be_u16(8) + bytes((0, 0, 0, 0)) + be_u16(120)
    cpay += struct.pack(">IIHHH", 16_000, 4_000, 1, 1900, 1767)
    cpay += bytes((1, 1)) + be_u16(1) + be_u16(1668) + be_u16(1667)
    cpay += bytes((0, 0)) + struct.pack(">I", 0)
    for value in range(7):
        cpay += struct.pack(">I", value)
    for value in (4, 5, 0, 2, 3, 0):
        cpay += be_u16(value)
    cpay += bytes((0, 3))
    for value in (1000, 4000, 2000, 5000, 2100, 5100):
        cpay += struct.pack(">I", value)
    assert len(cpay) == 463
    cal17 = parse_cal(bytes(cpay))
    assert cal17["cal_diag_revision"] == 17
    assert cal17["outlier_count"]["left_u"] == 4
    assert cal17["outlier_count"]["right_v"] == 3
    assert cal17["moe_confirmed_mask"] == 3
    assert cal17["moe_request_adc"] == [1000, 4000]
    assert cal17["moe_confirm_adc"] == [2000, 5000]
    assert cal17["first_sample_adc"] == [2100, 5100]

    # V21 sensor/observer diagnostics ABI: verify the appended PWM/ADC scheduling fields
    # independently of serial hardware so a stale debug parser cannot hide a
    # terminal detect state again.
    spay=bytearray((COMM_CUSTOM_APP_DATA,CUSTOM_SENSOR_INFO,0,1,SENSOR_HALL,0,DETECT_FAILED,0,15,0))
    spay += be_u16(0)
    spay += bytes((255,0,133,166,66,33,100,255))
    for a in (0,0,43688,54610,21844,10922,32766,0): spay += be_u16(a)
    spay += bytes((15,))
    spay += be_i32(300)+be_i32(0)+be_i32(0)+be_i32(0)
    for _ in range(8): spay += be_i32(0)
    spay += bytes((255,255,255,255,255,255,255,255))
    spay += be_u16(2773)+be_u16(2702)+be_u16(1948)
    spay += be_i32(0)+be_i32(0)+be_i32(0)
    spay += bytes((0,0))+be_i32(20000)
    spay += bytes((2,))+be_u16(8)+be_u16(120)+be_u16(3)+be_u16(1)+be_u16(120)
    sd=parse_sensor(bytes(spay))
    assert sd['state']==DETECT_FAILED and sd['diag_revision']==15
    assert sd['pwm_enable_pending_events']==2 and sd['pwm_enable_blank_cycles']==8
    assert sd['tim8_rcr']==1 and sd['adc_motor_phase_offset_ticks']==120

    # Extended telemetry revision 6: phase currents are inserted before dq.
    epay=bytearray((COMM_CUSTOM_APP_DATA,CUSTOM_EXT_TELEMETRY,0,6,1,SENSOR_HALL,0,0,1,1,5,15,0))
    for v in (1100,-2200,1100): epay += be_i32(v)
    core=(300,400,310,390,5000,6000,700,800,900,1000,1100,1200,48000,12345)
    for v in core: epay += be_i32(v)
    for v in (2000,2001,2002): epay += be_i32(v)
    for v in (100,1000,7200,3): epay += struct.pack(">I",v)
    epay += be_i32(42)+be_u16(1234)
    epay += bytes((1,1,1))
    for v in (45000,1234,99000,25000,50000,1000,35000,1800,2500,3000,600): epay += be_i32(v)
    epay += struct.pack(">II",16000,1000)
    ed=parse_extended(bytes(epay))
    assert ed['revision']==6 and abs(ed['phase_current_a']-1.1)<1e-9
    assert abs(ed['phase_current_b']+2.2)<1e-9 and abs(ed['phase_current_c']-1.1)<1e-9
    assert abs(ed['id']-0.3)<1e-9 and abs(ed['vd']-5.0)<1e-9 and abs(ed['vq']-6.0)<1e-9
    assert ed['observer_valid'] and ed['encoder_synced'] and ed['current_loop_hz']==16000

    # Standard Hall/encoder detector reply ABI. These commands are deliberately
    # non-auto-apply: parsers only expose the result returned to VESC Tool.
    encp=bytes((COMM_DETECT_ENCODER,))+be_i32(12_500_000)+be_i32(15_000_000)+bytes((1,))
    encd=parse_standard_encoder_detect(encp)
    assert encd['ok'] and abs(encd['offset_deg']-12.5)<1e-9 and encd['inverted']
    encfail=parse_standard_encoder_detect(bytes((COMM_DETECT_ENCODER,))+be_i32(1_001_000_000)+be_i32(0)+bytes((0,)))
    assert not encfail['ok']
    hallp=bytes((COMM_DETECT_HALL_FOC,255,0,33,66,100,133,166,255,1))
    halld=parse_standard_hall_detect(hallp)
    assert halld['ok'] and sum(1 for x in halld['hall_table'] if x!=255)==6

    # COMM_DIAG revision 7: local motor-2 forwarding is distinct from physical CAN.
    dpay=bytearray((COMM_CUSTOM_APP_DATA,CUSTOM_COMM_DIAG,7))
    for v in (11,12,13,14,15,16,17,18,19,20,21,115200): dpay += struct.pack(">I",v)
    dpay += bytes((2,1,1))
    dd=parse_comm_diag(bytes(dpay))
    assert dd['motor2_forwards']==20 and dd['unsupported_forward_ids']==21 and dd['baud']==115200

    d9=bytearray((COMM_CUSTOM_APP_DATA,CUSTOM_COMM_DIAG,9))
    for v in (11,12,13,14,15,16,17,18,19,20,21,115200): d9 += struct.pack(">I",v)
    d9 += bytes((2,1,1))
    for v in (31,32,33,34,35): d9 += struct.pack(">I",v)
    d9 += bytes((1,1,1,1)); d9 += struct.pack(">II",41,42); d9 += bytes((1,0))
    for v in (51,52,53,54): d9 += struct.pack(">I",v)
    dd9=parse_comm_diag(bytes(d9))
    assert dd9['config_integrity_ok'] and dd9['config_integrity_checks']==41 and dd9['config_integrity_failures']==42
    assert dd9['power_hold'] and not dd9['shutdown_latched'] and dd9['heartbeat_comm']==54

    d10=bytearray((COMM_CUSTOM_APP_DATA,CUSTOM_COMM_DIAG,10))
    for v in (11,12,13,14,15,16,17,18,19,20,21,115200): d10 += struct.pack(">I",v)
    d10 += bytes((2,1,1))
    for v in (31,32,33,34,35): d10 += struct.pack(">I",v)
    d10 += bytes((1,1,1,1)); d10 += struct.pack(">II",41,42); d10 += bytes((1,0))
    for v in (51,52,53,54): d10 += struct.pack(">I",v)
    for v in (61,62,63,64,65,66): d10 += struct.pack(">I",v)
    d10 += struct.pack(">HH",777,888)
    dd10=parse_comm_diag(bytes(d10))
    assert dd10['watchdog_unhealthy_mask']==61 and dd10['watchdog_miss_comm']==64
    assert dd10['sample_clamp_left']==65 and dd10['sample_clamp_right']==66
    assert dd10['sample_margin_left_q15']==777 and dd10['sample_margin_right_q15']==888

    d14=bytearray((COMM_CUSTOM_APP_DATA,CUSTOM_COMM_DIAG,14))
    for v in (11,12,13,14,15,16,17,18,19,20,21,115200): d14 += struct.pack(">I",v)
    d14 += bytes((2,1,1))
    for v in (31,32,33,34,35): d14 += struct.pack(">I",v)
    d14 += bytes((1,1,1,1)); d14 += struct.pack(">II",41,42); d14 += bytes((1,0))
    for v in (51,52,53,54,55): d14 += struct.pack(">I",v)
    for v in (61,62,63,64,65): d14 += struct.pack(">I",v)
    d14 += bytes((2,))
    d14 += struct.pack(">IIHH",66,67,777,888)
    d14 += struct.pack(">HHHHhhh",1000,2000,1100,2200,100,-200,300)
    d14 += bytes((0,1,1,0,1,2))
    for v in (0,1234,9,3990,4010,4096,3072): d14 += struct.pack(">I",v)
    d14 += struct.pack(">HHHHHHH",500,400,300,200,600,700,3)
    d14 += struct.pack(">II",4,5)
    dd14=parse_comm_diag(bytes(d14))
    assert dd14['sampling_contract_flags']==0 and dd14['isr_total_max_cycles']==1234
    assert dd14['heap_min_ever_bytes']==3072 and dd14['tx_queue_high_water']==3
    assert dd14['tx_queue_busy_drops']==4 and dd14['tx_low_priority_drops']==5

    print("SELF-TEST PASS: CRC/framing, VESC-6.00 FW/config, local-ID2 forwarding, "
          "sample/ADC parser, current-cal-v17 robust/MOE ABI, EXT-v6 telemetry, "
          "COMM_DIAG-v14 resources, sensor/observer diagnostics")
    return 0

def add_live_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--port",default="/dev/ttyUSB0")
    p.add_argument("--baud",type=int,default=115200)


def main() -> int:
    ap=argparse.ArgumentParser(description="STM32F103 dual VESC-like FOC debug/commissioning")
    ap.add_argument("--self-test",action="store_true",help="test Python protocol code without serial hardware")
    sub=ap.add_subparsers(dest="cmd")

    def sp(name,func,help):
        p=sub.add_parser(name,help=help); add_live_args(p); p.set_defaults(func=func); return p

    sp("info",cmd_info,"read and fully parse VESC firmware version")
    p=sp("vesc-tool-check",cmd_vesc_tool_check,"verify VESC Tool 6.00 FW + MCCONF/APPCONF read path; optional byte-exact write-back")
    p.add_argument("--motor",type=int,choices=[0,1],default=0)
    p.add_argument("--timeout",type=float,default=2.0)
    p.add_argument("--write-back",action="store_true",help="also SET the exact read config and require ACK/readback")
    p.add_argument("--write-timeout",type=float,default=5.0)
    p.add_argument("--yes",action="store_true",help="required with --write-back")
    p=sp("handshake",cmd_handshake,"raw COMM_FW_VERSION TX/RX diagnostic"); p.add_argument("--timeout",type=float,default=0.5); p.add_argument("--attempts",type=int,default=5)
    p=sp("baud-scan",cmd_baud_scan,"scan common UART baud rates for VESC handshake"); p.add_argument("--bauds",default="115200,230400,250000,460800,921600"); p.add_argument("--timeout",type=float,default=0.6)
    sp("comm-diag",cmd_comm_diag,"read firmware USART3 IRQ/ring diagnostics")
    sp("can-scan",cmd_can_scan,"VESC Tool-compatible COMM_PING_CAN scan; must discover local motor-2 ID 2")
    sp("vesc-tool-dual-basic",cmd_vesc_tool_dual_basic,"passive end-to-end VESC Tool dual-controller discovery/routing/config check")
    sp("motor2-forward",cmd_motor2_forward,"verify local second-motor forwarding ID 2 with FW_VERSION")
    sp("config-status",cmd_config_status,"read flash-emulated persistent config status")
    sp("config-save",cmd_config_save,"save runtime Hall/encoder/PID/timeout config to flash")
    p=sp("status",cmd_status,"standard VESC telemetry + extended telemetry"); p.add_argument("--motor",type=int,choices=[0,1])
    p=sp("monitor",cmd_monitor,"monitor both motors"); p.add_argument("--hz",type=float,default=5); p.add_argument("--seconds",type=float,default=10)
    p=sp("calibrate",cmd_calibrate,"re-run zero-current calibration and write TXT only"); p.add_argument("--timeout",type=float,default=8); p.add_argument("--zero-limit",type=float,default=0.30); p.add_argument("--out",help="TXT output path; default calibration_debug_TIMESTAMP.txt")
    p=sp("diagnose",cmd_diagnose,"full passive diagnostic; writes one TXT report"); p.add_argument("--timeout",type=float,default=8); p.add_argument("--zero-limit",type=float,default=0.30); p.add_argument("--out",help="TXT output path; default vesc_f103_full_debug_TIMESTAMP.txt")
    p=sp("sensor-info",cmd_sensor_info,"show runtime Hall/encoder state"); p.add_argument("--motor",type=int,choices=[0,1])
    p=sp("sensor-select",cmd_sensor_select,"live switch Hall/encoder while stopped"); p.add_argument("--motor",type=int,choices=[0,1],required=True); p.add_argument("--mode",choices=["hall","encoder"],required=True)
    p=sp("detect-encoder",cmd_detect_encoder_standard,"ACTIVE standard VESC COMM_DETECT_ENCODER; does not auto-apply"); p.add_argument("--motor",type=int,choices=[0],default=0); p.add_argument("--current",type=float,default=0.5); p.add_argument("--timeout",type=float,default=35.0); p.add_argument("--yes",action="store_true")
    p=sp("detect-hall",cmd_detect_hall_standard,"ACTIVE standard VESC COMM_DETECT_HALL_FOC; does not auto-apply"); p.add_argument("--motor",type=int,choices=[0,1],required=True); p.add_argument("--current",type=float,default=0.5); p.add_argument("--timeout",type=float,default=25.0); p.add_argument("--yes",action="store_true")
    p=sp("sensor-detect",cmd_sensor_detect,"VESC-style blocking Hall/encoder detect; writes TXT"); p.add_argument("--motor",type=int,choices=[0,1],required=True); p.add_argument("--mode",choices=["auto","hall","encoder"],default="auto"); p.add_argument("--current",type=float,default=0.5,help="diagnostic detect current in A (0.2..2.0, default 0.5)"); p.add_argument("--timeout",type=float,default=25); p.add_argument("--out",help="TXT output path; default vesc_f103_sensor_detect_TIMESTAMP.txt"); p.add_argument("--yes",action="store_true")
    p=sp("rotor",cmd_rotor,"stream COMM_ROTOR_POSITION"); p.add_argument("--motor",type=int,choices=[0,1],required=True); p.add_argument("--mode",choices=list(DISPLAY_MODES),default="encoder"); p.add_argument("--seconds",type=float,default=5)
    p=sp("sample",cmd_sample,"capture fast FOC samples from ISR buffer"); p.add_argument("--motor",type=int,choices=[0,1],required=True); p.add_argument("--count",type=int,default=128); p.add_argument("--decimation",type=int,default=8); p.add_argument("--timeout",type=float,default=8); p.add_argument("--csv"); p.add_argument("--raw",action="store_true")
    p=sp("detect-rl",cmd_detect_rl,"ACTIVE standard VESC FOC R/L detection"); p.add_argument("--motor",type=int,choices=[0,1],required=True); p.add_argument("--timeout",type=float,default=15.0); p.add_argument("--yes",action="store_true")
    p=sp("detect-flux",cmd_detect_flux,"ACTIVE standard VESC flux-linkage detection"); p.add_argument("--motor",type=int,choices=[0,1],required=True); p.add_argument("--current",type=float,default=1.0); p.add_argument("--resistance",type=float,required=True); p.add_argument("--min-erpm",type=float,default=1800.0); p.add_argument("--duty",type=float,default=0.2); p.add_argument("--openloop",action="store_true"); p.add_argument("--erpm-per-sec",type=float,default=2000.0); p.add_argument("--inductance",type=float,default=0.00005); p.add_argument("--timeout",type=float,default=20.0); p.add_argument("--yes",action="store_true")
    p=sp("detect-all-foc",cmd_detect_all_foc,"ACTIVE standard VESC Detect All FOC and persist result"); p.add_argument("--motor",type=int,choices=[0,1],required=True); p.add_argument("--max-power-loss",type=float,default=50.0); p.add_argument("--min-input-current",type=float,default=-20.0); p.add_argument("--max-input-current",type=float,default=20.0); p.add_argument("--openloop-erpm",type=float,default=3000.0); p.add_argument("--sl-erpm",type=float,default=2500.0); p.add_argument("--timeout",type=float,default=60.0); p.add_argument("--yes",action="store_true")
    p=sp("motor-test",cmd_motor_test,"active current/brake/rpm/duty/position test at 50 Hz command refresh"); p.add_argument("--motor",type=int,choices=[0,1],required=True); p.add_argument("--mode",choices=["current","brake","rpm","duty","position"],required=True); p.add_argument("--value",type=float,required=True); p.add_argument("--seconds",type=float,default=2); p.add_argument("--yes",action="store_true"); p.add_argument("--force",action="store_true")
    p=sp("speed-test",cmd_speed_test,"ACTIVE ERPM test with measured RT 50 Hz and APP ADC 20 Hz")
    p.add_argument("--motor",type=int,choices=[0,1],required=True)
    p.add_argument("--erpm",type=float,default=300.0)
    p.add_argument("--seconds",type=float,default=5.0)
    p.add_argument("--min-rt-hz",type=float,default=45.0)
    p.add_argument("--min-app-hz",type=float,default=18.0)
    p.add_argument("--csv",help="optional per-cycle CSV output")
    p.add_argument("--yes",action="store_true")
    p.add_argument("--force",action="store_true")
    p=sp("test-all",cmd_test_all,"passive end-to-end firmware/telemetry test"); p.add_argument("--zero-limit",type=float,default=0.30)
    p=sp("full-test",cmd_full_test,"ACTIVE commissioning: calibration, auto-detect, samples, current +/- and optional RPM/position"); p.add_argument("--yes",action="store_true"); p.add_argument("--current",type=float,default=0.5); p.add_argument("--erpm",type=float,default=300.0); p.add_argument("--stage-seconds",type=float,default=1.0); p.add_argument("--cal-timeout",type=float,default=5.0); p.add_argument("--detect-timeout",type=float,default=15.0); p.add_argument("--zero-limit",type=float,default=0.30); p.add_argument("--sample-decimation",type=int,default=8); p.add_argument("--skip-rpm",action="store_true"); p.add_argument("--position-step",type=float,default=5.0)

    args=ap.parse_args()
    if args.self_test:
        return self_test()
    if not args.cmd:
        ap.print_help(); return 1
    link=Link(args.port,args.baud)
    try:
        return int(args.func(link,args))
    except KeyboardInterrupt:
        try: link.stop(0); link.stop(1)
        except Exception: pass
        print("\nStopped")
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}",file=sys.stderr)
        return 1
    finally:
        link.close()


if __name__=="__main__":
    raise SystemExit(main())
