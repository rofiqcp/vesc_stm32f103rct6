#!/usr/bin/env python3
"""Debug/commissioning tool for STM32F103 dual-FOC firmware.

Only pyserial is required for live use:
    python3 -m pip install pyserial

Passive tests never command PWM. Motor-moving tests require --yes explicitly.
"""
from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
import time
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
COMM_SAMPLE_PRINT = 19
COMM_ROTOR_POSITION = 22
COMM_ALIVE = 30
COMM_FORWARD_CAN = 34
COMM_CUSTOM_APP_DATA = 36
COMM_PING_CAN = 62
COMM_SET_CURRENT_REL = 84

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
    4: "HALL_REV", 5: "HALL_EVAL", 6: "ENC_PREP", 7: "ENC_LOCK",
    8: "ENC_SWEEP", 9: "ENC_EVAL", 10: "DONE", 11: "FAILED",
}
DISPLAY_MODES = {
    "stop": 0, "inductance": 1, "observer": 2, "encoder": 3,
    "pid": 4, "pid-error": 5, "obs-enc": 6, "obs-hall": 7,
}

FAULT_NAMES = {
    0: "NONE", 1: "OVER_VOLTAGE", 2: "UNDER_VOLTAGE", 3: "DRV/HW",
    4: "ABS_OVER_CURRENT", 15: "CURRENT_OFFSET", 27: "SENSOR/ENCODER",
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


def get_values(link: Link, motor: int) -> Values:
    p = link.request_std(motor, bytes((COMM_GET_VALUES,)),
                         lambda x: bool(x) and x[0] == COMM_GET_VALUES)
    return parse_values(p)


def parse_cal(p: bytes) -> dict:
    if len(p) < 2 or p[:2] != bytes((COMM_CUSTOM_APP_DATA, CUSTOM_CURRENT_CAL)):
        raise ValueError("bukan current-cal reply")
    r=Reader(p,2)
    return {
        "done": bool(r.u8()), "valid": bool(r.u8()), "count": r.u32(), "target": r.u32(),
        "left_u": r.i32(), "left_v": r.i32(), "left_dc": r.i32(),
        "right_u": r.i32(), "right_v": r.i32(), "right_dc": r.i32(),
    }


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
    return {"motor":motor,"controller_id":controller_id,"mode":mode,"request":request,"state":state,"success":success,"pole_pairs":pp,
            "encoder_inverted":inv,"encoder_offset_u16":off,"hall_table":hall,"hall_angles_u16":angles}


def get_sensor(link: Link, motor: int) -> dict:
    p=link.request(bytes((COMM_CUSTOM_APP_DATA,CUSTOM_SENSOR_INFO,motor)),
                   lambda x: len(x)>=3 and x[:2]==bytes((COMM_CUSTOM_APP_DATA,CUSTOM_SENSOR_INFO)) and x[2]==motor,1.0)
    return parse_sensor(p)


def parse_extended(p: bytes) -> dict:
    if len(p)<2 or p[:2] != bytes((COMM_CUSTOM_APP_DATA,CUSTOM_EXT_TELEMETRY)):
        raise ValueError("bukan extended telemetry")
    r=Reader(p,2)
    d={}
    d["motor"]=r.u8(); d["revision"]=r.u8(); d["controller_id"]=r.u8(); d["sensor_mode"]=r.u8(); d["native_fault"]=r.u8(); d["detect_state"]=r.u8()
    d["cal_done"]=bool(r.u8()); d["cal_valid"]=bool(r.u8()); d["hall_raw"]=r.u8(); d["pole_pairs"]=r.u8(); d["encoder_inverted"]=bool(r.u8())
    names_scales=[("id",1000),("iq",1000),("id_filter",1000),("iq_filter",1000),("vd",1000),("vq",1000),
                  ("imotor",1000),("ibatt",1000),("erpm",1),("mech_rpm",10),("position_deg",1000),("rotor_elec_deg",1000),
                  ("vbus",1000),("duty",100000)]
    for name,scale in names_scales: d[name]=r.i32()/scale
    d["offset_u"]=r.i32(); d["offset_v"]=r.i32(); d["offset_dc"]=r.i32()
    d["cal_count"]=r.u32(); d["cal_target"]=r.u32(); d["isr_max_cycles"]=r.u32(); d["isr_overruns"]=r.u32()
    d["encoder_count"]=r.i32(); d["encoder_offset_u16"]=r.u16()
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
    raw,frames=_raw_handshake(link,args.timeout)
    print("RX raw:", raw.hex(" ") if raw else "<no bytes>")
    fw=next((p for p in frames if p and p[0]==COMM_FW_VERSION),None)
    if fw is None:
        print("FAIL: tidak ada frame COMM_FW_VERSION. Cek PB10/USART3_TX -> adapter RX, PB11/USART3_RX <- adapter TX, GND bersama, dan baud 115200.")
        return 2
    print("RX payload:", fw.hex(" "))
    print(parse_fw_version(fw))
    print("PASS: framing + CRC + COMM_FW_VERSION reply valid")
    return 0


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
               'virtual_can_forwards','virtual_can_unknown_ids','baud']
        for name in names: d[name]=r.u32()
        d['blocking_queue_depth']=r.u8()
        d['timeout_active']=bool(r.u8())
        d['config_valid']=bool(r.u8())
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
        print(f"      sensor={SENSOR_NAMES.get(e['sensor_mode'],e['sensor_mode'])} hall={e['hall_raw']:03b} "
              f"rotor={e['rotor_elec_deg']:.2f} deg mechRPM={e['mech_rpm']:.1f} "
              f"offsets={e['offset_u']}/{e['offset_v']}/{e['offset_dc']} ISRmax={e['isr_max_cycles']} overrun={e['isr_overruns']}")
    return 0


def cmd_calibrate(link: Link, args: argparse.Namespace) -> int:
    link.stop(0); link.stop(1); time.sleep(0.1)
    d=get_cal(link,trigger=True)
    print("Kalibrasi dimulai; PWM kedua motor OFF.")
    deadline=time.monotonic()+args.timeout
    while time.monotonic()<deadline:
        time.sleep(0.1)
        d=get_cal(link,False)
        print(f"\rprogress {d['count']}/{d['target']} valid={d['valid']}",end='',flush=True)
        if d['done']: break
    print()
    print(d)
    if not d['done'] or not d['valid']:
        print("FAIL: offset current tidak lolos validasi noise/range.")
        return 2
    time.sleep(0.4)
    ok=True
    for m in (0,1):
        v=get_values(link,m)
        print_values(m,v)
        # This validates zero-current result, independent of the exact A/count gain.
        if abs(v.id)>args.zero_limit or abs(v.iq)>args.zero_limit or abs(v.imotor)>args.zero_limit*1.5:
            ok=False
    print("PASS zero-current" if ok else "WARN/FAIL zero-current residual di atas limit")
    return 0 if ok else 3


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


def cmd_sensor_detect(link: Link,args: argparse.Namespace)->int:
    require_yes(args,"Auto-detect sensor")
    mode={"auto":SENSOR_AUTO,"hall":SENSOR_HALL,"encoder":SENSOR_ENCODER}[args.mode]
    if args.motor==1 and mode==SENSOR_ENCODER: raise RuntimeError("RIGHT tidak memiliki encoder")
    cal=get_cal(link,False)
    if not cal['done'] or not cal['valid']: raise RuntimeError("Current-zero calibration belum valid. Jalankan 'calibrate' dulu.")
    link.stop(args.motor); link.clear_fault(args.motor); time.sleep(0.1)
    link.send(bytes((COMM_CUSTOM_APP_DATA,CUSTOM_SENSOR_DETECT,args.motor,mode)))
    deadline=time.monotonic()+args.timeout
    last=None
    while time.monotonic()<deadline:
        d=get_sensor(link,args.motor)
        state=d['state']
        if state!=last:
            print(f"state={state} {DETECT_NAMES.get(state,'?')} mode={SENSOR_NAMES.get(d['mode'],d['mode'])}")
            last=state
        if state in (10,11):
            print(d)
            return 0 if state==10 and d['success'] else 4
        time.sleep(0.1)
    raise TimeoutError("sensor detect timeout")


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
        if d['state'] in (10,11):
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
        if not d['success'] or d['state'] != 10:
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

def cmd_can_scan(link: Link, _args: argparse.Namespace) -> int:
    p=link.request(bytes((COMM_PING_CAN,)),lambda x: bool(x) and x[0]==COMM_PING_CAN,2.0)
    ids=list(p[1:])
    print("Virtual CAN IDs:",ids)
    if 2 not in ids:
        print("FAIL: motor RIGHT virtual CAN ID 2 tidak terdeteksi")
        return 2
    fw=link.request_std(1,bytes((COMM_FW_VERSION,)),lambda x: bool(x) and x[0]==COMM_FW_VERSION,1.0)
    d=parse_fw_version(fw)
    print("RIGHT FW via COMM_FORWARD_CAN:",d)
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
    fw=(bytes((COMM_FW_VERSION,7,1))+b"STM32F103RC_DUAL_FOC\x00"+bytes(range(12))+
        bytes((1,1,0,0,0,0,0,0))+b"F103_DUAL_FOC_RTOS2_V6\x00"+struct.pack(">I",0))
    fwd=parse_fw_version(fw)
    assert fwd["major"]==7 and fwd["minor"]==1 and fwd["fw_name"].endswith("V6") and fwd["hw_crc"]==0
    vals=[1.0,-2.0,3.25,4.0,5.0,6.0,7.0,8.0,16000.0]
    samp=bytes((COMM_SAMPLE_PRINT,))+be_i16(3)+b''.join(struct.pack(">f",x) for x in vals)+bytes((0,123))+be_i32(3)
    idx,row=parse_sample_packet(samp)
    assert idx==3 and abs(row['current1']+2.0)<1e-6 and row['index_full']==3
    print("SELF-TEST PASS: CRC, framing, V6 FW parser, virtual-CAN routing, standard sample parser")
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
    p=sp("handshake",cmd_handshake,"raw COMM_FW_VERSION TX/RX diagnostic"); p.add_argument("--timeout",type=float,default=1.0)
    p=sp("baud-scan",cmd_baud_scan,"scan common UART baud rates for VESC handshake"); p.add_argument("--bauds",default="115200,230400,250000,460800,921600"); p.add_argument("--timeout",type=float,default=0.6)
    sp("comm-diag",cmd_comm_diag,"read firmware USART IRQ/ring diagnostics")
    sp("can-scan",cmd_can_scan,"verify virtual CAN RIGHT ID 2 + forwarded FW_VERSION")
    sp("config-status",cmd_config_status,"read flash-emulated persistent config status")
    sp("config-save",cmd_config_save,"save runtime Hall/encoder/PID/timeout config to flash")
    p=sp("status",cmd_status,"standard VESC telemetry + extended telemetry"); p.add_argument("--motor",type=int,choices=[0,1])
    p=sp("monitor",cmd_monitor,"monitor both motors"); p.add_argument("--hz",type=float,default=5); p.add_argument("--seconds",type=float,default=10)
    p=sp("calibrate",cmd_calibrate,"re-run zero-current calibration"); p.add_argument("--timeout",type=float,default=5); p.add_argument("--zero-limit",type=float,default=0.30)
    p=sp("sensor-info",cmd_sensor_info,"show runtime Hall/encoder state"); p.add_argument("--motor",type=int,choices=[0,1])
    p=sp("sensor-select",cmd_sensor_select,"live switch Hall/encoder while stopped"); p.add_argument("--motor",type=int,choices=[0,1],required=True); p.add_argument("--mode",choices=["hall","encoder"],required=True)
    p=sp("sensor-detect",cmd_sensor_detect,"forced-angle Hall/encoder autodetect"); p.add_argument("--motor",type=int,choices=[0,1],required=True); p.add_argument("--mode",choices=["auto","hall","encoder"],default="auto"); p.add_argument("--timeout",type=float,default=15); p.add_argument("--yes",action="store_true")
    p=sp("rotor",cmd_rotor,"stream COMM_ROTOR_POSITION"); p.add_argument("--motor",type=int,choices=[0,1],required=True); p.add_argument("--mode",choices=list(DISPLAY_MODES),default="encoder"); p.add_argument("--seconds",type=float,default=5)
    p=sp("sample",cmd_sample,"capture fast FOC samples from ISR buffer"); p.add_argument("--motor",type=int,choices=[0,1],required=True); p.add_argument("--count",type=int,default=128); p.add_argument("--decimation",type=int,default=8); p.add_argument("--timeout",type=float,default=5); p.add_argument("--csv"); p.add_argument("--raw",action="store_true")
    p=sp("motor-test",cmd_motor_test,"active current/brake/rpm/duty/position test at 50 Hz command refresh"); p.add_argument("--motor",type=int,choices=[0,1],required=True); p.add_argument("--mode",choices=["current","brake","rpm","duty","position"],required=True); p.add_argument("--value",type=float,required=True); p.add_argument("--seconds",type=float,default=2); p.add_argument("--yes",action="store_true"); p.add_argument("--force",action="store_true")
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
