#!/usr/bin/env python3
# Tester protocol VESC Tool ASLI lewat USART3 (/dev/ttyUSB0).
# Framing + CRC persis seperti pyvesc / VESC Tool (lib/ crc & packet).
import serial, time, struct, sys

PORT = "/dev/ttyUSB0"
BAUD = 115200

def crc16(data):
    # VESC CRC16 (0x1021 poly, init 0, xorout 0) -> big-endian
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def packet(cmd, payload=b""):
    # build vesc packet: 0x02 <len> <payload...> <crc16 BE> 0x03
    data = bytes([cmd]) + payload
    crc = crc16(data)
    n = len(data)
    if n < 256:
        body = bytes([0x02, n]) + data
    else:
        body = bytes([0x03, 0x00]) + struct.pack(">H", n) + data
    return body + struct.pack(">H", crc) + bytes([0x03])

def parse_packet(buf):
    # cari 0x02 ... 0x03, return (cmd, payload)
    s = buf.find(0x02)
    if s < 0 or s + 2 > len(buf):
        return None
    if buf[s+1] < 254:
        n = buf[s+1]; idx = s+2
    else:
        n = struct.unpack(">H", buf[s+2:s+4])[0]; idx = s+4
    if idx + n + 3 > len(buf):
        return None
    data = buf[idx:idx+n]
    crc_recv = struct.unpack(">H", buf[idx+n:idx+n+2])[0]
    end = buf[idx+n+2]
    if end != 0x03:
        return None
    if crc16(data) != crc_recv:
        return None
    return data[0], data[1:]

def send_recv(ser, cmd, payload=b"", tries=5, timeout=1.0):
    ser.write(packet(cmd, payload))
    start = time.time()
    buf = b""
    while time.time() - start < timeout:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            # coba parse dari buffer
            try:
                r = parse_packet(buf)
            except Exception:
                r = None
            if r and r[0] == cmd:
                return r[1]
        time.sleep(0.01)
    return None

def scaled_i32(buf, i, scale):
    v = struct.unpack_from(">i", buf, i[0])[0]; i[0]+=4
    return v/scale

def scaled_i16(buf, i, scale):
    v = struct.unpack_from(">h", buf, i[0])[0]; i[0]+=2
    return v/scale

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    # 1) COMM_FW_VERSION (cmd 0)
    print("=== COMM_FW_VERSION (cmd 0) ===")
    r = send_recv(ser, 0x00)
    if r is None:
        print("  !! TIDAK ada reply -> board/firmware/serial bermasalah")
        return
    # format: hw=cstring, fw=cstring, ... baca sederhana
    # fw version ascii sampai 20 bytes
    try:
        # cari string
        s = r.split(b'\x00')[0].decode('ascii','replace')
        print(f"  FW string: {s!r}")
        print(f"  raw bytes ({len(r)}): {r[:32].hex()}")
    except Exception as e:
        print("  parse:", e)

    # 2) COMM_GET_VALUES (cmd 4) mask 0x003FFFFF, semuanya scaled
    print("\n=== COMM_GET_VALUES (cmd 4) ===")
    r = send_recv(ser, 0x04)
    if r is None:
        print("  !! TIDAK ada reply")
        return
    # parse sesuai mask bit (field order):
    # 0 temp_mos(i16 dC),1 temp_motor(i16 dC),2 current_motor(i32*100),
    # 3 current_in(i32*100),4 id(i32*100),5 iq(i32*100),6 duty(i16*1000),
    # 7 erpm(i32),8 v_in(i16*10),9 amp_hours(i32*10000),10 amp_hours_charged,
    # 11 watt_hours,12 watt_hours_charged,13 tacho(i32),14 tacho_abs(i32),
    # 15 fault(i8),16 position(i32*1e6),17 controller_id(u8),18 ... temps
    i = [0]
    temp_mos = scaled_i16(r,i,10); temp_motor = scaled_i16(r,i,10)
    current_motor = scaled_i32(r,i,100); current_in = scaled_i32(r,i,100)
    idq = scaled_i32(r,i,100); iqq = scaled_i32(r,i,100)
    duty = scaled_i16(r,i,1000); erpm = struct.unpack_from(">i", r, i[0])[0]; i[0]+=4
    vin = scaled_i16(r,i,10)
    # skip amp/watt hours + tacho (6x i32) + fault + position + ctrl id
    for _ in range(8):
        struct.unpack_from(">i", r, i[0])[0]; i[0]+=4
    fault = r[i[0]]; i[0]+=1
    pos = struct.unpack_from(">i", r, i[0])[0]; i[0]+=4
    cid = r[i[0]]; i[0]+=1
    print(f"  temp_mos={temp_mos:.1f}C temp_motor={temp_motor:.1f}C")
    print(f"  current_motor={current_motor:.2f}A current_in={current_in:.2f}A")
    print(f"  id={idq:.2f}A iq={iqq:.2f}A")
    print(f"  duty={duty:.3f} erpm={erpm} v_in={vin:.2f}V")
    print(f"  fault_code={fault} controller_id={cid}")

    # 3) COMM_GET_VALUES_SELECTIVE (cmd 50) mask = current+duty+v_in+erpm
    print("\n=== COMM_GET_VALUES_SELECTIVE (cmd 50) ===")
    mask = (1<<2)|(1<<3)|(1<<6)|(1<<7)|(1<<8)
    r = send_recv(ser, 0x32, struct.pack(">I", mask))
    if r is None:
        print("  !! TIDAK ada reply")
    else:
        print(f"  selective reply len={len(r)} (protocol ok)")

    print("\n>>> TEST SELESAI: protocol VESC Tool ASLI merespons.")

if __name__ == "__main__":
    main()
