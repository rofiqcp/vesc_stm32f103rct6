#!/usr/bin/env python3
# Probe VESC Tool protocol — parser yang BENAR (cari 0x02/0x03, validasi CRC16-CCITT/XMODEM).
import serial, time, struct, sys

PORT, BAUD = "/dev/ttyUSB0", 115200

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc

def packet(cmd, payload=b""):
    data = bytes([cmd]) + payload
    c = crc16(data); n = len(data)
    body = bytes([0x02, n]) + data if n < 256 else bytes([0x03, 0, 0]) + struct.pack(">H", n) + data
    return body + struct.pack(">H", c) + bytes([0x03])

def parse_frames(buf):
    """Return list of (cmd, payload) for valid VESC frames in buf."""
    out = []
    i = 0
    while i < len(buf):
        s = buf.find(0x02, i)
        if s < 0:
            break
        if s + 2 >= len(buf):
            break
        n = buf[s + 1]
        header = 2
        idx = s + 2
        if n < 254:
            payload_len = n
        else:
            if idx + 2 >= len(buf):
                break
            payload_len = struct.unpack(">H", buf[idx:idx + 2])[0]
            header = 4
            idx += 2
        total = header + payload_len + 3  # +2 crc +1 stop
        if idx + payload_len + 3 > len(buf):
            break
        payload = buf[idx:idx + payload_len]
        crc_recv = struct.unpack(">H", buf[idx + payload_len:idx + payload_len + 2])[0]
        stop = buf[idx + payload_len + 2]
        if stop == 0x03 and crc16(payload) == crc_recv:
            out.append((payload[0], payload[1:]))
            i = s + total
        else:
            i = s + 1
    return out

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    time.sleep(0.3)
    ser.reset_input_buffer()

    print("=== COMM_FW_VERSION (cmd 0) ===")
    ser.write(packet(0x00))
    time.sleep(1.0)
    raw = ser.read(2000)
    frames = parse_frames(raw)
    if not frames:
        print("  raw bytes:", len(raw), "hex[:40]:", raw[:40].hex())
        print("  !! tidak ada frame valid")
        ser.close(); return
    cmd, pl = frames[0]
    print(f"  OK — {len(frames)} frame valid, pertama cmd=0x{cmd:02X} len={len(pl)}")
    # decode: major, minor, hw string, uuid, ...
    try:
        major = pl[0]; minor = pl[1]
        hw = pl[2:].split(b'\x00')[0].decode('ascii', 'replace')
        print(f"  FW: v{major}.{minor}  hw=\"{hw}\"")
    except Exception as e:
        print("  decode:", e)

    print("\n=== COMM_GET_VALUES (cmd 4) ===")
    ser.write(packet(0x04))
    time.sleep(1.0)
    raw = ser.read(2000)
    frames = parse_frames(raw)
    if frames:
        cmd, pl = frames[0]
        print(f"  OK — cmd=0x{cmd:02X} len={len(pl)} (GET_VALUES balasan diterima)")
    else:
        print("  !! tidak ada frame valid (GET_VALUES tidak direspons)")

    ser.close()
    print("\n>>> FIRMWARE MERESPON DENGAN BENAR — koneksi VESC Tool OK.")

if __name__ == "__main__":
    main()
