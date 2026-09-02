#!/usr/bin/env python3
"""Tool teks VESC STM32F103RCT6 dual motor.

Runtime default:
    /dev/ttyUSB0, 115200 baud, USART3

File ini juga menjadi PlatformIO pre-script untuk memasukkan kernel FreeRTOS.
Saat dijalankan oleh PlatformIO, CLI serial tidak dieksekusi.
"""
from __future__ import annotations

import os


def _platformio_configure() -> None:
    """Fungsi PlatformIO: menambahkan source/include FreeRTOS STM32CubeF1 tanpa menggandakan queue.c."""
    Import("env")  # type: ignore[name-defined]  # disediakan oleh SCons/PlatformIO
    framework = env.PioPlatform().get_package_dir("framework-stm32cubef1")  # type: ignore[name-defined]
    if not framework:
        raise RuntimeError("framework-stm32cubef1 package not found")
    fr = os.path.join(framework, "Middlewares", "Third_Party", "FreeRTOS", "Source")
    portable = os.path.join(fr, "portable", "GCC", "ARM_CM3")
    memmang = os.path.join(fr, "portable", "MemMang")
    env.Append(CPPPATH=[fr, os.path.join(fr, "include"), portable])  # type: ignore[name-defined]
    env.BuildSources(  # type: ignore[name-defined]
        os.path.join("$BUILD_DIR", "freertos_kernel"), fr,
        src_filter=["+<*.c>", "-<queue.c>"],
    )
    env.BuildSources(  # type: ignore[name-defined]
        os.path.join("$BUILD_DIR", "freertos_port"), portable,
        src_filter=["+<port.c>"],
    )
    env.BuildSources(  # type: ignore[name-defined]
        os.path.join("$BUILD_DIR", "freertos_heap"), memmang,
        src_filter=["+<heap_4.c>"],
    )


# PlatformIO/SCons menyediakan simbol Import. Pada Python biasa simbol ini tidak ada.
if "Import" in globals():
    _platformio_configure()
else:
    import argparse
    import shlex
    import sys
    import time

    import debug as dv

    COMM_TERMINAL_CMD = 20
    COMM_PRINT = 21

    CONTROL_RESULT = {
        0: "NONE", 1: "ACCEPTED", 2: "BAD_LENGTH", 3: "SHUTDOWN",
        4: "MOTOR_NOT_READY", 5: "UART_REJECTED", 6: "INPUT_BLOCKED",
    }
    APP_REJECT = {
        0: "NONE", 1: "INVALID_ID", 2: "DETECT_BUSY", 3: "OUTPUT_DISABLED",
        4: "FAULT", 5: "CAL_NOT_DONE", 6: "CAL_INVALID",
    }


    def _motor(value: str) -> int:
        """Fungsi CLI: mengubah nama LEFT/RIGHT atau indeks menjadi motor internal 0/1."""
        text=value.strip().lower()
        if text in ("left", "l", "0", "1-local"):
            return 0
        if text in ("right", "r", "1", "2", "id2"):
            return 1
        raise argparse.ArgumentTypeError("motor harus left/right atau 0/1")


    def _stop_standard(link: dv.Link, motor: int) -> None:
        """Fungsi CLI: menghentikan command standar dengan current=0 dan duty=0, lalu custom stop sebagai pengaman."""
        link.send_std(motor, bytes((dv.COMM_SET_CURRENT,))+dv.be_i32(0))
        link.send_std(motor, bytes((dv.COMM_SET_DUTY,))+dv.be_i32(0))
        link.stop(motor)


    def _payload(mode: str, value: float) -> bytes:
        """Fungsi CLI: menyusun payload command motor dengan skala wire VESC 6.00."""
        if mode in ("duty", "current", "brake", "rpm", "position"):
            return dv.command_payload(mode,value)
        if mode == "handbrake":
            return bytes((dv.COMM_SET_HANDBRAKE,))+dv.be_i32(round(abs(value)*1000.0))
        if mode == "current-rel":
            return bytes((dv.COMM_SET_CURRENT_REL,))+dv.be_i32(round(value*100000.0))
        raise ValueError(mode)


    def _print_command_diag(link: dv.Link) -> None:
        """Fungsi CLI: menampilkan breadcrumb firmware agar silent-reject command langsung terlihat."""
        try:
            p=link.request(bytes((dv.COMM_CUSTOM_APP_DATA,dv.CUSTOM_COMM_DIAG)),
                           lambda x: len(x)>=3 and x[:2]==bytes((dv.COMM_CUSTOM_APP_DATA,dv.CUSTOM_COMM_DIAG)),1.0)
            d=dv.parse_comm_diag(p)
            result=d.get("last_control_result")
            reject=d.get("last_control_app_reject")
            print(
                "command_diag: "
                f"cmd={d.get('last_control_cmd','?')} motor={d.get('last_control_motor','?')} "
                f"result={result}({CONTROL_RESULT.get(result,'?')}) "
                f"app_reject={reject}({APP_REJECT.get(reject,'?')}) "
                f"raw={d.get('last_control_value_scaled','?')} "
                f"accept/reject={d.get('control_accept_count','?')}/{d.get('control_reject_count','?')}"
            )
        except Exception as exc:
            print(f"command_diag unavailable: {exc}")


    def _run_motion(link: dv.Link, motor: int, mode: str, value: float,
                    seconds: float, yes: bool, force: bool) -> int:
        """Fungsi CLI: mengirim ulang command motor 50 Hz agar timeout VESC tidak memutus gerakan."""
        if not yes:
            raise RuntimeError("command aktif membutuhkan --yes")
        if seconds <= 0.0:
            raise ValueError("--seconds harus > 0")
        dv._motion_precheck(link,motor,force,position=(mode=="position"))
        link.clear_fault(motor)
        time.sleep(0.05)
        payload=_payload(mode,value)
        deadline=time.monotonic()+seconds
        next_cmd=0.0
        next_values=0.0
        try:
            while time.monotonic()<deadline:
                now=time.monotonic()
                if now>=next_cmd:
                    link.send_std(motor,payload)
                    next_cmd+=0.02
                    if next_cmd<now:
                        next_cmd=now+0.02
                if now>=next_values:
                    try:
                        v=dv.get_values(link,motor,0.3)
                        dv.print_values(motor,v)
                        if v.fault:
                            _print_command_diag(link)
                            return 6
                    except TimeoutError:
                        print("WARN GET_VALUES timeout")
                    next_values=now+0.20
                time.sleep(0.002)
        finally:
            _stop_standard(link,motor)
            time.sleep(0.03)
            _print_command_diag(link)
        return 0


    def cmd_info(link: dv.Link, _args: argparse.Namespace) -> int:
        """Fungsi CLI: membaca identitas dan realtime value kedua motor."""
        for motor in (0,1):
            p=link.request_std(motor,bytes((dv.COMM_FW_VERSION,)),lambda x: bool(x) and x[0]==dv.COMM_FW_VERSION,1.5)
            fw=dv.parse_fw_version(p)
            name="LEFT" if motor==0 else "RIGHT"
            print(f"{name}: FW={fw['major']}.{fw['minor']} HW={fw['hw_name']} UUID={fw['uuid']}")
            dv.print_values(motor,dv.get_values(link,motor,1.0))
        return 0


    def cmd_values(link: dv.Link, args: argparse.Namespace) -> int:
        """Fungsi CLI: membaca GET_VALUES standar VESC sekali."""
        motors=(0,1) if args.motor is None else (args.motor,)
        for motor in motors:
            dv.print_values(motor,dv.get_values(link,motor,1.0))
        return 0


    def cmd_stream(link: dv.Link, args: argparse.Namespace) -> int:
        """Fungsi CLI: polling GET_VALUES standar VESC secara kontinu."""
        motors=(0,1) if args.motor is None else (args.motor,)
        end=time.monotonic()+args.seconds if args.seconds>0 else float("inf")
        period=1.0/max(1.0,args.hz)
        while time.monotonic()<end:
            t0=time.monotonic()
            for motor in motors:
                dv.print_values(motor,dv.get_values(link,motor,1.0))
            print("-")
            dt=time.monotonic()-t0
            if dt<period:
                time.sleep(period-dt)
        return 0


    def cmd_motion(link: dv.Link, args: argparse.Namespace) -> int:
        """Fungsi CLI: menjalankan satu command duty/current/RPM/position/brake/handbrake/current-relative."""
        return _run_motion(link,args.motor,args.cmd,args.value,args.seconds,args.yes,args.force)


    def cmd_stop(link: dv.Link, args: argparse.Namespace) -> int:
        """Fungsi CLI: menghentikan satu atau kedua motor."""
        motors=(0,1) if args.motor is None else (args.motor,)
        for motor in motors:
            _stop_standard(link,motor)
        return 0


    def cmd_alive(link: dv.Link, args: argparse.Namespace) -> int:
        """Fungsi CLI: mengirim COMM_ALIVE standar VESC ke motor yang dipilih."""
        motors=(0,1) if args.motor is None else (args.motor,)
        for motor in motors:
            link.send_std(motor,bytes((dv.COMM_ALIVE,)))
        return 0


    def cmd_terminal(link: dv.Link, args: argparse.Namespace) -> int:
        """Fungsi CLI: menjalankan terminal command standar VESC dan mencetak COMM_PRINT yang diterima."""
        motor=args.motor
        payload=bytes((COMM_TERMINAL_CMD,))+args.text.encode("utf-8")
        link.send_std(motor,payload)
        deadline=time.monotonic()+args.timeout
        got=False
        while time.monotonic()<deadline:
            try:
                p=link.recv(timeout=min(0.15,max(0.01,deadline-time.monotonic())))
            except TimeoutError:
                continue
            if p and p[0]==COMM_PRINT:
                print(p[1:].decode("utf-8",errors="replace"),end="",flush=True)
                got=True
        if got:
            print()
        return 0


    def cmd_shell(link: dv.Link, _args: argparse.Namespace) -> int:
        """Fungsi CLI: menyediakan shell teks sederhana untuk commissioning tanpa mengingat opsi argparse."""
        print("VESC text shell. Motor: left/right. 'help' untuk perintah, 'quit' keluar.")
        while True:
            try:
                parts=shlex.split(input("vesc> "))
            except (EOFError,KeyboardInterrupt):
                print()
                break
            if not parts:
                continue
            if parts[0] in ("quit","exit"):
                break
            if parts[0]=="help":
                print("info | values [left|right] | stop [left|right] | diag | terminal [motor] TEXT")
                print("duty/current/rpm/position/brake/handbrake/current-rel MOTOR VALUE [SECONDS]")
                continue
            try:
                if parts[0]=="info":
                    cmd_info(link,argparse.Namespace())
                elif parts[0]=="values":
                    m=None if len(parts)<2 else _motor(parts[1])
                    cmd_values(link,argparse.Namespace(motor=m))
                elif parts[0]=="stop":
                    m=None if len(parts)<2 else _motor(parts[1])
                    cmd_stop(link,argparse.Namespace(motor=m))
                elif parts[0]=="diag":
                    _print_command_diag(link)
                elif parts[0]=="terminal":
                    if len(parts)<3:
                        print("usage: terminal MOTOR TEXT")
                        continue
                    cmd_terminal(link,argparse.Namespace(motor=_motor(parts[1]),text=" ".join(parts[2:]),timeout=0.8))
                elif parts[0] in ("duty","current","rpm","position","brake","handbrake","current-rel"):
                    if len(parts)<3:
                        print("usage: MODE MOTOR VALUE [SECONDS]")
                        continue
                    sec=float(parts[3]) if len(parts)>3 else 2.0
                    _run_motion(link,_motor(parts[1]),parts[0],float(parts[2]),sec,True,False)
                else:
                    print("unknown command")
            except Exception as exc:
                print(f"ERROR: {exc}")
        for motor in (0,1):
            _stop_standard(link,motor)
        return 0


    def _parser() -> argparse.ArgumentParser:
        """Fungsi CLI: membangun parser dengan default USART3 /dev/ttyUSB0 115200."""
        ap=argparse.ArgumentParser(description="VESC text tool - STM32F103RCT6 dual motor")
        ap.add_argument("--port",default="/dev/ttyUSB0",help="default /dev/ttyUSB0")
        ap.add_argument("--baud",type=int,default=115200,help="USART3 default 115200")
        sub=ap.add_subparsers(dest="cmd",required=True)
        sub.add_parser("info")
        p=sub.add_parser("values")
        p.add_argument("--motor",type=_motor)
        p=sub.add_parser("stream")
        p.add_argument("--motor",type=_motor)
        p.add_argument("--hz",type=float,default=50.0)
        p.add_argument("--seconds",type=float,default=0.0,help="0 = terus sampai Ctrl-C")
        for mode in ("duty","current","rpm","position","brake","handbrake","current-rel"):
            p=sub.add_parser(mode)
            p.add_argument("value",type=float)
            p.add_argument("--motor",type=_motor,default=0)
            p.add_argument("--seconds",type=float,default=2.0)
            p.add_argument("--yes",action="store_true")
            p.add_argument("--force",action="store_true")
        p=sub.add_parser("stop")
        p.add_argument("--motor",type=_motor)
        p=sub.add_parser("alive")
        p.add_argument("--motor",type=_motor)
        p=sub.add_parser("terminal")
        p.add_argument("text")
        p.add_argument("--motor",type=_motor,default=0)
        p.add_argument("--timeout",type=float,default=0.8)
        sub.add_parser("shell")
        return ap


    def main() -> int:
        """Fungsi CLI: membuka UART, menjalankan command, dan memastikan motor berhenti saat interrupt/error."""
        args=_parser().parse_args()
        link=dv.Link(args.port,args.baud)
        try:
            if args.cmd=="info":
                return cmd_info(link,args)
            if args.cmd=="values":
                return cmd_values(link,args)
            if args.cmd=="stream":
                return cmd_stream(link,args)
            if args.cmd in ("duty","current","rpm","position","brake","handbrake","current-rel"):
                return cmd_motion(link,args)
            if args.cmd=="stop":
                return cmd_stop(link,args)
            if args.cmd=="alive":
                return cmd_alive(link,args)
            if args.cmd=="terminal":
                return cmd_terminal(link,args)
            if args.cmd=="shell":
                return cmd_shell(link,args)
            raise ValueError(args.cmd)
        except KeyboardInterrupt:
            print("\nSTOP")
            for motor in (0,1):
                try:
                    _stop_standard(link,motor)
                except Exception:
                    pass
            return 130
        except Exception as exc:
            print(f"ERROR: {exc}",file=sys.stderr)
            return 1
        finally:
            link.close()


    if __name__=="__main__":
        raise SystemExit(main())
