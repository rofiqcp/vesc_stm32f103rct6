#!/usr/bin/env bash
set -Eeuo pipefail

# One-command debug launcher for THIS STM32F103RCT6 VESC firmware.
# Native FreeRTOS awareness + ST-LINK + OpenOCD + GDB helper commands.

ENV_NAME="stm32f103rc_debug"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
CFG_DIRECT="$SCRIPT_DIR/openocd.cfg"
CFG_HLA="$SCRIPT_DIR/openocd_hla.cfg"
GDB_HELPERS="$SCRIPT_DIR/vesc.gdb"
ELF="$PROJECT_ROOT/.pio/build/$ENV_NAME/firmware.elf"
OPENOCD_LOG="$SCRIPT_DIR/openocd_debug.log"

DO_BUILD=1
DO_FLASH=1
FORCE_HLA=0
REUSE_OPENOCD=0
STARTED_OPENOCD=0
OPENOCD_PID=""
GDB_CMDS=""

usage() {
  cat <<USAGE
Usage: ./openocd_debug/run_debug.sh [options]
  --no-build        skip PlatformIO debug build
  --no-flash        do not GDB 'load' the ELF
  --hla             force legacy ST-LINK HLA config
  --reuse-openocd   reuse an existing GDB server on localhost:3333
  -h, --help        show this help
USAGE
}

for arg in "$@"; do
  case "$arg" in
    --no-build) DO_BUILD=0 ;;
    --no-flash) DO_FLASH=0 ;;
    --hla) FORCE_HLA=1 ;;
    --reuse-openocd) REUSE_OPENOCD=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[ERROR] Unknown option: $arg" >&2; usage; exit 2 ;;
  esac
done

find_exe() {
  local name="$1"; shift
  if command -v "$name" >/dev/null 2>&1; then command -v "$name"; return 0; fi
  local c
  for c in "$@"; do [[ -x "$c" ]] && { printf '%s\n' "$c"; return 0; }; done
  return 1
}

PIO="$(find_exe pio "$HOME/.platformio/penv/bin/pio" "$HOME/.local/bin/pio" 2>/dev/null || true)"
GDB="$(find_exe arm-none-eabi-gdb \
  "$HOME/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gdb" \
  "$HOME/.platformio/packages/toolchain-gccarmnoneeabi@1.70201.0/bin/arm-none-eabi-gdb" 2>/dev/null || true)"
READELF="$(find_exe arm-none-eabi-readelf \
  "$HOME/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-readelf" \
  "$HOME/.platformio/packages/toolchain-gccarmnoneeabi@1.70201.0/bin/arm-none-eabi-readelf" 2>/dev/null || true)"
OPENOCD="$(find_exe openocd /usr/bin/openocd /usr/local/bin/openocd 2>/dev/null || true)"

reset_target_run_best_effort() {
  # GDB normally executes hook-quit below. This second path is deliberate: it
  # also covers Ctrl-C/terminal closure or a GDB crash. Do not depend on netcat
  # being installed; Python is already required by tools/debug.py.
  if command -v nc >/dev/null 2>&1; then
    printf 'reset run\nexit\n' | nc -w 1 127.0.0.1 4444 >/dev/null 2>&1 && return 0
  fi
  if command -v python3 >/dev/null 2>&1; then
    python3 - <<'PY_RESET' >/dev/null 2>&1 || true
import socket, time
s = socket.create_connection(("127.0.0.1", 4444), timeout=0.5)
s.settimeout(0.2)
try:
    s.recv(4096)
except Exception:
    pass
s.sendall(b"reset run\n")
time.sleep(0.08)
s.sendall(b"exit\n")
s.close()
PY_RESET
  fi
}

cleanup() {
  local rc=$?
  [[ -n "$GDB_CMDS" && -f "$GDB_CMDS" ]] && rm -f "$GDB_CMDS"
  if [[ "$STARTED_OPENOCD" -eq 1 && -n "$OPENOCD_PID" ]]; then
    # Never leave the MCU halted when the launcher exits. A halted target has
    # a perfectly silent USART and previously looked exactly like a comm bug.
    reset_target_run_best_effort
    sleep 0.1
    echo "[INFO] stopping OpenOCD PID $OPENOCD_PID (target reset+running)"
    kill "$OPENOCD_PID" 2>/dev/null || true
    wait "$OPENOCD_PID" 2>/dev/null || true
  fi
  exit "$rc"
}
trap cleanup EXIT INT TERM

port3333() {
  if command -v ss >/dev/null 2>&1; then
    ss -ltn 2>/dev/null | awk '{print $4}' | grep -Eq '(^|:)3333$'
  elif command -v nc >/dev/null 2>&1; then
    nc -z 127.0.0.1 3333 >/dev/null 2>&1
  else
    return 1
  fi
}

wait_openocd() {
  local i
  for i in {1..60}; do
    port3333 && return 0
    [[ -n "$OPENOCD_PID" ]] && ! kill -0 "$OPENOCD_PID" 2>/dev/null && return 1
    sleep 0.1
  done
  return 1
}

start_openocd_cfg() {
  local cfg="$1"
  : > "$OPENOCD_LOG"
  "$OPENOCD" -f "$cfg" >"$OPENOCD_LOG" 2>&1 &
  OPENOCD_PID=$!
  STARTED_OPENOCD=1
  wait_openocd
}

[[ -f "$PROJECT_ROOT/platformio.ini" ]] || { echo "[ERROR] platformio.ini missing" >&2; exit 1; }
[[ -f "$GDB_HELPERS" ]] || { echo "[ERROR] $GDB_HELPERS missing" >&2; exit 1; }
[[ -n "$GDB" ]] || { echo "[ERROR] arm-none-eabi-gdb not found" >&2; exit 1; }
cd "$PROJECT_ROOT"

echo "============================================================"
echo " VESC F103 - FreeRTOS + OpenOCD + GDB"
echo "============================================================"
echo "SAFETY: halting the CPU halts real-time control. Debug powered FOC only"
echo "with the power stage/motor in a safe commissioning condition."

echo "[1/5] Build"
if [[ "$DO_BUILD" -eq 1 ]]; then
  [[ -n "$PIO" ]] || { echo "[ERROR] pio not found" >&2; exit 1; }
  "$PIO" run -e "$ENV_NAME"
else
  echo "      skipped"
fi
[[ -f "$ELF" ]] || { echo "[ERROR] ELF not found: $ELF" >&2; exit 1; }

echo "[2/5] DWARF symbols"
if [[ -n "$READELF" ]]; then
  "$READELF" -S "$ELF" | grep -q '\.debug_info' || { echo "[ERROR] .debug_info missing" >&2; exit 1; }
  "$READELF" -S "$ELF" | grep -q '\.debug_line' || { echo "[ERROR] .debug_line missing" >&2; exit 1; }
  echo "      .debug_info + .debug_line OK"
fi

echo "[3/5] OpenOCD"
if port3333; then
  if [[ "$REUSE_OPENOCD" -ne 1 ]]; then
    echo "[ERROR] port 3333 is already in use." >&2
    echo "        Stop the old OpenOCD, or explicitly pass --reuse-openocd." >&2
    exit 1
  fi
  echo "      reusing existing server; custom monitor commands depend on its config"
else
  [[ -n "$OPENOCD" ]] || { echo "[ERROR] openocd not found" >&2; exit 1; }
  if [[ "$FORCE_HLA" -eq 1 ]]; then
    start_openocd_cfg "$CFG_HLA" || {
      cat "$OPENOCD_LOG" >&2; exit 1;
    }
  else
    if ! start_openocd_cfg "$CFG_DIRECT"; then
      echo "      DAP-direct failed; trying legacy HLA..."
      [[ -n "$OPENOCD_PID" ]] && kill "$OPENOCD_PID" 2>/dev/null || true
      wait "$OPENOCD_PID" 2>/dev/null || true
      STARTED_OPENOCD=0; OPENOCD_PID=""
      start_openocd_cfg "$CFG_HLA" || { cat "$OPENOCD_LOG" >&2; exit 1; }
    fi
  fi
  echo "      ready; log: $OPENOCD_LOG"
fi

echo "[4/5] GDB startup"
GDB_CMDS="$(mktemp /tmp/vesc-f103-XXXXXX.gdb)"
cat > "$GDB_CMDS" <<EOF_GDB
set pagination off
set confirm off
set breakpoint pending on
set remotetimeout 10
target extended-remote localhost:3333
monitor reset halt
source $GDB_HELPERS

# GDB detach leaves a halted Cortex-M halted. Always reset+run on normal quit so
# the UART is alive immediately after leaving this debug session.
define hook-quit
  monitor reset run
end
EOF_GDB

if [[ "$DO_FLASH" -eq 1 ]]; then
cat >> "$GDB_CMDS" <<'EOF_GDB'
echo \n=== Programming debug ELF ===\n
load
monitor reset halt
EOF_GDB
else
cat >> "$GDB_CMDS" <<'EOF_GDB'
echo \n=== --no-flash: target MUST match this ELF exactly ===\n
EOF_GDB
fi

cat >> "$GDB_CMDS" <<'EOF_GDB'
# Stop at either a successful scheduler/packet-task start or an early boot fatal.
# This is much more useful than stopping at main(), where UART/DMA/tasks are
# expected to be zero/uninitialized.
break early_fatal
break Error_Handler_Local
tbreak packet_process_thread
continue
vesc_boot
vesc_uart
echo \n============================================================\n
echo Boot diagnostic stop reached.\n
echo If stage=100 and packet_process_thread is selected, RTOS+UART boot succeeded.\n
echo If stage<100, inspect boot error and vesc_uart / monitor vesc_uart_hw.\n
echo Use 'continue' to run firmware. Ctrl-C later for vesc_snapshot.\n
echo Quitting this script resets+runs the MCU before OpenOCD is stopped.\n
echo ============================================================\n
EOF_GDB

echo "[5/5] Interactive GDB"
"$GDB" -x "$GDB_CMDS" "$ELF"
