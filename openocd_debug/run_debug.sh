#!/usr/bin/env bash
set -Eeuo pipefail

# =============================================================================
# One-command VESC STM32F103RCT6 debug launcher
#
# Expected layout:
#   vesc_stm32f103rct6/
#   ├── platformio.ini
#   ├── src/
#   ├── tools/
#   └── openocd_debug/
#       ├── openocd.cfg
#       └── run_debug.sh
#
# Default behaviour:
#   1) Build env stm32f103rc_debug
#   2) Verify DWARF debug information exists
#   3) Reuse OpenOCD on :3333, or start this folder's openocd.cfg
#   4) Connect GDB
#   5) Program the DEBUG ELF with GDB "load"
#   6) reset/halt -> temporary breakpoint at main -> continue
#   7) Leave you at an interactive (gdb) prompt at main()
#
# Options:
#   --no-build   Skip PlatformIO build
#   --no-flash   Do not execute GDB "load" (only safe when target already has
#                the exact same debug ELF programmed)
#   --help       Show usage
# =============================================================================

ENV_NAME="stm32f103rc_debug"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
OPENOCD_CFG="$SCRIPT_DIR/openocd.cfg"
ELF="$PROJECT_ROOT/.pio/build/$ENV_NAME/firmware.elf"
OPENOCD_LOG="$SCRIPT_DIR/openocd_debug.log"

DO_BUILD=1
DO_FLASH=1
STARTED_OPENOCD=0
OPENOCD_PID=""
GDB_CMDS=""

usage() {
    sed -n '3,30p' "$0" | sed 's/^# \{0,1\}//'
}

for arg in "$@"; do
    case "$arg" in
        --no-build) DO_BUILD=0 ;;
        --no-flash) DO_FLASH=0 ;;
        --help|-h) usage; exit 0 ;;
        *) echo "[ERROR] Unknown option: $arg" >&2; usage; exit 2 ;;
    esac
done

find_exe() {
    local name="$1"
    shift
    if command -v "$name" >/dev/null 2>&1; then
        command -v "$name"
        return 0
    fi
    local candidate
    for candidate in "$@"; do
        if [[ -x "$candidate" ]]; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done
    return 1
}

PIO="$(find_exe pio \
    "$HOME/.platformio/penv/bin/pio" \
    "$HOME/.local/bin/pio" 2>/dev/null || true)"

GDB="$(find_exe arm-none-eabi-gdb \
    "$HOME/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gdb" \
    "$HOME/.platformio/packages/toolchain-gccarmnoneeabi@1.70201.0/bin/arm-none-eabi-gdb" \
    2>/dev/null || true)"

READELF="$(find_exe arm-none-eabi-readelf \
    "$HOME/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-readelf" \
    "$HOME/.platformio/packages/toolchain-gccarmnoneeabi@1.70201.0/bin/arm-none-eabi-readelf" \
    2>/dev/null || true)"

OPENOCD="$(find_exe openocd /usr/bin/openocd /usr/local/bin/openocd 2>/dev/null || true)"

cleanup() {
    local rc=$?
    [[ -n "$GDB_CMDS" && -f "$GDB_CMDS" ]] && rm -f "$GDB_CMDS"
    if [[ "$STARTED_OPENOCD" -eq 1 && -n "$OPENOCD_PID" ]]; then
        echo
        echo "[INFO] Stopping OpenOCD started by this script (PID $OPENOCD_PID)..."
        kill "$OPENOCD_PID" 2>/dev/null || true
        wait "$OPENOCD_PID" 2>/dev/null || true
    fi
    exit "$rc"
}
trap cleanup EXIT INT TERM

[[ -f "$PROJECT_ROOT/platformio.ini" ]] || {
    echo "[ERROR] platformio.ini not found at: $PROJECT_ROOT/platformio.ini" >&2
    exit 1
}
[[ -f "$OPENOCD_CFG" ]] || {
    echo "[ERROR] OpenOCD config not found at: $OPENOCD_CFG" >&2
    exit 1
}
[[ -n "$GDB" ]] || {
    echo "[ERROR] arm-none-eabi-gdb not found." >&2
    echo "        Expected under ~/.platformio/packages/toolchain-gccarmnoneeabi/bin/" >&2
    exit 1
}

cd "$PROJECT_ROOT"

echo "============================================================"
echo " VESC STM32F103RCT6 - BUILD + OPENOCD + GDB DEBUG"
echo "============================================================"
echo "Project : $PROJECT_ROOT"
echo "Env     : $ENV_NAME"
echo "GDB     : $GDB"
echo "ELF     : $ELF"
echo ""
echo "SAFETY: halting the MCU stops real-time control execution."
echo "For ISR/FOC debugging, keep the motor/power stage in a safe state."
echo "============================================================"

if [[ "$DO_BUILD" -eq 1 ]]; then
    [[ -n "$PIO" ]] || {
        echo "[ERROR] PlatformIO 'pio' not found." >&2
        echo "        Tried PATH, ~/.platformio/penv/bin/pio and ~/.local/bin/pio" >&2
        exit 1
    }
    echo "[1/5] Building debug firmware..."
    "$PIO" run -e "$ENV_NAME"
else
    echo "[1/5] Build skipped (--no-build)."
fi

[[ -f "$ELF" ]] || {
    echo "[ERROR] Debug ELF not found: $ELF" >&2
    exit 1
}

echo "[2/5] Checking DWARF debug information..."
if [[ -n "$READELF" ]]; then
    if "$READELF" -S "$ELF" | grep -q '\.debug_info'; then
        echo "      OK: .debug_info present"
    else
        echo "[ERROR] $ELF has no .debug_info section." >&2
        echo "        Do not continue: GDB source/locals/line debugging would be invalid." >&2
        exit 1
    fi
    if "$READELF" -S "$ELF" | grep -q '\.debug_line'; then
        echo "      OK: .debug_line present"
    else
        echo "[ERROR] $ELF has no .debug_line section." >&2
        exit 1
    fi
else
    echo "      WARN: arm-none-eabi-readelf not found; symbol-section check skipped."
fi

port_3333_listening() {
    if command -v ss >/dev/null 2>&1; then
        ss -ltn 2>/dev/null | awk '{print $4}' | grep -Eq '(^|:)3333$'
    elif command -v nc >/dev/null 2>&1; then
        nc -z 127.0.0.1 3333 >/dev/null 2>&1
    else
        return 1
    fi
}

if port_3333_listening; then
    echo "[3/5] OpenOCD GDB server already listening on :3333; reusing it."
else
    [[ -n "$OPENOCD" ]] || {
        echo "[ERROR] OpenOCD is not running and 'openocd' executable was not found." >&2
        exit 1
    }
    echo "[3/5] Starting OpenOCD with ST-LINK DAP-direct..."
    : > "$OPENOCD_LOG"
    "$OPENOCD" -f "$OPENOCD_CFG" >"$OPENOCD_LOG" 2>&1 &
    OPENOCD_PID=$!
    STARTED_OPENOCD=1

    ready=0
    for _ in {1..50}; do
        if port_3333_listening; then
            ready=1
            break
        fi
        if ! kill -0 "$OPENOCD_PID" 2>/dev/null; then
            break
        fi
        sleep 0.1
    done

    if [[ "$ready" -ne 1 ]]; then
        echo "[ERROR] OpenOCD did not open GDB port 3333." >&2
        echo "-------- $OPENOCD_LOG --------" >&2
        cat "$OPENOCD_LOG" >&2 || true
        exit 1
    fi
    echo "      OpenOCD ready. Log: $OPENOCD_LOG"
fi

echo "[4/5] Preparing GDB startup commands..."
GDB_CMDS="$(mktemp /tmp/vesc-gdb-XXXXXX.gdb)"
cat > "$GDB_CMDS" <<EOF_GDB
set pagination off
set print pretty on
set print frame-arguments all
set breakpoint pending on
set remotetimeout 10

target extended-remote localhost:3333
monitor reset halt
EOF_GDB

if [[ "$DO_FLASH" -eq 1 ]]; then
    cat >> "$GDB_CMDS" <<'EOF_GDB'

echo \n=== Programming DEBUG ELF to STM32 ===\n
load
monitor reset halt
EOF_GDB
else
    cat >> "$GDB_CMDS" <<'EOF_GDB'

echo \n=== WARNING: --no-flash selected; target code must exactly match this ELF ===\n
EOF_GDB
fi

cat >> "$GDB_CMDS" <<'EOF_GDB'

echo \n=== Break once at main() ===\n
tbreak main
continue

echo \n============================================================\n
echo Target stopped at main(). Source-level debug is ready.\n
echo Useful commands:\n
echo   list\n
echo   bt\n
echo   info locals\n
echo   info threads\n
echo   thread apply all bt\n
echo   thbreak DMA1_Channel1_IRQHandler\n
echo   thbreak USART3_IRQHandler\n
echo   monitor vesc_fault_state\n
echo   monitor vesc_motor_periph_state\n
echo ============================================================\n
EOF_GDB

echo "[5/5] Starting interactive GDB..."
echo "      Debug build will be programmed unless --no-flash was supplied."
echo
"$GDB" -x "$GDB_CMDS" "$ELF"
