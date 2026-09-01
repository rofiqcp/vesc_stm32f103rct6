#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ENV_NAME="stm32f103rc"
ELF="$ROOT/.pio/build/$ENV_NAME/firmware.elf"
CFG="$SCRIPT_DIR/openocd_recover_under_reset.cfg"

PIO="${PIO:-$(command -v pio 2>/dev/null || true)}"
[[ -n "$PIO" ]] || PIO="$HOME/.platformio/penv/bin/pio"
OPENOCD="${OPENOCD:-$(command -v openocd 2>/dev/null || true)}"

[[ -x "$PIO" ]] || { echo "[ERROR] pio not found" >&2; exit 1; }
[[ -n "$OPENOCD" ]] || { echo "[ERROR] openocd not found" >&2; exit 1; }

cd "$ROOT"
echo "[RECOVERY] Build release firmware"
"$PIO" run -e "$ENV_NAME"
[[ -f "$ELF" ]] || { echo "[ERROR] ELF missing: $ELF" >&2; exit 1; }

echo "[RECOVERY] Connect-under-reset @ 200 kHz and program ELF"
echo "           NRST must be connected. This is for replacing an OLD bad image only."
"$OPENOCD" -f "$CFG" \
  -c "init" \
  -c "reset halt" \
  -c "program $ELF verify" \
  -c "reset run" \
  -c "shutdown"

echo "[RECOVERY] Done. This fixed firmware keeps SWD enabled; use normal upload next time."
