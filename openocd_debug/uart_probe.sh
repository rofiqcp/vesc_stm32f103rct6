#!/usr/bin/env bash
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PORT="${1:-/dev/ttyUSB0}"
BAUD="${2:-115200}"
cd "$ROOT"
echo "=== raw VESC handshake: $PORT @ $BAUD ==="
python3 tools/debug.py handshake --port "$PORT" --baud "$BAUD" --attempts 5 --timeout 0.7 && exit 0

echo
echo "=== no valid frame; scan common baud rates ==="
python3 tools/debug.py baud-scan --port "$PORT" --baud "$BAUD" --timeout 0.7
