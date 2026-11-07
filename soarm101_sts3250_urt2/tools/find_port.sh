#!/usr/bin/env bash
set -euo pipefail

KIT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$KIT_DIR/config.env"
if [[ "$LEROBOT_DIR" != /* ]]; then
  LEROBOT_DIR="$KIT_DIR/$LEROBOT_DIR"
fi
PYTHON_CMD="${PYTHON_CMD:-python}"
cd "$LEROBOT_DIR"

"$PYTHON_CMD" -m lerobot.scripts.lerobot_find_port
