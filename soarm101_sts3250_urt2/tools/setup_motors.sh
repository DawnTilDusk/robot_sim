#!/usr/bin/env bash
set -euo pipefail

KIT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$KIT_DIR/config.env"
if [[ "$LEROBOT_DIR" != /* ]]; then
  LEROBOT_DIR="$KIT_DIR/$LEROBOT_DIR"
fi
PYTHON_CMD="${PYTHON_CMD:-python}"
cd "$LEROBOT_DIR"

"$PYTHON_CMD" -m lerobot.scripts.lerobot_setup_motors \
  --robot.type=so101_follower \
  --robot.port="$FOLLOWER_PORT" \
  --robot.id="$FOLLOWER_ID" \
  --robot.motor_model=sts3250
