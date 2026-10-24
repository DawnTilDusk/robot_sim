#!/usr/bin/env bash
set -euo pipefail

KIT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$KIT_DIR/config.env"

if [ ! -d "$LEROBOT_DIR/src/lerobot" ]; then
  echo "找不到 LeRobot：$LEROBOT_DIR" >&2
  exit 1
fi

PATCH_FILE="$KIT_DIR/patches/lerobot-sts3250-follower.patch"
if grep -q 'motor_model: str = "sts3215"' "$LEROBOT_DIR/src/lerobot/robots/so_follower/config_so_follower.py"; then
  echo "STS3250 follower 补丁已经应用。"
  exit 0
fi

(
  cd "$LEROBOT_DIR"
  git apply --check "$PATCH_FILE"
  git apply "$PATCH_FILE"
)
echo "补丁已应用。之后给 follower 命令加 --robot.motor_model=sts3250。"
