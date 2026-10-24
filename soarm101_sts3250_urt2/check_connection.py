#!/usr/bin/env python3
"""只读检查 FE-URT-2 上 ID 1–6 的 STS3250，不写入电机 EEPROM 或目标位置。"""

from __future__ import annotations

import os
from pathlib import Path


def load_env(path: Path) -> None:
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            key, value = line.split("=", 1)
            os.environ.setdefault(key, value)


kit_dir = Path(__file__).resolve().parent
load_env(kit_dir / "config.env")
lerobot_dir = (kit_dir / os.environ["LEROBOT_DIR"]).resolve()
import sys
sys.path.insert(0, str(lerobot_dir / "src"))

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
motors = {
    name: Motor(index, "sts3250", MotorNormMode.DEGREES)
    for index, name in enumerate(names, start=1)
}
bus = FeetechMotorsBus(port=os.environ["FOLLOWER_PORT"], motors=motors)

try:
    bus.connect(handshake=True)
    model = bus._read_model_number(bus.ids, raise_on_error=True)
    position = bus.sync_read("Present_Position", normalize=False)
    voltage = bus.sync_read("Present_Voltage", normalize=False)
    temperature = bus.sync_read("Present_Temperature", normalize=False)
    print("motor\tid\tmodel\tposition\tvoltage_raw\ttemperature_c")
    for name, servo in bus.motors.items():
        print(f"{name}\t{servo.id}\t{model[servo.id]}\t{position[name]}\t{voltage[name]}\t{temperature[name]}")
finally:
    if bus.is_connected:
        bus.disconnect(disable_torque=False)