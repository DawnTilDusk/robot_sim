"""Standalone follower motion tool; run with --help for command-line options."""

import argparse
import math
import time


def main():
    parser = argparse.ArgumentParser(description="无需主臂，控制从臂底座相对转动")
    parser.add_argument("--angle", type=float, required=True,
                        help="相对转角（度），负数表示反方向")
    parser.add_argument("--speed", type=float, default=5.0,
                        help="目标推进速度（度/秒），默认 5，必须大于 0")
    args = parser.parse_args()
    if not math.isfinite(args.angle):
        parser.error("--angle 必须是有限数值")
    if not math.isfinite(args.speed) or args.speed <= 0:
        parser.error("--speed 必须是大于 0 的有限数值")
    if args.angle == 0:
        print("转角为 0，无需运动。")
        return

    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    from lerobot.robots.so_follower.so_follower import SOFollower

    robot = SOFollower(SOFollowerRobotConfig(
        port="/dev/ttyACM0",
        id="soarm101_sts3250",
        motor_model="sts3250",
        use_degrees=True,
        max_relative_target=1.0,
    ))
    try:
        robot.connect()
        observation = robot.get_observation()
        target = {k: v for k, v in observation.items() if k.endswith(".pos")}
        print("当前关节位置：", target)
        duration = abs(args.angle) / args.speed
        print(f"计划相对转动 {args.angle:g}°，目标速度 {args.speed:g}°/秒，"
              f"预计至少 {duration:.1f} 秒。")
        input("确认机械臂固定、整个转动路径无障碍且不超过限位，按 Enter 开始…")
        start = target["shoulder_pan.pos"]
        # Keep individual target increments small even at high requested speeds.
        steps = max(1, math.ceil(duration / 0.03), math.ceil(abs(args.angle) / 0.5))
        for step in range(1, steps + 1):
            target["shoulder_pan.pos"] = start + args.angle * step / steps
            robot.send_action(target)
            time.sleep(duration / steps)
        input("运动结束。托住机械臂后按 Enter，关闭扭矩…")
    except KeyboardInterrupt:
        print("\n已中断，关闭扭矩；请注意机械臂可能下落。")
    finally:
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()
