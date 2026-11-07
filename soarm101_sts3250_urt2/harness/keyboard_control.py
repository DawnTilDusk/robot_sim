"""SO101 keyboard harness adapted from lerobot-kinematics (Box2AI Robotics).

Simulation by default. Hardware output requires --real and a verified mapping.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time

KIT = Path(__file__).resolve().parents[1]
NAMES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('必须为大于零的有限数值')
    return value


def main():
    parser = argparse.ArgumentParser(description='SO101 键盘控制：默认仿真，按住空格再按方向键运动')
    parser.add_argument('--real', action='store_true', help='连接 STS3250 真机')
    parser.add_argument('--mapping', type=Path, help='经过实物核对的关节映射 JSON')
    parser.add_argument('--port', help='覆盖 config.env 的 FOLLOWER_PORT')
    parser.add_argument('--fps', type=positive, default=30)
    parser.add_argument('--joint-speed', type=positive, default=5, help='关节最大目标速度，度/秒')
    parser.add_argument('--linear-speed', type=positive, default=0.01, help='末端目标速度，米/秒')
    parser.add_argument('--gripper-speed', type=positive, default=5, help='夹爪开度百分比/秒')
    parser.add_argument('--check', action='store_true', help='加载模型并验证 FK/IK，不打开窗口或连接硬件')
    args = parser.parse_args()
    if args.real and args.mapping is None:
        parser.error('--real 需要 --mapping，先核对方向、零位和夹爪端点')

    config = {}
    for line in (KIT / 'config.env').read_text().splitlines():
        if line.strip() and not line.lstrip().startswith('#'):
            key, value = line.split('=', 1)
            config[key.strip()] = value.strip()
    kinematics = KIT.parent / 'lerobot-kinematics'
    sys.path.insert(0, str(kinematics))
    sys.path.insert(0, str((KIT / config['LEROBOT_DIR']).resolve() / 'src'))
    import numpy as np
    import mujoco
    from lerobot_kinematics import get_robot, lerobot_FK, lerobot_IK

    model = mujoco.MjModel.from_xml_path(str(kinematics / 'examples/scene_so101.xml'))
    data = mujoco.MjData(model)
    indices = [model.jnt_qposadr[model.joint(name).id] for name in NAMES]
    limits = np.array([model.jnt_range[model.joint(name).id] for name in NAMES])
    kin = get_robot('so101')
    limits[1:5, 0] = np.maximum(limits[1:5, 0], kin.qlim[0])
    limits[1:5, 1] = np.minimum(limits[1:5, 1], kin.qlim[1])
    mapping = json.loads((args.mapping or Path(__file__).with_name('mapping.example.json')).read_text())
    signs = np.asarray(mapping['sign'], dtype=float)
    offsets = np.asarray(mapping['offset_deg'], dtype=float)
    grip = np.asarray(mapping['gripper_radians_at_0_100'], dtype=float)
    if (signs.shape != (5,) or offsets.shape != (5,) or grip.shape != (2,)
            or not np.all(np.isin(signs, [-1, 1])) or not np.all(np.isfinite(offsets))
            or not np.all(np.isfinite(grip)) or grip[0] == grip[1]):
        raise ValueError('无效映射：sign 为五个 ±1，offset_deg 为五个有限数，夹爪为两个不同的有限端点')
    if args.real and not mapping.get('verified', False):
        raise ValueError('映射尚未核对；核对后在你自己的 JSON 中设置 verified=true')

    def from_observation(obs):
        degrees = np.array([obs[f'{name}.pos'] for name in NAMES[:5]])
        return np.r_[np.deg2rad((degrees - offsets) / signs),
                     grip[0] + obs['gripper.pos'] / 100 * (grip[1] - grip[0])]

    def to_action(q):
        values = np.r_[signs * np.rad2deg(q[:5]) + offsets,
                       (q[5] - grip[0]) / (grip[1] - grip[0]) * 100]
        return {f'{name}.pos': float(value) for name, value in zip(NAMES, values)}

    q = np.zeros(6)
    pose = lerobot_FK(q[1:5], robot=kin)
    solution, ok = lerobot_IK(q[1:5].copy(), pose, robot=kin)
    if not ok or not np.all(np.isfinite(solution)):
        raise RuntimeError('初始 FK/IK 自检失败')
    if args.check:
        assert np.allclose(from_observation(to_action(q)), q)
        print('模型、关节名称、FK/IK 和映射往返检查通过；未连接硬件。')
        return

    import mujoco.viewer
    from pynput import keyboard

    arm = None
    listener = None
    lock = threading.Lock()
    pressed = set()
    stop = threading.Event()

    def key_name(key):
        if key == keyboard.Key.space:
            return 'space'
        return getattr(key, 'char', '') or ''

    def on_press(key):
        if key == keyboard.Key.esc:
            stop.set()
        with lock:
            pressed.add(key_name(key).lower())

    def on_release(key):
        with lock:
            pressed.discard(key_name(key).lower())

    try:
        if args.real:
            from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
            from lerobot.robots.so_follower.so_follower import SOFollower
            arm = SOFollower(SOFollowerRobotConfig(
                port=args.port or config['FOLLOWER_PORT'], id=config['FOLLOWER_ID'],
                motor_model='sts3250', use_degrees=True, max_relative_target=1.0,
            ))
            if not arm.calibration:
                raise RuntimeError('缺少标定，请先运行 tools/calibrate_follower.sh')
            input('确认映射已核对、机械臂固定且路径无障碍，按 Enter 连接并使能扭矩…')
            # Check calibration before configure() can enable torque; never recalibrate here.
            arm.bus.connect()
            if not arm.is_calibrated:
                raise RuntimeError('电机与标定文件不一致，请先单独标定')
            q = from_observation(arm.get_observation())
            for i, name in enumerate(NAMES[:5]):
                cal = arm.calibration[name]
                mid = (cal.range_min + cal.range_max) / 2
                raw_limits = np.array([cal.range_min, cal.range_max])
                degree_limits = (raw_limits - mid) * 360 / 4095
                mapped = np.sort(np.deg2rad((degree_limits - offsets[i]) / signs[i]))
                limits[i, 0] = max(limits[i, 0], mapped[0])
                limits[i, 1] = min(limits[i, 1], mapped[1])
            limits[5, 0] = max(limits[5, 0], min(grip))
            limits[5, 1] = min(limits[5, 1], max(grip))
            if np.any(limits[:, 0] >= limits[:, 1]) or np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
                raise RuntimeError('实际姿态或映射不在模型/标定共同限位内，拒绝运动')
            # Seed each goal from its encoder before enabling torque, so stale goals
            # from an earlier session cannot pull the arm toward an old pose.
            arm.bus.disable_torque()
            arm.bus.configure_motors()
            for name in NAMES:
                arm.bus.write('Operating_Mode', name, 0)  # Feetech position mode
                arm.bus.write('P_Coefficient', name, arm.config.position_p_coefficient)
                arm.bus.write('I_Coefficient', name, arm.config.position_i_coefficient)
                arm.bus.write('D_Coefficient', name, arm.config.position_d_coefficient)
            arm.bus.write('Max_Torque_Limit', 'gripper', 500)
            arm.bus.write('Protection_Current', 'gripper', 250)
            arm.bus.write('Overload_Torque', 'gripper', 25)
            raw = arm.bus.sync_read('Present_Position', normalize=False)
            arm.bus.sync_write('Goal_Position', raw, normalize=False)
            q = from_observation(arm.get_observation())
            if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
                raise RuntimeError('使能前姿态已超出共同限位，拒绝运动')
            arm.bus.enable_torque()

        target = q.copy()
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        print('按住空格启用运动；w/s 前后，a/d 底座，r/f 上下，q/e 横滚，g/t 俯仰，z/c 夹爪。')
        print('松开空格保持；Esc 退出并关闭扭矩（先支撑机械臂）。不提供全零复位键。')
        last = time.monotonic()
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and not stop.is_set():
                started = time.monotonic()
                dt = min(started - last, 1 / args.fps)
                last = started
                if arm:
                    q = from_observation(arm.get_observation())
                    if not np.all(np.isfinite(q)) or np.any(q < limits[:, 0] - 0.02) or np.any(q > limits[:, 1] + 0.02):
                        raise RuntimeError('反馈超出限位，停止控制')
                    if np.any(np.abs(q[:5] - target[:5]) > np.deg2rad(5)):
                        raise RuntimeError('实际位置偏离目标超过 5°，停止控制')
                with lock:
                    keys = pressed.copy()
                candidate = target.copy()
                def axis(plus, minus):
                    return int(plus in keys) - int(minus in keys)
                if 'space' in keys:
                    candidate[0] += axis('a', 'd') * np.deg2rad(args.joint_speed) * dt
                    candidate[5] += axis('z', 'c') * (grip[1] - grip[0]) * args.gripper_speed / 100 * dt
                    pose = lerobot_FK(target[1:5], robot=kin)
                    pose[0] += axis('w', 's') * args.linear_speed * dt
                    pose[2] += axis('r', 'f') * args.linear_speed * dt
                    pose[3] += axis('q', 'e') * np.deg2rad(args.joint_speed) * dt
                    pose[4] += axis('g', 't') * np.deg2rad(args.joint_speed) * dt
                    if keys.intersection('wsrfqegt'):
                        solved, success = lerobot_IK(target[1:5].copy(), pose, robot=kin)
                        if success and np.all(np.isfinite(solved)):
                            candidate[1:5] = solved
                else:
                    candidate = q.copy()
                candidate = np.clip(candidate, limits[:, 0], limits[:, 1])
                rate = np.r_[np.full(5, np.deg2rad(args.joint_speed)),
                             abs(grip[1] - grip[0]) * args.gripper_speed / 100]
                target = target + np.clip(candidate - target, -rate * dt, rate * dt)
                if arm:
                    # Respect the driver's actual clamped command, avoiding accumulating unreachable targets.
                    target = from_observation(arm.send_action(to_action(target)))
                else:
                    q = target.copy()
                data.qpos[indices] = q
                mujoco.mj_forward(model, data)
                viewer.sync()
                time.sleep(max(0, 1 / args.fps - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print('已中断。')
    finally:
        if listener:
            listener.stop()
        if arm and arm.bus.is_connected:
            arm.bus.disconnect(disable_torque=True)


if __name__ == '__main__':
    main()
