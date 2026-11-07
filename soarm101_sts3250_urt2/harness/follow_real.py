"""Display measured STS3250 joint positions in MuJoCo; never command motion."""

import argparse
import json
import math
from pathlib import Path
import sys
import time

from keyboard_control import KIT, NAMES, positive, hardware_limits, check_pose_limits


def load_mapping(path):
    mapping = json.loads(path.read_text())
    signs = mapping['sign']
    offsets = mapping['offset_deg']
    grip = mapping['gripper_radians_at_0_100']
    if (len(signs) != 5 or len(offsets) != 5 or len(grip) != 2
            or any(s not in (-1, 1) for s in signs)
            or not all(math.isfinite(v) for v in [*offsets, *grip])
            or grip[0] == grip[1]):
        raise ValueError('无效关节映射')
    return signs, offsets, grip


def map_positions(positions, mapping):
    signs, offsets, grip = mapping
    q = [math.radians((positions[name] - offsets[i]) / signs[i])
         for i, name in enumerate(NAMES[:5])]
    q.append(grip[0] + positions['gripper'] / 100 * (grip[1] - grip[0]))
    if not all(math.isfinite(v) for v in q):
        raise ValueError('收到非有限位置数据')
    return q


def mirror(bus, model, data, viewer, mujoco, mapping, fps):
    indices = [model.jnt_qposadr[model.joint(name).id] for name in NAMES]
    limits = hardware_limits(bus.calibration, *mapping)
    for name, bounds in zip(NAMES, limits):
        model.jnt_range[model.joint(name).id] = bounds
    next_log = 0.0
    while viewer.is_running():
        started = time.monotonic()
        positions = bus.sync_read('Present_Position', num_retry=2)
        q = map_positions(positions, mapping)
        # Do not clamp: displaying the measured pose helps diagnose bad offsets.
        with viewer.lock():
            data.qpos[indices] = q
            mujoco.mj_forward(model, data)
        viewer.sync()
        if started >= next_log:
            print('实测（前五轴°，夹爪%）：' + '  '.join(
                f'{name}={positions[name]:.1f}' for name in NAMES))
            try:
                check_pose_limits(q, limits, *mapping, '反馈超出真机标定范围：')
            except RuntimeError as error:
                # Read-only inspection should keep displaying the measured pose.
                print(error)
            next_log = started + 1
        time.sleep(max(0, 1 / fps - (time.monotonic() - started)))


def main():
    parser = argparse.ArgumentParser(description='手动移动真机，MuJoCo 跟随显示；默认只读，不改变扭矩')
    parser.add_argument('--port', help='覆盖 config.env 中的串口')
    parser.add_argument('--calibration', type=Path, help='可选：指定 LeRobot 标定 JSON')
    parser.add_argument('--mapping', type=Path, default=Path(__file__).with_name('mapping.local.json'),
                        help='关节映射文件，默认使用脚本目录下的 mapping.local.json')
    parser.add_argument('--fps', type=positive, default=30)
    parser.add_argument('--release-torque', action='store_true', help='提示确认后关闭扭矩，便于手动移动；退出不恢复')
    args = parser.parse_args()
    mapping = load_mapping(args.mapping)
    config = {}
    for line in (KIT / 'config.env').read_text().splitlines():
        if line.strip() and not line.lstrip().startswith('#'):
            key, value = line.split('=', 1)
            config[key.strip()] = value.strip()
    sys.path.insert(0, str((KIT / config['LEROBOT_DIR']).resolve() / 'src'))

    import mujoco
    import mujoco.viewer
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS

    path = args.calibration or HF_LEROBOT_CALIBRATION / ROBOTS / 'so_follower' / f"{config['FOLLOWER_ID']}.json"
    calibration = {name: MotorCalibration(**value) for name, value in json.loads(path.read_text()).items()}
    for i, name in enumerate(NAMES, 1):
        if name not in calibration or calibration[name].id != i or calibration[name].range_min >= calibration[name].range_max:
            raise ValueError(f'标定缺失或无效：{name}')
    motors = {name: Motor(i, 'sts3250', MotorNormMode.RANGE_0_100 if name == 'gripper' else MotorNormMode.DEGREES)
              for i, name in enumerate(NAMES, 1)}
    model = mujoco.MjModel.from_xml_path(str(KIT.parent / 'lerobot-kinematics/examples/scene_so101.xml'))
    for name, bounds in zip(NAMES, hardware_limits(calibration, *mapping)):
        model.jnt_range[model.joint(name).id] = bounds
    data = mujoco.MjData(model)
    bus = FeetechMotorsBus(port=args.port or config['FOLLOWER_PORT'], motors=motors, calibration=calibration)
    try:
        bus.connect()
        if not bus.is_calibrated:
            raise RuntimeError('电机标定与文件不一致；请先单独运行标定工具')
        if args.release_torque:
            input('请先托住机械臂，按 Enter 关闭扭矩；之后可以手动移动，退出不恢复扭矩…')
            bus.disable_torque()
        elif any(bus.sync_read('Torque_Enable', normalize=False).values()):
            raise RuntimeError('当前电机扭矩开启，请勿强行手掰。托住机械臂后用 --release-torque 重新启动。')
        print(f'已连接 {bus.port}，只读位置跟随，不发送目标、不修改标定。')
        print(f'映射：{args.mapping}（允许未验证映射用于对齐）。关闭窗口或 Ctrl+C 退出。')
        initial = map_positions(bus.sync_read('Present_Position', num_retry=2), mapping)
        data.qpos[[model.jnt_qposadr[model.joint(name).id] for name in NAMES]] = initial
        mujoco.mj_forward(model, data)
        with mujoco.viewer.launch_passive(model, data) as viewer:
            mirror(bus, model, data, viewer, mujoco, mapping, args.fps)
    except KeyboardInterrupt:
        print('已停止跟随显示。')
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


if __name__ == '__main__':
    main()
