"""Human-guided SO101 simulation/hardware alignment; no motion commands."""

import argparse
import json
import math
from pathlib import Path
import select
import statistics
import sys
import time

from keyboard_control import KIT, NAMES
from follow_real import load_mapping


def single_pose_mapping(measured, reference_deg, signs, grip):
    """Align offsets only; one pose cannot establish joint directions."""
    if len(reference_deg) != 5 or len(signs) != 5 or any(v not in (-1, 1) for v in signs):
        raise ValueError('需要五个参考角度和五个 ±1 方向')
    offsets = [measured[name] - signs[i] * reference_deg[i]
               for i, name in enumerate(NAMES[:5])]
    if not all(math.isfinite(v) for v in [*reference_deg, *offsets]):
        raise ValueError('角度必须是有限数')
    return dict(verified=False, sign=list(signs), offset_deg=offsets,
                gripper_radians_at_0_100=list(grip),
                alignment_method='single_pose', reference_deg=list(reference_deg),
                alignment_samples={name: measured[name] for name in NAMES[:5]})


def wait_for_enter(viewer):
    """Keep the viewer responsive while waiting for a terminal confirmation (Linux)."""
    while viewer.is_running():
        viewer.sync()
        if select.select([sys.stdin], [], [], 0.03)[0]:
            if sys.stdin.readline() == '':
                raise RuntimeError('终端输入已关闭')
            return
    raise KeyboardInterrupt


def stable_sample(bus, viewer, names):
    readings = []
    for _ in range(15):
        if not viewer.is_running():
            raise KeyboardInterrupt
        readings.append(bus.sync_read('Present_Position', num_retry=2))
        viewer.sync()
        time.sleep(0.03)
    result = {}
    for name in names:
        values = [row[name] for row in readings]
        if not all(math.isfinite(v) for v in values) or max(values) - min(values) > 1.5:
            raise ValueError(f'{name} 采样时不稳定，请保持不动后重试')
        result[name] = statistics.median(values)
    return result


def main():
    parser = argparse.ArgumentParser(description='单姿态对齐：摆好一次，采样前五轴零位偏移；保留方向和夹爪映射')
    parser.add_argument('--port')
    parser.add_argument('--calibration', type=Path)
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('mapping.local.json'))
    parser.add_argument('--mapping', type=Path, help='沿用此映射的方向和夹爪端点；默认优先 mapping.local.json')
    parser.add_argument('--reference-deg', nargs=5, type=float, default=[0.0] * 5,
                        metavar='DEG', help='前五轴参考角度，默认全部为零')
    parser.add_argument('--release-torque', action='store_true', help='确认后关闭扭矩；退出不恢复')
    args = parser.parse_args()
    if args.output.exists():
        parser.error(f'不覆盖已有文件 {args.output}；请用 --output 指定新文件')
    source = args.mapping
    if source is None:
        local = Path(__file__).with_name('mapping.local.json')
        source = local if local.exists() else Path(__file__).with_name('mapping.example.json')
    signs, _, grip = load_mapping(source)
    if not all(math.isfinite(v) for v in args.reference_deg):
        parser.error('参考角度必须是有限数')
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
    calibration = {name: MotorCalibration(**v) for name, v in json.loads(path.read_text()).items()}
    for i, name in enumerate(NAMES, 1):
        if name not in calibration or calibration[name].id != i or calibration[name].range_min >= calibration[name].range_max:
            raise ValueError(f'标定缺失或无效：{name}')
    motors = {name: Motor(i, 'sts3250', MotorNormMode.RANGE_0_100 if name == 'gripper' else MotorNormMode.DEGREES)
              for i, name in enumerate(NAMES, 1)}
    model = mujoco.MjModel.from_xml_path(str(KIT.parent / 'lerobot-kinematics/examples/scene_so101.xml'))
    data = mujoco.MjData(model)
    indices = [model.jnt_qposadr[model.joint(name).id] for name in NAMES]
    reference = [math.radians(v) for v in args.reference_deg] + [sum(grip) / 2]
    for name, value in zip(NAMES, reference):
        lo, hi = model.jnt_range[model.joint(name).id]
        if not lo <= value <= hi:
            raise ValueError(f'参考姿态超出模型限位：{name}')
    bus = FeetechMotorsBus(port=args.port or config['FOLLOWER_PORT'], motors=motors, calibration=calibration)
    try:
        bus.connect()
        if not bus.is_calibrated:
            raise RuntimeError('电机与标定文件不一致，请先单独标定')
        if args.release_torque:
            input('先托住机械臂，按 Enter 关闭扭矩；本工具不会再使能扭矩…')
            bus.disable_torque()
        elif any(bus.sync_read('Torque_Enable', normalize=False).values()):
            raise RuntimeError('电机扭矩开启，请托住机械臂后使用 --release-torque，勿强行手掰')
        print('请对照连杆相对角度，保持实物与模型底座朝向一致；窗口只显示参考姿态。')
        print('按 Enter 请在终端操作。若姿态不可达或不确定，请 Ctrl+C 退出，不要硬掰限位。')
        print(f'沿用方向 {signs}；夹爪端点来自 {source}。此次不采集夹爪。')
        data.qpos[indices] = reference
        mujoco.mj_forward(model, data)
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print('将真机前五个关节摆成图中姿态；夹爪开合无需对齐。')
            print('摆好后在终端按一次 Enter，保持静止约半秒。')
            while True:
                wait_for_enter(viewer)
                try:
                    measured = stable_sample(bus, viewer, NAMES[:5])
                    break
                except ValueError as error:
                    print(f'{error}；保持不动，按 Enter 重试。')
            result = single_pose_mapping(measured, args.reference_deg, signs, grip)
            result['calibration_file'] = str(path)
            result['source_mapping'] = str(source)
            with args.output.open('x') as output:
                json.dump(result, output, ensure_ascii=False, indent=2)
                output.write('\\n')
            print(f'已保存：{args.output}')
            print('零位偏移（度）：', result['offset_deg'])
            print('单姿态不验证方向，保存为 verified=false。先用 follow_real.py 检查运动方向，')
            print('确认各关节与夹爪映射正确后，再将此文件的 verified 改为 true 用于真机键盘控制。')
    except KeyboardInterrupt:
        print('\n已取消对齐，未生成新的映射文件。')
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


if __name__ == '__main__':
    main()
