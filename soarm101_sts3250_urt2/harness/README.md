# SO101 / STS3250 键盘控制 harness

基于相邻 `lerobot-kinematics/examples/lerobot_keycon_gpos_101.py`（Box2AI Robotics）迁移，复用 SO101 XML、网格和 FK/IK。默认只运行仿真，不连接串口。路径按脚本位置解析，可以从任意工作目录启动。

## 仿真

本机现有 `robot-kin` 环境可运行模型自检和仿真：

```bash
conda activate robot-kin
cd /home/dawntildusk/Desktop/robot_sim/soarm101_sts3250_urt2
python harness/keyboard_control.py --check
python harness/keyboard_control.py
```

`--check` 不打开窗口、不连接硬件。交互模式需要桌面图形环境及 pynput 键盘权限；无头终端不能直接使用窗口和键盘监听。pynput 是全局监听，操作期间不要切到其他应用输入。

按住空格，再按下列按键；松开空格保持当前位置：

| 按键 | 动作 |
| --- | --- |
| w / s | 当前机械臂平面内前后 |
| a / d | 底座正反旋转，并非世界坐标直线左右 |
| r / f | 上下 |
| q / e | 横滚 |
| g / t | 俯仰 |
| z / c | 夹爪向 100 / 0 开度移动 |
| Esc | 退出；真机模式关闭扭矩 |

大臂、肘部、腕俯仰、腕旋转由四关节 IK 求解；底座与夹爪独立控制。不保留原示例的全零复位键。不可达的 IK 请求不更新这四个关节。

```bash
python harness/keyboard_control.py --joint-speed 5 --linear-speed 0.01 --gripper-speed 5 --fps 30
```

单位分别为度/秒、米/秒、开度百分比/秒。关节速度限制可能使末端实际速度低于请求。模型限位与 IK 限位取交集；真机还会与标定行程取交集。显示使用 `mj_forward` 展示姿态，并非动力学或碰撞安全验证。

## 真机依赖

本机 `robot-kin` 是 Python 3.10；当前 LeRobot 真机代码要求 Python 3.12，不能直接混用两个环境的二进制扩展。`robot-lr` 目前缺少 MuJoCo、SciPy 和运动学相关依赖。

在 `robot-lr` 中补齐依赖并重新编译运动学扩展后才能运行真机模式。上游固定的 NumPy 1.24.4 不适合此 Python 版本，因此不能直接照抄上游普通安装命令。可按下面方式保留当前环境的 NumPy，并跳过上游依赖固定版本：

```bash
conda activate robot-lr
python -m pip install mujoco scipy spatialmath-python spatialgeometry ansitable progress pynput setuptools wheel
python -m pip install --no-deps --no-build-isolation -e /home/dawntildusk/Desktop/robot_sim/lerobot-kinematics
python harness/keyboard_control.py --check
```

上述 Python 3.12 编译安装尚未在本次迁移中执行或验证；如编译失败，需要先解决对应扩展兼容性。不要复制 Python 3.10 的 `.so` 到 Python 3.12 使用。

## 关节映射与连接

`mapping.example.json` 是待核对模板，不是你的实际标定。先复制为个人配置：

```bash
cp harness/mapping.example.json harness/mapping.local.json
```

前五项顺序为 `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll`：

```text
真机角度 = sign × 仿真弧度 × 180 / π + offset_deg
```

核对每个关节在模型零位时的真机读数，填入 `offset_deg`；小幅正方向移动以核对 `sign`（只能是 1 或 -1）。结构不变不代表校准零位相同。`gripper_radians_at_0_100` 是真机开度 0 和 100 对应的模型弧度。确认这两端的顺序和范围后再使用夹爪键。核对完成后在个人文件中设置 `verified: true`，否则真机模式拒绝启动。

通信、标定及映射准备好后：

```bash
conda activate robot-lr
python harness/keyboard_control.py --real --mapping harness/mapping.local.json
```

端口和 ID 从工具包根目录 `config.env` 读取；`--port /dev/ttyACM0` 可覆盖端口。复用 STS3250 标定，程序不自动重标定。启动读取实际姿态、检查共同限位、将当前位置写入目标后使能扭矩。不要在标定不匹配时强行绕过检查。

真机窗口显示实际反馈，目标还受驱动单次 1°（夹爪 1 个开度单位）限幅。超过 5° 的关节跟踪偏差、越界反馈或通信异常会结束控制并尝试关闭扭矩；超时由底层串口驱动处理。退出、异常或 Ctrl+C 都可能导致机械臂失去支撑，退出前先托住或放稳机械臂。空格是软件运动使能，不替代物理断电。

本次没有运行真实硬件或验证方向映射、碰撞、动力学和负载。先用低速、小范围逐关节验证，再使用末端控制。
