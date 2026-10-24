# SO-ARM101（STS3250）+ FE-URT-2 + LeRobot

本目录用于将一台使用 6 颗 Feetech STS3250 的改版 SO-ARM101 接入 [LeRobot](https://github.com/huggingface/lerobot)。它提供 STS3250 型号适配、URT-2 串口检查、零位/行程标定和遥操作启动脚本。

适用范围：Linux、Conda、FE-URT-2、TTL 总线、6 轴 follower。原版 SO-101 leader 可选，用于人工遥操作。

## 重要安全说明

STS3250 是 12 V 级别的串行舵机。USB-C 只为 URT-2 的通信逻辑供电，**不能为机械臂供电**。接线、标定和首次遥操作前，请确认电机电压、外部电源极性、额定电流及机械限位均正确。

```text
电脑 USB-C ──→ FE-URT-2
                 │ TTL-BUS（三针）
                 └──────────────→ STS3250 串联总线

外部直流电源 ──→ FE-URT-2 电源端子 ──→ 舵机总线
```

- STS3250 接 URT-2 的 **TTL-BUS**，不能接 RS485 接口。
- 不要把 12 V 电源接入 URT-2 的 USB-C 口。
- 任何机械调整前，先断开外部舵机电源或关闭扭矩。

## 仓库布局

本工具包默认与 LeRobot 源码相邻放置：

```text
robot_sim/
├── lerobot/                  # Hugging Face LeRobot 源码
└── soarm101_sts3250_urt2/    # 本目录
```

如果你只克隆了本目录所在仓库，请在同一级目录获取 LeRobot：

```bash
git clone https://github.com/huggingface/lerobot.git
```

`config.env` 中的 `LEROBOT_DIR=../lerobot` 对应上述布局；若目录不同，请改为 LeRobot 源码的相对或绝对路径。

## 目录内容

| 文件 | 作用 |
| --- | --- |
| `patches/lerobot-sts3250-follower.patch` | 为 SO follower 增加 `motor_model` 参数的补丁 |
| `apply_lerobot_patch.sh` | 应用 STS3250 支持；重复执行不会重复修改 |
| `config.env.example` | 本机端口、机械臂名称、leader 和相机配置模板 |
| `find_port.sh` | 通过拔插 URT-2 自动找出 Linux 串口 |
| `check_connection.py` | 只读检查 ID 1–6、型号、位置、电压、温度 |
| `setup_motors.sh` | 为尚未编号的新电机逐颗设置 ID 与波特率 |
| `calibrate_follower.sh` | 写入机械中位、关节安全行程和 LeRobot 标定文件 |
| `teleoperate.sh` | 原版 SO-101 leader 到 STS3250 follower 的限幅遥操作 |

## 1. 创建 Conda 环境并安装 LeRobot

当前 LeRobot 源码要求 Python 3.12 或更高版本。以下命令不依赖 `uv`：

```bash
conda create -n soarm101 python=3.12 -y
conda activate soarm101

cd /path/to/robot_sim/lerobot
python -m pip install --upgrade pip
python -m pip install -e ".[feetech]"
```

验证飞特驱动：

```bash
python -c "from lerobot.motors.feetech import FeetechMotorsBus; print('Feetech driver: OK')"
```

若已有符合要求的 Conda 环境，直接激活它即可，无需重复创建：

```bash
conda activate <your-environment>
```

## 2. 授予当前用户串口权限

插上 URT-2 后，Linux 常显示 `/dev/ttyACM0` 或 `/dev/ttyUSB0`。设备通常只允许 `dialout` 用户组访问。

```bash
sudo usermod -aG dialout "$USER"
```

执行后必须注销并重新登录图形会话，再打开终端。用下列命令确认：

```bash
id -nG
```

输出应含有 `dialout`。若设备所属组不是 `dialout`，以此命令显示的组名为准：

```bash
ls -l /dev/ttyACM0
```

## 3. 创建本机配置并找到 URT-2 端口

```bash
cd /path/to/robot_sim/soarm101_sts3250_urt2
cp config.env.example config.env
bash find_port.sh
```

运行 `find_port.sh` 后：保持 URT-2 已连接，按提示拔掉 USB-C，按 Enter；脚本会输出端口，例如：

```text
The port of this MotorsBus is '/dev/ttyACM0'
```

重新插上 URT-2，然后编辑 `config.env`：

```bash
FOLLOWER_PORT=/dev/ttyACM0
```

保留 `PYTHON_CMD=python`，它会使用当前已激活的 Conda 环境。若使用原版 SO-101 leader，也填写 `LEADER_PORT`；没有 leader 时无需运行遥操作脚本。

## 4. 应用 STS3250 适配

原版 SO-101 的 LeRobot 类默认使用 STS3215。本包将其改为可由命令行选择型号：

```bash
cd /path/to/robot_sim/soarm101_sts3250_urt2
bash apply_lerobot_patch.sh
```

补丁后，follower 命令使用 `--robot.motor_model=sts3250`。补丁只修改：

- `lerobot/src/lerobot/robots/so_follower/config_so_follower.py`

- `lerobot/src/lerobot/robots/so_follower/so_follower.py`

## 5. 检查电机通信

给 URT-2 接上符合 STS3250 规格的外部电源，确认所有电机已串联，再执行：

```bash
python check_connection.py
```

该脚本只读取数据，不写入 ID、零位、限位或目标位置。成功时应显示六行信息，ID 为 `1–6`，每个 `model` 为 `2825`。

如果出现 `Permission denied: /dev/ttyACM0`，说明第 2 步的组权限尚未在当前登录会话中生效。若出现缺少 ID 或型号不符，先检查串线、电源和电机配置。

## 6. 电机 ID

已按下表设置 ID 的机械臂，可跳过 `setup_motors.sh`：

| ID | LeRobot 关节 | 机械位置 |
| ---: | --- | --- |
| 1 | `shoulder_pan` | 底座旋转 |
| 2 | `shoulder_lift` | 大臂 |
| 3 | `elbow_flex` | 小臂/肘部 |
| 4 | `wrist_flex` | 腕俯仰 |
| 5 | `wrist_roll` | 腕旋转 |
| 6 | `gripper` | 夹爪 |

若电机均为新件且 ID 冲突，才运行：

```bash
bash setup_motors.sh
```

脚本会按提示一次只连接一颗电机，并分配上述 ID；不要在全部新电机串联时运行。

## 7. 校准零位与行程

通信检查通过后执行：

```bash
bash calibrate_follower.sh
```

程序会先关闭扭矩。将机械臂手动摆到你定义的中位姿态后按 Enter，再依次手动移动除 `wrist_roll` 外的关节至完整、安全的机械行程。它会把当前中位写入 STS3250 的 `Homing_Offset`，并保存最小和最大位置。

校准后不要再用其他工具单独更改零位或行程；否则舵机内部参数与 LeRobot 标定文件会不一致。需要重新定义零位时，重新执行本步骤。

## 8. 遥操作

首次遥操作前保持 `config.env` 中：

```bash
MAX_RELATIVE_TARGET=5
```

它限制单次目标变化，便于先确认运动方向和机械限位。连接原版 SO-101 leader 后运行：

```bash
bash teleoperate.sh
```

确认所有关节方向正确后，再按需要逐步提高限幅。leader 若也使用 STS3250，需要为 leader 做独立的型号适配和标定，本工具包默认 leader 为原版 STS3215。

## 常见问题

| 现象 | 处理方式 |
| --- | --- |
| 能看到 `/dev/ttyACM0`，但脚本报 `Permission denied` | 将当前用户加入设备所属组，注销并重新登录。 |
| `check_connection.py` 找不到电机 | 确认外部舵机电源已接通、URT-2 接在 TTL-BUS、所有线缆牢靠。USB 识别不代表舵机已供电。 |
| 只识别到 ID 1 | 新电机可能都仍为 ID 1；逐颗运行 `setup_motors.sh`。 |
| `model` 不是 2825 | 电机并非标准 STS3250，或固件/型号不同；停止后核对硬件型号。 |
| 校准或运动时通信中断 | 检查电源电流余量、接线端子、总线线缆和机械卡滞。 |

## 与 CaP-X 的关系

本目录解决的是 STS3250 真机通信、标定与 LeRobot 控制。CaP-X 当前的 Panda RoboSuite 仿真不能直接发送到这台 6 轴机械臂；后续需要建立 SO-ARM101 的运动学模型、关节映射和真机环境接口。
