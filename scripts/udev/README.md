# 设备 udev 规则安装说明

本目录保存当前 Jetson 上与机器人硬件相关的 udev 规则快照，供新设备迁移使用。

## 文件与绑定关系

### 项目自定义规则

| 文件 | 作用 |
|---|---|
| `99-can-names.rules` | 按 USB-CAN 序列号绑定底盘 `can_chassis`、右臂 `can_right`、左臂 `can_left` |
| `can-link-up.sh` | CAN 重命名后自动设置 bitrate 并启动接口 |
| `99-camera-alias.rules` | 按 VID/PID 与物理 USB 路径创建头部、左手、右手相机别名 |
| `99-zlg-usbcanfd.rules` | 给 ZLG `3068:0009` USB-CANFD 设备设置访问权限 |
| `99-dexrobot-libusb.rules` | 给 DexRobot 使用的 `3068:0009` 设备设置访问权限 |

CAN 绑定：

- `0032004B5246571520393733` → `can_chassis`，500000 bit/s
- `0039001E4759530920353131` → `can_right`，1000000 bit/s
- `004400494759530920353131` → `can_left`，1000000 bit/s

目标 Jetson `10.168.1.101` 相机绑定：

- D435 `8086:0b07`，物理路径 `platform-3610000.usb-usb-0:3.3` → `/dev/camera_head_rgb`、`/dev/camera_head_depth`、`/dev/camera_head_ir`
- `1e45:8022`，物理路径 `platform-3610000.usb-usb-0:4.1:1.0` → `/dev/camera_right`
- `1e45:8022`，物理路径 `platform-3610000.usb-usb-0:4.4:1.0` → `/dev/camera_left`

`/dev/camera_head_color` 是 `/dev/camera_head_rgb` 的兼容别名，两者都指向
D435 的彩色采集节点。项目文档和配置统一使用 `camera_head_rgb`。

这些物理路径来自当前 Jetson。迁移到 x86 或新 Jetson 后，先用 `udevadm info` 获取新端口的 `ID_PATH`，再修改规则。

### 系统通用规则快照

| 文件 | 原始位置 | 作用 |
|---|---|---|
| `60-persistent-alsa.rules` | `/lib/udev/rules.d/` | 创建 `/dev/snd/by-id` 和 `/dev/snd/by-path` |
| `90-pulseaudio.rules` | `/lib/udev/rules.d/` | PulseAudio 声卡分类与属性 |
| `60-persistent-v4l.rules` | `/lib/udev/rules.d/` | 创建 `/dev/v4l/by-id` 和 `/dev/v4l/by-path` |
| `60-librealsense2-udev-rules.rules` | `/lib/udev/rules.d/` | Intel RealSense USB 设备权限 |

通用规则通常由系统或 RealSense 软件包提供。把同名文件安装到 `/etc/udev/rules.d/` 会覆盖 `/lib/udev/rules.d/` 中的软件包版本；仅在目标机缺少规则或需要复现当前版本时安装。

## 安装

在仓库根目录执行。

### 1. 安装项目自定义规则

```bash
sudo install -m 0644 scripts/udev/99-can-names.rules /etc/udev/rules.d/
sudo install -m 0644 scripts/udev/99-camera-alias.rules /etc/udev/rules.d/
sudo install -m 0644 scripts/udev/99-zlg-usbcanfd.rules /etc/udev/rules.d/
sudo install -m 0644 scripts/udev/99-dexrobot-libusb.rules /etc/udev/rules.d/
sudo install -m 0755 scripts/udev/can-link-up.sh /usr/local/sbin/
```

`99-zlg-usbcanfd.rules` 与 `99-dexrobot-libusb.rules` 当前匹配同一 VID/PID。按文件名顺序，最终权限由后处理的 ZLG 规则收敛为 `0666`；迁移完成后可合并为一条规则。

这些规则只处理设备访问权限，不会固定 DexHand SDK 的 `adapter_index`。
SDK 仍按 USB 枚举顺序编号；即使按序列号创建 udev 软链接，当前 SDK 也不会
使用该软链接打开设备。左右手索引应根据 SDK 启动日志中的序列号核对，并通过
`left_hand_adapter_index`、`right_hand_adapter_index` 启动参数配置。

### 2. 可选：安装通用规则快照

```bash
sudo install -m 0644 scripts/udev/60-persistent-alsa.rules /etc/udev/rules.d/
sudo install -m 0644 scripts/udev/90-pulseaudio.rules /etc/udev/rules.d/
sudo install -m 0644 scripts/udev/60-persistent-v4l.rules /etc/udev/rules.d/
sudo install -m 0644 scripts/udev/60-librealsense2-udev-rules.rules /etc/udev/rules.d/
```

### 3. 重载并生效

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --action=add
sudo udevadm trigger --subsystem-match=sound --action=change
```

CAN 网络接口和 V4L 相机可能无法在使用中安全重命名。安装后应拔插相关 USB 设备；正式部署建议直接重启一次。

## 验证

### 底盘与机械臂 CAN

```bash
ip -br link show
ip -details link show can_chassis
ip -details link show can_left
ip -details link show can_right
```

期望 `can_chassis` 为 500000 bit/s，`can_left`、`can_right` 为 1000000 bit/s，接口状态为 `UP`。

### 夹爪 ZLG 适配器

```bash
lsusb -d 3068:0009
udevadm monitor --udev --subsystem-match=usb --property
```

当前左右夹爪仍由程序的 `adapter_index=0/1` 区分，udev 规则只保证权限，没有稳定的左右别名。

### 相机

```bash
ls -l /dev/camera_left /dev/camera_right
ls -l /dev/camera_head_rgb /dev/camera_head_color \
  /dev/camera_head_depth /dev/camera_head_ir
ls -l /dev/v4l/by-id/ /dev/v4l/by-path/
rs-enumerate-devices
```

JetPack 6 从源码安装 librealsense 时，应使用同版本源码自带的
`config/99-realsense-libusb.rules`（在本仓库快照中对应
`60-librealsense2-udev-rules.rules`），并确认 `rs-enumerate-devices -s`
无需 root 即可识别设备。项目相机别名规则只负责稳定命名，不能替代
RealSense 的 USB 权限规则。

若别名没有生成，查看设备属性并更新 `99-camera-alias.rules` 中的 `ID_PATH`：

```bash
udevadm info --query=property --name=/dev/video0
udevadm info --attribute-walk --name=/dev/video0
```

### 麦克风与扬声器

```bash
ls -l /dev/snd/by-id/ /dev/snd/by-path/
pactl list short sources
pactl list short sinks
```

项目当前不使用自定义音频别名，而是在 `src/voice_assistant/config/voice_assistant.yaml` 中按名称匹配：

- 麦克风：`XFM-DP`
- 扬声器：`HDA Intel PCH`（PulseAudio 活动端口为 `Headphones`）

## 回滚

删除 `/etc/udev/rules.d/` 中本次安装的同名文件，并重载规则或重启。删除通用规则快照后，系统会重新使用 `/lib/udev/rules.d/` 中由软件包维护的版本。
