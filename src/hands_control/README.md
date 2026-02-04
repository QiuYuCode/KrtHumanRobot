# hands_control

ROS 2 功能包，用于通过 Action 接口控制两个 DexHand021S 灵巧手设备。

## 功能特性

- 支持同时控制两个 DexHand021S 设备（左手和右手）
- 基于 ROS 2 Action 接口提供异步控制
- 支持单个手指精确控制
- 支持手部重置功能
- 提供实时位置反馈

## 依赖

### 系统依赖
- ROS 2 (Humble/Foxy)
- Python 3.10+
- DexHand Python SDK

### ROS 2 依赖
- `rclpy`
- `rclpy-action`
- `hands_control_interfaces` (本项目包含)

### 安装 DexHand SDK

```bash
# 克隆 SDK
git clone https://github.com/DexRobot/dexhand_sdk_python.git
cd dexhand_sdk_python

# 安装
pip3 install -e .
```

## 构建

```bash
cd /home/create/DataDisk/WorkSpace/CrtWorkSpace/KrtHumanRobot

# 构建接口包
colcon build --packages-select hands_control_interfaces --symlink-install

# 构建控制包
colcon build --packages-select hands_control --symlink-install

source install/setup.bash
```

## 使用方法

### 1. 启动 Action Server

```bash
# 使用默认参数启动
ros2 launch hands_control hand_control_launch.py

# 自定义参数启动
ros2 launch hands_control hand_control_launch.py \
    adapter_type:=ZLG_MINI \
    left_hand_device_id:=1 \
    right_hand_device_id:=2
```

**参数说明:**
- `adapter_type`: ZLG 适配器类型 (默认: ZLG_MINI, 支持: ZLG_MINI, ZLG_200U)
- `left_hand_device_id`: 左手设备 ID (默认: 1, 即 0x01)
- `right_hand_device_id`: 右手设备 ID (默认: 2, 即 0x02)

### 2. 测试客户端

```bash
# 运行测试客户端（包含示例动作）
ros2 run hands_control hand_control_client
```

### 3. 命令行调用 Action

#### 控制手指移动

```bash
# 左手手指 1 移动到位置 1000
ros2 action send_goal /hand_control hands_control_interfaces/action/HandControl \
    "{adapter_index: 0, finger_id: 1, position: 1000, speed: 600, force: 85, wait_time: 10}" \
    --feedback

# 左手所有手指闭合
ros2 action send_goal /hand_control hands_control_interfaces/action/HandControl \
    "{adapter_index: 0, finger_id: 0, position: 1000, speed: 600, force: 85, wait_time: 10}" \
    --feedback

# 右手所有手指张开
ros2 action send_goal /hand_control hands_control_interfaces/action/HandControl \
    "{adapter_index: 1, finger_id: 0, position: 0, speed: 600, force: 85, wait_time: 10}" \
    --feedback

# 右手手指 2 移动到位置 800
ros2 action send_goal /hand_control hands_control_interfaces/action/HandControl \
    "{adapter_index: 1, finger_id: 2, position: 800, speed: 500, force: 85, wait_time: 10}" \
    --feedback
```

#### 重置手部

```bash
# 重置左手
ros2 action send_goal /reset_hand hands_control_interfaces/action/ResetHand \
    "{adapter_index: 0}" --feedback

# 重置右手
ros2 action send_goal /reset_hand hands_control_interfaces/action/ResetHand \
    "{adapter_index: 1}" --feedback
```

## Action 接口说明

### HandControl.action

**Goal (目标):**
- `uint8 adapter_index` - 适配器索引 (0=左手, 1=右手)
- `uint8 finger_id` - 手指 ID (1, 2, 3；0 表示所有手指)
- `int32 position` - 目标位置 (0-1000)
- `int32 speed` - 速度 (0-1000)
- `uint8 force` - 力度 (0x00-0xFF, 推荐 0x55=85)
- `uint8 wait_time` - 等待时间 (单位: 100ms)

**Result (结果):**
- `bool success` - 是否成功
- `string message` - 执行消息
- `int32[] final_positions` - 所有手指的最终位置 [finger1, finger2, finger3]

**Feedback (反馈):**
- `int32[] current_positions` - 当前所有手指位置
- `float32 progress` - 执行进度 (0.0-1.0)

### ResetHand.action

**Goal (目标):**
- `uint8 adapter_index` - 适配器索引 (0=左手, 1=右手)

**Result (结果):**
- `bool success` - 是否成功
- `string message` - 执行消息

**Feedback (反馈):**
- `string status` - 重置状态

## Python API 示例

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from hands_control_interfaces.action import HandControl

class MyHandController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.client = ActionClient(self, HandControl, 'hand_control')
        
    def control_finger(self):
        goal = HandControl.Goal()
        goal.adapter_index = 0  # 左手
        goal.finger_id = 1
        goal.position = 1000
        goal.speed = 600
        goal.force = 0x55
        goal.wait_time = 10
        
        self.client.wait_for_server()
        future = self.client.send_goal_async(goal)
        # 处理 future...

def main():
    rclpy.init()
    node = MyHandController()
    node.control_finger()
    rclpy.spin(node)
```

## 硬件配置

### 设备连接
- 左手：adapter_index=0 (第一个 ZLG 适配器)
- 右手：adapter_index=1 (第二个 ZLG 适配器)

### 手指 ID
- 0x01 (1): 手指 1
- 0x02 (2): 手指 2
- 0x03 (3): 手指 3

### 参数范围
- 位置: 0 (完全张开) - 1000 (完全闭合)
- 速度: 0 - 1000
- 力度: 0x00 - 0xFF (推荐 0x55)

## 故障排查

### 1. 找不到 dexhand 模块
```bash
pip3 install -e /path/to/dexhand_sdk_python
```

### 2. 设备未连接
- 检查 ZLG 适配器是否正确连接
- 确认设备 ID 配置正确
- 查看驱动是否安装 (参考 [ZLG 手册](https://manual.zlg.cn/web/#/146))

### 3. 权限问题
```bash
sudo usermod -a -G dialout $USER
# 注销重新登录
```

## 参考文档

- [DexHand SDK Python](https://github.com/DexRobot/dexhand_sdk_python)
- [ZLG 驱动文档](https://manual.zlg.cn/web/#/146)
- [DexHand 文档](https://dexrobot.feishu.cn/docx/ATs0dq9TAolpKpxXaZvcY8t7nZd)

## 许可证

Apache-2.0
