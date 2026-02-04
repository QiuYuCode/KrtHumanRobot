# hands_control 快速入门

## 快速开始

### 1. 安装 DexHand SDK

```bash
cd /tmp
git clone https://github.com/DexRobot/dexhand_sdk_python.git
cd dexhand_sdk_python
pip3 install -e .
```

### 2. 构建功能包

```bash
cd /home/create/DataDisk/WorkSpace/CrtWorkSpace/KrtHumanRobot

# 构建
colcon build --packages-select hands_control_interfaces hands_control --symlink-install

# 加载环境
source install/setup.bash
```

### 3. 启动服务器

```bash
ros2 launch hands_control hand_control_launch.py
```

### 4. 测试控制

**终端 1 - 运行测试客户端:**
```bash
source install/setup.bash
ros2 run hands_control hand_control_client
```

**终端 2 - 命令行测试:**
```bash
source install/setup.bash

# 左手手指 1 移动
ros2 action send_goal /left/hand_control hands_control_interfaces/action/HandControl \
    "{adapter_index: 0, finger_id: 1, position: 1000, speed: 600, force: 85, wait_time: 10}" \
    --feedback

# 左手所有手指闭合
ros2 action send_goal /left/hand_control hands_control_interfaces/action/HandControl \
    "{adapter_index: 0, finger_id: 0, position: 1000, speed: 600, force: 85, wait_time: 10}" \
    --feedback

# 右手所有手指张开
ros2 action send_goal /right/hand_control hands_control_interfaces/action/HandControl \
    "{adapter_index: 1, finger_id: 0, position: 0, speed: 600, force: 85, wait_time: 10}" \
    --feedback

# 右手手指 2 移动
ros2 action send_goal /right/hand_control hands_control_interfaces/action/HandControl \
    "{adapter_index: 1, finger_id: 2, position: 800, speed: 500, force: 85, wait_time: 10}" \
    --feedback

# 重置左手
ros2 action send_goal /left/reset_hand hands_control_interfaces/action/ResetHand \
    "{adapter_index: 0}" --feedback
```

## 常用命令

### 查看 Action 接口

```bash
# 查看 HandControl action 定义
ros2 interface show hands_control_interfaces/action/HandControl

# 查看 ResetHand action 定义
ros2 interface show hands_control_interfaces/action/ResetHand
```

### 查看 Action 状态

```bash
# 列出所有 action servers
ros2 action list

# 查看 action 详细信息
ros2 action info /hand_control
ros2 action info /reset_hand
```

### 监控话题

```bash
# 查看节点
ros2 node list

# 查看节点信息
ros2 node info /hand_control_server
```

## 参数配置

### 修改设备 ID

编辑 `launch/hand_control_launch.py`:

```python
left_device_id_arg = DeclareLaunchArgument(
    'left_hand_device_id',
    default_value='1',  # 改为你的左手设备 ID
    description='Left hand device ID'
)

right_device_id_arg = DeclareLaunchArgument(
    'right_hand_device_id',
    default_value='2',  # 改为你的右手设备 ID
    description='Right hand device ID'
)
```

或启动时传参:

```bash
ros2 launch hands_control hand_control_launch.py \
    adapter_type:=ZLG_MINI \
    left_hand_device_id:=1 \
    right_hand_device_id:=2
```

## 编写自定义控制器

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from hands_control_interfaces.action import HandControl, ResetHand


class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        
        # 创建 action clients
        self.hand_client = ActionClient(self, HandControl, 'hand_control')
        self.reset_client = ActionClient(self, ResetHand, 'reset_hand')
    
    def grab_left_hand(self):
        """左手抓取动作."""
        goal = HandControl.Goal()
        goal.adapter_index = 0  # 左手
        
        # 三个手指同时闭合
        for finger_id in [1, 2, 3]:
            goal.finger_id = finger_id
            goal.position = 1000  # 完全闭合
            goal.speed = 600
            goal.force = 0x55
            goal.wait_time = 10
            
            self.hand_client.wait_for_server()
            future = self.hand_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
    
    def release_right_hand(self):
        """右手释放动作."""
        goal = HandControl.Goal()
        goal.adapter_index = 1  # 右手
        
        # 三个手指同时张开
        for finger_id in [1, 2, 3]:
            goal.finger_id = finger_id
            goal.position = 0  # 完全张开
            goal.speed = 600
            goal.force = 0x55
            goal.wait_time = 10
            
            self.hand_client.wait_for_server()
            future = self.hand_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)


def main():
    rclpy.init()
    controller = MyController()
    
    # 执行动作
    controller.grab_left_hand()
    controller.release_right_hand()
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 故障排查

### 问题 1: ImportError: No module named 'dexhand'

**解决:**
```bash
pip3 install -e /path/to/dexhand_sdk_python
```

### 问题 2: 设备初始化失败

**检查项:**
1. ZLG 适配器是否连接
2. 设备 ID 是否正确
3. 驱动是否安装

```bash
# 查看日志
ros2 launch hands_control hand_control_launch.py

# 应该看到:
# [INFO] [hand_control_server]: 左手初始化成功 (adapter_index=0, device_id=1)
# [INFO] [hand_control_server]: 右手初始化成功 (adapter_index=1, device_id=2)
```

### 问题 3: Action 调用无响应

**检查 server 是否运行:**
```bash
ros2 action list
# 应该看到:
# /hand_control
# /reset_hand
```

**检查节点状态:**
```bash
ros2 node list
# 应该看到:
# /hand_control_server
```

### 问题 4: 权限错误

```bash
sudo usermod -a -G dialout $USER
# 注销并重新登录
```

## 下一步

- 阅读完整文档: `README.md`
- 查看 DexHand SDK: https://github.com/DexRobot/dexhand_sdk_python
- 查看示例代码: `hands_control/hand_control_client.py`
