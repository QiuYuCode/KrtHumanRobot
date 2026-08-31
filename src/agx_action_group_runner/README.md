# agx_action_group_runner

Nero 双臂示教录制与动作组回放。

## 功能

- 进入/退出 Nero 示教模式：`/left/set_teach_mode`、`/right/set_teach_mode`
- 保存命名动作组：`/agx_action_group/start_teach`、`/agx_action_group/stop_teach`
- 执行动作组：`/agx_action_group/run_action_group`
- 示教默认按 50 Hz 同时记录机械臂和同侧 `hands_control` 三指；内置
  `agx_gripper` / `Revo2` 关节随 `leader_joint_states` 一起记录
- 动作组数据运行时生成，默认位置：
  - `config/action_groups.yaml`
  - `config/steps/`

`action_groups.yaml` 和 `steps/` 是运行时录制数据，已被 git 忽略。

## 启动流程

### 终端 1：左臂控制节点

```bash
cd /home/nvidia/WorkSpace/KrtHumanRobot
source install/setup.bash

ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  namespace:=left \
  can_port:=can_left \
  arm_type:=nero \
  auto_enable:=true \
  speed_percent:=30
```

### 终端 2：右臂控制节点

```bash
cd /home/nvidia/WorkSpace/KrtHumanRobot
source install/setup.bash

ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  namespace:=right \
  can_port:=can_right \
  arm_type:=nero \
  auto_enable:=true \
  speed_percent:=30
```

### 终端 3：示教录制 + 动作组执行服务

```bash
cd /home/nvidia/WorkSpace/KrtHumanRobot
source install/setup.bash

ros2 launch agx_action_group_runner teach_action_group.launch.py \
  groups_file:=/home/nvidia/WorkSpace/KrtHumanRobot/src/agx_action_group_runner/config/action_groups.yaml \
  left_namespace:=/left \
  right_namespace:=/right
```

如果回放太慢或太快，可调流式发送间隔：

```bash
ros2 launch agx_action_group_runner teach_action_group.launch.py \
  groups_file:=/home/nvidia/WorkSpace/KrtHumanRobot/src/agx_action_group_runner/config/action_groups.yaml \
  left_namespace:=/left \
  right_namespace:=/right \
  stream_step_interval_sec:=0.01
```

示教采样频率可单独设置：

```bash
ros2 launch agx_action_group_runner teach_action_group.launch.py \
  sample_rate_hz:=50.0
```

回放时会自动识别持续静止的关节采样。机械臂先到达停顿姿态，再完整保持
录制的停顿时间；停顿期间夹爪仍按原采样频率同步回放。默认识别持续 0.5 秒、
关节范围不超过 0.002 rad 的静止段，可按需调整：

```bash
ros2 launch agx_action_group_runner teach_action_group.launch.py \
  pause_min_duration_sec:=0.5 \
  pause_joint_range_rad:=0.002 \
  pause_reach_tolerance_rad:=0.02
```

## 完整流程测试

### 确认服务和 action

```bash
source /home/nvidia/WorkSpace/KrtHumanRobot/install/setup.bash

ros2 service list | grep teach
ros2 service list | grep set_teach_mode
ros2 action list | grep run_action_group
```

### 左臂录制并回放

进入左臂示教：

```bash
ros2 service call /agx_action_group/start_teach agx_action_group_interfaces/srv/StartTeach "{arm_target: left, group_name: 挥手}"
```

手动拖动左臂做动作。

保存并退出示教：

```bash
ros2 service call /agx_action_group/stop_teach agx_action_group_interfaces/srv/StopTeach "{arm_target: left, group_name: 挥手}"
```

执行刚保存的动作组：

```bash
ros2 action send_goal /agx_action_group/run_action_group agx_action_group_interfaces/action/RunActionGroup "{group_name: 挥手, repeat_count: 1, arm_target: left}" --feedback
```

### 右臂录制并回放

```bash
ros2 service call /agx_action_group/start_teach agx_action_group_interfaces/srv/StartTeach "{arm_target: right, group_name: 敬礼}"
```

手动拖动右臂做动作。

```bash
ros2 service call /agx_action_group/stop_teach agx_action_group_interfaces/srv/StopTeach "{arm_target: right, group_name: 敬礼}"
```

```bash
ros2 action send_goal /agx_action_group/run_action_group agx_action_group_interfaces/action/RunActionGroup "{group_name: 敬礼, repeat_count: 1, arm_target: right}" --feedback
```

## 单独测试 Nero 示教模式接口

左臂：

```bash
ros2 service call /left/set_teach_mode std_srvs/srv/SetBool "{data: true}"
ros2 service call /left/set_teach_mode std_srvs/srv/SetBool "{data: false}"
```

右臂：

```bash
ros2 service call /right/set_teach_mode std_srvs/srv/SetBool "{data: true}"
ros2 service call /right/set_teach_mode std_srvs/srv/SetBool "{data: false}"
```

Nero 固件 1.12 以后，退出示教使用 follower mode，不使用 `set_normal_mode()`。

## 清空已录制动作组

先停掉 `agx_action_group_runner` / `teach_action_group`，再删除：

```bash
cd /home/nvidia/WorkSpace/KrtHumanRobot

rm -f src/agx_action_group_runner/config/action_groups.yaml
rm -rf src/agx_action_group_runner/config/steps
```

下次启动 `teach_action_group` 时会自动创建空的 `action_groups.yaml`。
