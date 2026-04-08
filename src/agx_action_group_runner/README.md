# agx_action_group_runner

用于按动作组执行 AGX 机械臂动作，并支持“示教录制后自动复现一次”。

## 功能

- Action 执行动作组：`/run_action_group`
- 支持目标臂选择：`left` / `right` / `both`
- 录制节点监听示教状态，自动采样 `/feedback/joint_states`
- 结束示教后自动生成动作组 YAML，并调用 Action 回放一次

## 前提

- 已启动左右臂控制节点，推荐命名空间：
  - 左臂：`/left_arm`
  - 右臂：`/right_arm`
- `agx_action_group_runner` 与 `agx_action_group_interfaces` 已编译

## 启动动作组执行器

```bash
ros2 launch agx_action_group_runner run_action_group_runner.launch.py \
  groups_file:=/home/nvidia/WorkSpace/KrtHumanRobot/src/agx_action_group_runner/config/action_groups.yaml \
  left_namespace:=/left_arm \
  right_namespace:=/right_arm
```

## 启动示教录制-自动复现节点

```bash
ros2 launch agx_action_group_runner teach_record_replay.launch.py \
  arm_namespace:=/left_arm \
  auto_replay_once:=true \
  sample_rate_hz:=20.0 \
  min_joint_delta_rad:=0.01
```

## 测试流程（左臂）

1. 启动左右臂控制节点与 `/run_action_group` 执行器。
2. 启动 `teach_record_replay` 节点，监听 `/left_arm/feedback/*`。
3. 将左臂进入示教模式（当前默认通过人工/硬件侧触发）。
4. 拖动左臂完成一段动作。
5. 退出示教模式（可用 `/left_arm/exit_teach_mode`，若设备型号支持）。
6. 节点自动：
   - 停止录制
   - 生成动作组文件
   - 调用 `/run_action_group` 复现 1 次

## 录制文件输出

默认输出目录：`<当前工作目录>/teach_records`

每次录制生成：

- `teach_replay_YYYYmmdd_HHMMSS/action_groups.yaml`
- `teach_replay_YYYYmmdd_HHMMSS/steps/step_0001.yaml` ... `step_NNNN.yaml`
- `latest_action_groups.yaml`（最新一次覆盖）

## 直接手动触发复现（可选）

如果你不想自动回放，可以设置 `auto_replay_once:=false`，然后手动调用：

```bash
ros2 action send_goal /run_action_group \
  agx_action_group_interfaces/action/RunActionGroup \
  "{group_name: teach_replay_YYYYmmdd_HHMMSS, repeat_count: 1, arm_target: left}" --feedback
```

`group_name` 以生成文件中的组名为准。

## 限制说明

- 当前仓库未提供稳定 ROS 接口读取“固件离线示教轨迹点”。
- 参数 `prefer_offline_teach:=true` 目前会记录告警并回退到关节采样方案。
- 录制采样是离散关键帧，不是时间连续轨迹；过低采样率会导致复现不平滑。
