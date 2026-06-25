# Nav2 Waypoint 打点、巡航与到点任务计划

## Summary

- 在 `ranger_nav` 实现 waypoint 数据管理、巡航执行、到点 task 分发。
- 在 `krt_human_robot` 只接语音/行为树意图：打点、巡航、继续、删除点位、列出点位等，然后调用 `ranger_nav` 能力。
- 不让 waypoint task 反向走“语音 -> 意图识别 -> 行为树”整条链；到点任务直接调用底层 ROS 接口或轻量函数。
- Nav2 `waypoint_follower` 原生是按收到的 waypoint 列表跑一遍，跑完 action 结束；指定次数和循环巡航由 `ranger_nav` 自己实现。

## Key Changes

- `ranger_nav` 新增 `waypoint_manager`：
  - `mark [name] [--task TASK] [--args JSON]`：读取当前 `map -> base_footprint` TF 保存点位。
  - `list`：列出点位。
  - `remove <name>` / `clear`：删除点位。
  - `cruise [names...]`：按保存顺序或指定名称巡航。
  - `cruise --repeat N [names...]`：指定巡航次数。
  - `cruise --loop [names...]`：无限循环巡航，直到取消。
  - `continue_input`：发布 input 继续信号。
- 点位保存到 `~/maps/waypoints.yaml`：
  - 未命名自动生成 `wp_001`、`wp_002`。
  - 指定名称必须唯一，重名直接失败。
  - 每个点支持 `task + args`，先实现 `wait/photo/input/speak/describe/arm_group`。
- `ranger_nav` 到点 task 分发：
  - `wait`：等待 `wait_ms`。
  - `photo`：订阅相机图像并保存。
  - `input`：等待 `/input_at_waypoint/input`。
  - `speak`：调用 `/voice/tts/synthesize`。
  - `describe`：调用 `/krt_human_robot/vision/describe_scene` 后播报描述。
  - `arm_group`：调用现有机械臂动作组接口。
  - 未知 task 直接失败并提示，不静默跳过。
- `krt_human_robot` 扩展：
  - 新增 intent：打点、开始巡航、巡航指定点、循环巡航、巡航 N 遍、停止巡航、继续、删除点位、列出点位。
  - `NavigationAction` 调用 `RangerNavAdapter` 的 waypoint 方法。
  - 不保存 waypoint，不管理巡航循环，不实现 task 细节。
- `nav2_params.yaml` 保留并补全 Nav2 内置插件配置：
  - `wait_at_waypoint`
  - `photo_at_waypoint`
  - `input_at_waypoint`
  - 用于单一插件 `/follow_waypoints` 场景；混合任务用 Python 顺序发 `/navigate_to_pose`。

## Public Interfaces

- Console script：
  - `ros2 run ranger_nav waypoint_manager mark`
  - `ros2 run ranger_nav waypoint_manager mark 前台 --task photo`
  - `ros2 run ranger_nav waypoint_manager mark 展示区 --task speak --args '{"text":"这里是展示区"}'`
  - `ros2 run ranger_nav waypoint_manager mark 展台 --task describe --args '{"camera_id":"head","question":"请描述这个巡航点看到的内容"}'`
  - `ros2 run ranger_nav waypoint_manager cruise`
  - `ros2 run ranger_nav waypoint_manager cruise --repeat 3 前台 展示区`
  - `ros2 run ranger_nav waypoint_manager cruise --loop`
  - `ros2 run ranger_nav waypoint_manager continue_input`
- ROS interfaces：
  - `/navigate_to_pose`：`nav2_msgs/action/NavigateToPose`
  - `/follow_waypoints`：`nav2_msgs/action/FollowWaypoints`
  - `/input_at_waypoint/input`：`std_msgs/msg/Empty`
  - `/voice/tts/synthesize`：用于 `speak`
  - `/krt_human_robot/vision/describe_scene`：用于 `describe`
  - 现有机械臂动作组接口：用于 `arm_group`
- Config defaults under `adapters.navigation`：
  - `waypoints_file: ~/maps/waypoints.yaml`
  - `waypoint_input_topic: /input_at_waypoint/input`
  - `waypoint_image_topic: /camera/camera/color/image_raw`
  - `waypoint_image_dir: ~/maps/waypoint_images`
  - `default_waypoint_wait_ms: 200`
  - `waypoint_vision_service: /krt_human_robot/vision/describe_scene`

## Cruise Behavior

- 默认 `cruise` 只跑一遍，全部点完成后退出。
- `--repeat N` 跑指定次数，`N` 必须是正整数。
- `--loop` 无限循环，直到用户语音“停止巡航”或 CLI 取消。
- 巡航停止时取消当前 `/navigate_to_pose` goal，并退出后续循环。
- 单个点失败时默认停止本轮巡航；后续如果需要再加 `--continue-on-failure`。

## Test Plan

- Build：
  - `colcon build --packages-select ranger_nav krt_human_robot --symlink-install`
- Import check：
  - `source install/setup.bash && cd src/voice_assistant && UV_CACHE_DIR=/tmp/uv-cache uv run python -c "import krt_human_robot.robot_node; import krt_human_robot.tree_factory; import krt_human_robot.config; print('krt imports ok')"`
- Smoke test：
  - 启动导航 launch。
  - 打两个点，确认 YAML 名称唯一且自动递增。
  - 测 `list`、`cruise`、`cruise <name>`。
  - 测 `cruise --repeat 2` 和 `cruise --loop` 后停止巡航。
  - 测 `wait/photo/input/speak/arm_group` 各一个点位。
  - 语音触发“打点、开始巡航、循环巡航、巡航三遍、停止巡航、继续、去某点”。

## Assumptions

- ROS 2 distro 是 Humble。
- 当前位姿固定读取 `map -> base_footprint`。
- waypoint task 是路线执行逻辑，不复用完整行为树意图流程。
- 先不做 rename；需要改名就删除后重新打点。
- 不写 C++ 自定义 Nav2 WaypointTaskExecutor 插件，除非后续确认必须把每点 task 放进 Nav2 原生插件体系。
