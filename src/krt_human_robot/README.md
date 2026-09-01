# krt_human_robot

整机核心入口包。这里拥有 `krtHumanRobot` 节点、行为树、黑板、意图识别、LLM 规划和动作分发。

`voice_assistant` 只作为语音子系统存在，核心包通过 `/voice/...` ROS 接口调用它，不 import 语音包内部 Python 实现。

## 包边界

- `krt_human_robot/behaviors/core`：黑板、意图、规划、对话循环。
- `krt_human_robot/behaviors/voice.py`：语音行为树适配，只调用 `/voice/...` topic/service/action。
- `krt_human_robot/adapters`：相机、雷达、底盘、导航、机械臂等 ROS 接口适配入口预留。
- `config/krt_human_robot.yaml`：整机行为树、意图、LLM、动作开关和功能包接口名。
- 夹爪动作现在通过 `hands_control` 的 ROS action 走，右手压力读数通过 ROS service 走。

依赖方向固定为：`krt_human_robot` 调用功能包 ROS 接口；功能包不反向依赖核心包。

## 启动

```bash
cd /home/nvidia/WorkSpace/KrtHumanRobot
source install/setup.bash

# 默认入口：先拉起相机栈和 voice_stack，再启动核心行为树
ros2 launch krt_human_robot robot.launch.py

# 只测核心，不启动语音栈
ros2 launch krt_human_robot robot.launch.py enable_voice_stack:=false

# 不启动手部控制栈
ros2 launch krt_human_robot robot.launch.py enable_hands_control_stack:=false

# 不启动相机栈
ros2 launch krt_human_robot robot.launch.py enable_camera_stack:=false
```

常用参数：

- `config_file`：核心配置，默认 `config/krt_human_robot.yaml`。
- `voice_config_file`：语音栈配置，默认 `voice_assistant/config/voice_assistant.yaml`。
- `enable_voice_stack`：是否一起启动语音栈。
- `enable_hands_control_stack`：是否一起启动左右手控制栈，默认开启。
- `enable_camera_stack`：是否一起启动 RealSense 和左右手 USB 相机。
- `core_start_delay_s`：相机/语音栈启动后延迟多少秒启动核心。

## Web 地图管理

```bash
source install/setup.bash
ros2 launch krt_human_robot web_console.launch.py \
  host:=0.0.0.0 map_archive_dir:=$HOME/maps
```

也可以直接使用 `ros2 run` 启动 Web 控制台或操作示教动作组：

```bash
ros2 run krt_human_robot web_app serve --host 127.0.0.1 --port 8443
ros2 run krt_human_robot web_app teach start --arm left --group 挥手
ros2 run krt_human_robot web_app teach stop --arm left --group 挥手
ros2 run krt_human_robot web_app play 挥手 --arm left --repeat 1
```

示教动作组默认按 50 Hz 记录机械臂关节；夹爪动作由独立动作编排控制。可在
`teach_action_group.launch.py` 中通过 `sample_rate_hz` 调整。

默认仅监听 `127.0.0.1`；只有在受控机器人局域网中需要远程访问时才显式传入
`host:=0.0.0.0`，并建议配置 `certfile`/`keyfile`。

导航页按地图名称登记每次 Web 建图。FAST-LIO 和 Spark + SAM 建图均保持
RViz 开启；结束建图后，`map.yaml`、`map.pgm`、`cloud.pcd` 和元数据会登记到
时间戳目录。选择表格中的地图后：

- 导航从数据库读取该地图的绝对 `map.yaml`/`cloud.pcd` 路径；
- 新点位自动绑定当前 `map_id`，点位列表和巡航只使用当前地图的数据；
- “编辑”打开本地打包的 ROS-SLAM-Map-Editor（固定上游 commit
  `646104ee80570d66ce86d51fd19fb44b31d936a0`）；
- 编辑保存会原子覆盖当前时间戳目录中的 `map.yaml`/`map.pgm`，不会修改
  `cloud.pcd`，也不会创建 revision。

编辑前必须先停止导航，建图或保存期间不能切换、编辑地图。当前不提供 Keepout
编辑。原命令行入口保持兼容：

```bash
ros2 launch ranger_nav navigation.launch.py map:=$HOME/maps/map.yaml
```

## Web 语音触发动作编排

“动作编排”页面的 Routine 编辑弹窗支持配置多个语音触发关键词和执行成功播报。
关键词可按行填写，也可使用中文或英文逗号分隔；全部留空表示关闭该 Routine 的
语音触发。保存后从下一轮语音识别开始生效，无需重启核心节点。

语音触发设置与 Routine 一起保存在 `KRT_ROBOT_DB` 指向的 SQLite 数据库中。
`robot.launch.py` 会将同一数据库路径传给核心节点、Web 控制台和编排服务；分别
启动这些进程时也必须确保 `KRT_ROBOT_DB`/`robot_db` 指向同一个文件。

旧版 `routine_keyword_actions` 仅在数据库升级后首次启动时导入一次，之后数据库
为唯一运行时来源。自定义 Routine 关键词优先于普通意图和 LLM；匹配采用包含
关系并优先最长关键词。同一个完整关键词不能绑定多个 Routine，同时命中多个
同等长度的 Routine 时不会执行动作，而是提示用户说得更明确。

## 验证

```bash
colcon build --packages-select hand_camera_driver krt_human_robot --symlink-install

ROS_LOG_DIR=/tmp/ros_log ros2 launch krt_human_robot robot.launch.py --show-args
ROS_LOG_DIR=/tmp/ros_log ros2 launch krt_human_robot robot.launch.py enable_voice_stack:=false
```

相机话题检查：

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic hz /left_gripper/image_raw
ros2 topic hz /right_gripper/image_raw
```
