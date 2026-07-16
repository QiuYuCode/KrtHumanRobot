.. _quickstart:

快速开始
========

本章按功能给出可直接复制的常用命令。除特别说明外，所有命令都在工作区根目录执行，
每个新终端都需要重新加载 ROS 2 环境。

环境准备
--------

.. code-block:: bash

   cd /home/nvidia/WorkSpace/KrtHumanRobot
   source /opt/ros/humble/setup.bash
   source install/setup.bash

如果修改了 Python Launch、配置文件或节点代码，先重新构建对应功能包：

.. code-block:: bash

   colcon build --symlink-install --packages-select <package_name>
   source install/setup.bash

整机入口
--------

``robot.launch.py`` 默认启动语音、RealSense、双手相机、双手控制、任务流程和动作组，
等待 8 秒后启动核心行为树。机械臂控制和 Web 控制台默认关闭。

.. code-block:: bash

   # 查看全部启动参数
   ros2 launch krt_human_robot robot.launch.py --show-args

   # 默认启动
   ros2 launch krt_human_robot robot.launch.py

   # 启动左右 Nero 机械臂；CAN 口按实机名称修改
   ros2 launch krt_human_robot robot.launch.py \
     enable_arm_control_stack:=true \
     left_arm_can_port:=can_left right_arm_can_port:=can_right

   # 示例：关闭相机或关闭语音
   ros2 launch krt_human_robot robot.launch.py enable_camera_stack:=false
   ros2 launch krt_human_robot robot.launch.py enable_voice_stack:=false

.. graphviz::
   :caption: 顶层 launch 包含关系
   :align: center

   digraph launch_tree {
     graph [rankdir=LR, bgcolor="transparent", nodesep=0.25, ranksep=0.5];
     node [shape=box, style="rounded,filled", fillcolor="#f5f7fa", color="#667085", fontname="sans"];
     robot [label="krt_human_robot\nrobot.launch.py", fillcolor="#dbeafe", color="#2563eb"];
     robot -> voice [label="默认开"];
     robot -> camera [label="默认开"];
     robot -> hands [label="默认开"];
     robot -> task [label="默认开"];
     robot -> groups [label="默认开"];
     robot -> arms [label="默认关"];
     voice [label="voice_assistant\nvoice_stack.launch.py"];
     camera [label="RealSense + 手部 USB 相机"];
     hands [label="hands_control\nhand_control_launch.py"];
     task [label="krt_task\nroutine_runner.launch.py"];
     groups [label="agx_action_group_runner\nteach_action_group.launch.py"];
     arms [label="左右 agx_arm_ctrl"];
   }

底盘：建图、打点、导航与巡航
------------------------------

建图
^^^^

推荐使用带回环优化的 Spark Fast-LIO + KISS-Matcher-SAM。建图时可用以下任一方式
移动底盘，并保持低速行驶、尽量回到起点形成回环：

* **键盘遥控**：另开终端运行 ``teleop_twist_keyboard``，通过 ``/cmd_vel`` 控制底盘；
* **底盘配套遥控器**：保持建图 Launch 运行，按底盘厂家操作说明开启遥控器并切换到
  遥控模式，无需运行键盘遥控节点。可通过 ``/remote_control_status`` 查看遥控状态反馈。

.. warning::

   键盘遥控与底盘配套遥控器二选一。使用实体遥控器前应确认急停有效、低速挡位已启用，
   不要同时从键盘、导航或其他节点发送运动指令。

.. code-block:: bash

   # 终端 1：回环建图
   ros2 launch ranger_nav mapping_sam.launch.py

   # 终端 2：使用键盘遥控时运行；使用配套遥控器时跳过
   ros2 run teleop_twist_keyboard teleop_twist_keyboard

   # 可选：确认底盘已收到遥控器状态反馈
   ros2 topic echo /remote_control_status

   # 保存回环优化后的 PCD；该命令在终端 3 执行
   ros2 topic pub --once /km_sam/save_dir std_msgs/msg/String \
     "{data: '$HOME/maps'}"

小场景快速验证可以使用纯 FAST-LIO：

.. code-block:: bash

   ros2 launch ranger_nav mapping.launch.py

   # 使用键盘遥控时运行；使用配套遥控器时跳过
   ros2 run teleop_twist_keyboard teleop_twist_keyboard

   # 保存到 launch 配置的 ~/maps/scans.pcd
   ros2 service call /map_save std_srvs/srv/Trigger "{}"

以上底层命令首先保存 PCD。通过整机行为树执行“保存地图”时，会继续运行
``pcd2pgm`` 和 Nav2 ``map_saver_cli``，生成时间戳归档以及最新的
``~/maps/map.yaml``、``~/maps/map.pgm``。手动转换时需使用与机器人雷达高度和滤波参数
一致的 ``pcd2pgm.yaml``，不要直接套用其他设备的参数。

手动修补地图
^^^^^^^^^^^^

生成 ``map.yaml`` 和 ``map.pgm`` 后，可以使用
`ROS SLAM Map Editor <https://gyropalm.github.io/ROS-SLAM-Map-Editor/editor.html>`_
修补误识别的墙体、障碍和未扫描区域。编辑器源码位于
`GyroPalm/ROS-SLAM-Map-Editor <https://github.com/GyroPalm/ROS-SLAM-Map-Editor>`_。

先备份原始地图：

.. code-block:: bash

   mkdir -p $HOME/maps/backup
   cp $HOME/maps/map.yaml $HOME/maps/map.pgm $HOME/maps/backup/

如果地图不适合在在线页面中打开，可以使用 Python 标准库在本机运行编辑器：

.. code-block:: bash

   cd /tmp
   git clone --depth 1 https://github.com/GyroPalm/ROS-SLAM-Map-Editor.git
   cd ROS-SLAM-Map-Editor
   python3 -m http.server 8000

然后在浏览器打开 ``http://localhost:8000/editor.html``。修补流程如下：

1. 同时载入 ``$HOME/maps/map.yaml`` 和 ``$HOME/maps/map.pgm``；
2. 使用 **Wall** 补画占用区域，使用 **Erase** 清除错误障碍，使用 **Un-Scan** 将
   区域恢复为未知状态；
3. 可配合 Freehand、Line、Rectangle、画笔大小以及 Undo/Redo 完成修补；
4. 点击 **Download Map**，保存编辑器导出的 ``map_edited.yaml`` 和
   ``map_edited.pgm``；
5. 将两个文件放入同一目录，并确认 YAML 的 ``image`` 字段指向编辑后的 PGM。

.. code-block:: bash

   cp $HOME/Downloads/map_edited.yaml $HOME/maps/
   cp $HOME/Downloads/map_edited.pgm $HOME/maps/
   grep '^image:' $HOME/maps/map_edited.yaml

   # 使用修补后的地图启动 2D 导航
   ros2 launch ranger_nav navigation.launch.py \
     map:=$HOME/maps/map_edited.yaml

.. note::

   **Download Keepout Mask** 导出的禁行区文件不会被当前 ``ranger_nav`` 自动加载，
   需要另行配置 Nav2 Costmap Filter 后才能生效。手动修补只改变 2D 栅格地图，不会
   修改 3D 定位使用的 PCD 点云。

导航
^^^^

2D AMCL 使用栅格地图。启动后先在 RViz 中用 **2D Pose Estimate** 设置初始位姿，
再用 **Nav2 Goal** 发送目标点。

.. code-block:: bash

   ros2 launch ranger_nav navigation.launch.py \
     map:=$HOME/maps/map.yaml

3D 定位同时需要栅格地图和 PCD 地图，不要与 2D AMCL 导航同时启动：

.. code-block:: bash

   ros2 launch ranger_nav navigation_3dloc.launch.py \
     map:=$HOME/maps/map.yaml \
     pcd_map_path:=$HOME/maps/scans.pcd

   # 可选：从已知位姿初始化 3D 定位
   ros2 launch ranger_nav navigation_3dloc.launch.py \
     map:=$HOME/maps/map.yaml \
     pcd_map_path:=$HOME/maps/scans.pcd \
     set_initial_pose:=true \
     initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0

打点与巡航
^^^^^^^^^^

打点依赖可用的 ``map → base_footprint`` TF。机器人到达目标位置后保存当前位姿：

.. code-block:: bash

   # 保存、查看和删除点位
   ros2 run ranger_nav waypoint_manager mark wp_001
   ros2 run ranger_nav waypoint_manager mark wp_002
   ros2 run ranger_nav waypoint_manager list
   ros2 run ranger_nav waypoint_manager remove wp_002

   # 将点位绑定到达后执行的任务；未绑定时仅导航
   ros2 run ranger_nav waypoint_manager bind wp_001 --routine welcome

   # 按给定顺序巡航一次、重复三次或持续循环
   ros2 run ranger_nav waypoint_manager cruise wp_001 wp_002
   ros2 run ranger_nav waypoint_manager cruise wp_001 wp_002 --repeat 3
   ros2 run ranger_nav waypoint_manager cruise wp_001 wp_002 --loop

   # 清空全部点位
   ros2 run ranger_nav waypoint_manager clear

默认点位数据库是 ``~/maps/krt_robot.db``。巡航需要 ``/navigate_to_pose`` Action；
点位绑定任务后，还需要 ``/krt_task/run_routine`` Action。

.. code-block:: bash

   ros2 run ranger_nav nav_tf_diagnostics
   ros2 action info /navigate_to_pose
   ros2 topic hz /scan
   ros2 topic info -v /cmd_vel

语音：KWS、VAD、ASR 与 TTS
---------------------------

术语说明：KWS 是关键词唤醒，VAD 是语音活动检测，ASR 是语音转文字，TTS 是文字
转语音。语音模型和虚拟环境应先按安装章节准备完成。

.. code-block:: bash

   # 启动完整语音栈
   ros2 launch voice_assistant voice_assistant.launch.py

   # 查看可按需关闭的模块参数
   ros2 launch voice_assistant voice_assistant.launch.py --show-args

各模块可以独立启动并在另一个终端运行测试客户端：

.. code-block:: bash

   # KWS：说出配置文件中的唤醒词
   ros2 launch voice_assistant kws_test.launch.py
   ros2 run voice_test_tools kws_test_client

   # VAD：说一句话后保持安静
   ros2 launch voice_assistant vad_test.launch.py
   ros2 run voice_test_tools vad_test_client

   # ASR：启动后说一句话，客户端输出识别文字
   ros2 launch voice_assistant asr_test.launch.py
   ros2 run voice_test_tools asr_test_client

   # TTS：合成并从扬声器播放指定文字
   ros2 launch voice_assistant tts_test.launch.py
   ros2 run voice_test_tools tts_test_client --ros-args \
     -p text:="你好，这是语音合成测试"

也可以直接观察事件或调用接口：

.. code-block:: bash

   ros2 topic echo /voice/kws/events
   ros2 topic echo /voice/vad/events
   ros2 action info /voice/asr/stream
   ros2 service call /voice/tts/synthesize \
     voice_interfaces/srv/SynthesizeSpeech \
     "{text: '你好', language: 'zh-CN', style: '', priority: 0}"

头部 RealSense 相机
--------------------

以下命令发布彩色、深度和对齐深度图像。需要固定分辨率时，按设备支持的 profile
修改参数。

.. code-block:: bash

   ros2 launch realsense2_camera rs_launch.py \
     enable_color:=true \
     enable_depth:=true \
     align_depth.enable:=true \
     enable_sync:=true \
     rgb_camera.color_profile:=640,480,30 \
     depth_module.depth_profile:=640,480,30

   # 检查数据频率和编码
   ros2 topic hz /camera/camera/color/image_raw
   ros2 topic info -v /camera/camera/color/image_raw
   ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw

   # 打开图像查看器，在界面中选择 Topic；磁盘按钮可保存当前帧
   ros2 run rqt_image_view rqt_image_view

   # 录制和回放 ROS 图像流
   ros2 bag record -o head_camera \
     /camera/camera/color/image_raw \
     /camera/camera/color/camera_info
   ros2 bag play head_camera

这里的“视频流”是 ROS 2 ``sensor_msgs/Image`` Topic。``ros2 bag`` 保存的是 ROS bag，
不是 MP4，也不提供 RTSP/WebRTC 网络推流。

左右手 USB 相机
----------------

默认设备符号链接为 ``/dev/camera_left`` 和 ``/dev/camera_right``：

.. code-block:: bash

   ros2 launch hand_camera_driver hand_cameras.launch.py \
     left_device:=/dev/camera_left \
     right_device:=/dev/camera_right \
     width:=640 height:=480 fps:=30.0

   ros2 topic hz /left_gripper/image_raw
   ros2 topic hz /right_gripper/image_raw

   # 查看并通过界面磁盘按钮保存当前帧
   ros2 run rqt_image_view rqt_image_view

   # 同时录制左右手图像流
   ros2 bag record -o hand_cameras \
     /left_gripper/image_raw \
     /right_gripper/image_raw
   ros2 bag play hand_cameras

如果设备名不存在，先检查实际视频设备和权限：

.. code-block:: bash

   ls -l /dev/camera_left /dev/camera_right /dev/video*
   groups

三指灵巧手控制
--------------

.. warning::

   以下命令会驱动 DexHand021S 真机。首次测试应确认手指周围没有人员或物体，并从
   较低速度开始。运行 ``hand_control_client`` 会直接执行内置示例动作。

启动控制节点
^^^^^^^^^^^^

默认启动左右手两个 Action Server，设备 ID 分别为 1 和 2。底层监听与自动反馈默认
关闭，需要读取实时传感器时再开启。

.. code-block:: bash

   # 使用默认参数启动
   ros2 launch hands_control hand_control_launch.py

   # 显式指定全部常用参数
   ros2 launch hands_control hand_control_launch.py \
     adapter_type:=ZLG_MINI \
     left_hand_device_id:=1 \
     right_hand_device_id:=2 \
     left_hand_listen:=false \
     right_hand_listen:=false \
     left_hand_realtime_response:=false \
     right_hand_realtime_response:=false

   # 可选：运行功能包自带的动作测试客户端
   ros2 run hands_control hand_control_client

``HandControl`` Action 参数如下：

.. list-table::
   :header-rows: 1
   :widths: 22 28 50

   * - 字段
     - 范围
     - 说明
   * - ``adapter_index``
     - ``0`` 或 ``1``
     - ``0`` 为左手，``1`` 为右手，必须与 Action 命名空间一致
   * - ``finger_id``
     - ``0～3``
     - ``1～3`` 为单指，``0`` 表示三根手指同时执行
   * - ``position``
     - ``0～1000``
     - ``0`` 为完全张开，``1000`` 为完全闭合
   * - ``speed``
     - ``0～1000``
     - 手指运动速度
   * - ``force``
     - ``0～255``
     - 手指力度，README 推荐 ``85``（十六进制 ``0x55``）
   * - ``wait_time``
     - ``0～255``
     - 等待时间，单位为 100 ms；例如 ``10`` 表示 1 秒

动作控制
^^^^^^^^

单指运动：

.. code-block:: bash

   # 左手第 1 指移动到位置 800
   ros2 action send_goal /left/hand_control \
     hands_control_interfaces/action/HandControl \
     "{adapter_index: 0, finger_id: 1, position: 800, speed: 500, force: 85, wait_time: 10}" \
     --feedback

   # 右手第 1 指移动到位置 800
   ros2 action send_goal /right/hand_control \
     hands_control_interfaces/action/HandControl \
     "{adapter_index: 1, finger_id: 1, position: 800, speed: 500, force: 85, wait_time: 10}" \
     --feedback

三指同时运动时使用 ``finger_id: 0``：

.. code-block:: bash

   # 左手全部闭合
   ros2 action send_goal /left/hand_control \
     hands_control_interfaces/action/HandControl \
     "{adapter_index: 0, finger_id: 0, position: 1000, speed: 500, force: 85, wait_time: 10}" \
     --feedback

   # 右手全部闭合
   ros2 action send_goal /right/hand_control \
     hands_control_interfaces/action/HandControl \
     "{adapter_index: 1, finger_id: 0, position: 1000, speed: 500, force: 85, wait_time: 10}" \
     --feedback

   # 右手全部张开
   ros2 action send_goal /right/hand_control \
     hands_control_interfaces/action/HandControl \
     "{adapter_index: 1, finger_id: 0, position: 0, speed: 500, force: 85, wait_time: 10}" \
     --feedback

复位关节：

.. code-block:: bash

   ros2 action send_goal /left/reset_hand \
     hands_control_interfaces/action/ResetHand "{adapter_index: 0}" --feedback
   ros2 action send_goal /right/reset_hand \
     hands_control_interfaces/action/ResetHand "{adapter_index: 1}" --feedback

监听、状态与传感器
^^^^^^^^^^^^^^^^^^

监听和自动反馈可以在运行时独立开关：

.. code-block:: bash

   ros2 param set /left/hand_control_server listen_enabled true
   ros2 param set /left/hand_control_server realtime_response_enabled true
   ros2 param get /left/hand_control_server listen_enabled
   ros2 param get /left/hand_control_server realtime_response_enabled

   ros2 param set /right/hand_control_server listen_enabled true
   ros2 param set /right/hand_control_server realtime_response_enabled true
   ros2 param get /right/hand_control_server listen_enabled
   ros2 param get /right/hand_control_server realtime_response_enabled

常用设备状态与清错服务：

.. code-block:: bash

   ros2 service call /left/get_device_id \
     hands_control_interfaces/srv/GetDeviceId "{channel: 0}"
   ros2 service call /right/get_device_id \
     hands_control_interfaces/srv/GetDeviceId "{channel: 0}"
   ros2 service call /left/get_firmware_version \
     hands_control_interfaces/srv/GetDeviceString "{}"
   ros2 service call /right/get_firmware_version \
     hands_control_interfaces/srv/GetDeviceString "{}"
   ros2 service call /left/clear_error std_srvs/srv/Trigger "{}"
   ros2 service call /right/clear_error std_srvs/srv/Trigger "{}"

右手带压力和接近觉传感器。读取前应打开右手监听和自动反馈：

.. code-block:: bash

   ros2 param set /right/hand_control_server listen_enabled true
   ros2 param set /right/hand_control_server realtime_response_enabled true
   ros2 service call /right/get_normal_pressure \
     hands_control_interfaces/srv/GetNormalPressure "{finger_id: 1}"
   ros2 service call /right/get_tangent_pressure \
     hands_control_interfaces/srv/GetTangentPressure "{finger_id: 1}"
   ros2 service call /right/get_approaching_value \
     hands_control_interfaces/srv/GetApproachingValue "{finger_id: 1}"

左手没有压力和接近觉传感器，因此不会提供对应的三个 ``/left/get_*`` 服务。

接口检查
^^^^^^^^

.. code-block:: bash

   ros2 interface show hands_control_interfaces/action/HandControl
   ros2 interface show hands_control_interfaces/action/ResetHand
   ros2 action info /left/hand_control
   ros2 action info /right/hand_control
   ros2 action info /left/reset_hand
   ros2 action info /right/reset_hand
   ros2 node info /left/hand_control_server
   ros2 node info /right/hand_control_server

机械臂基本控制与动作组
----------------------

.. warning::

   机械臂启动、回零和动作组回放都会驱动真机运动。先确认急停可用、CAN 接口正确、
   工作空间内无人，并从较低的 ``speed_percent`` 开始测试。

启动与基本控制
^^^^^^^^^^^^^^

先确认 CAN 接口存在，然后分别启动左右 Nero 机械臂：

.. code-block:: bash

   ip -br link show can_left
   ip -br link show can_right

   # 终端 1：左臂
   ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
     namespace:=left can_port:=can_left arm_type:=nero \
     auto_enable:=true speed_percent:=30

   # 终端 2：右臂
   ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
     namespace:=right can_port:=can_right arm_type:=nero \
     auto_enable:=true speed_percent:=30

基本服务和反馈接口：

.. code-block:: bash

   # 使能、失能和回零；将 left 换成 right 可控制右臂
   ros2 service call /left/enable_agx_arm std_srvs/srv/SetBool "{data: true}"
   ros2 service call /left/enable_agx_arm std_srvs/srv/SetBool "{data: false}"
   ros2 service call /left/move_home std_srvs/srv/Empty "{}"

   # 急停：保持当前位置
   ros2 service call /left/emergency_stop std_srvs/srv/Empty "{}"

   # 查看关节和机械臂状态
   ros2 topic echo /left/feedback/joint_states --once
   ros2 topic echo /left/feedback/arm_status --once

需要 MoveIt 规划和 RViz 交互控制时，可一键启动单臂驱动与 MoveIt：

.. code-block:: bash

   ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
     namespace:=left can_port:=can_left arm_type:=nero \
     follow:=true auto_control_gate:=true speed_percent:=30

录制动作组
^^^^^^^^^^

保持左右机械臂控制节点运行，再启动动作组录制与回放服务。当前实现把动作组保存到
SQLite 数据库 ``~/maps/krt_robot.db``；旧版 ``groups_file`` 参数已经不再使用。

.. code-block:: bash

   # 终端 3
   ros2 launch agx_action_group_runner teach_action_group.launch.py \
     robot_db:=$HOME/maps/krt_robot.db \
     left_namespace:=/left \
     right_namespace:=/right \
     stream_step_interval_sec:=0.02

一次只能录制一条机械臂。开始录制后手动拖动机械臂，完成后停止并保存：

.. code-block:: bash

   # 开始录制左臂“挥手”动作组
   ros2 service call /agx_action_group/start_teach \
     agx_action_group_interfaces/srv/StartTeach \
     "{arm_target: left, group_name: '挥手'}"

   # 拖动左臂完成动作后，停止并保存
   ros2 service call /agx_action_group/stop_teach \
     agx_action_group_interfaces/srv/StopTeach \
     "{arm_target: left, group_name: '挥手'}"

   # 回放一次或重复三次
   ros2 action send_goal /agx_action_group/run_action_group \
     agx_action_group_interfaces/action/RunActionGroup \
     "{group_name: '挥手', repeat_count: 1, arm_target: left}" --feedback

   ros2 action send_goal /agx_action_group/run_action_group \
     agx_action_group_interfaces/action/RunActionGroup \
     "{group_name: '挥手', repeat_count: 3, arm_target: left}" --feedback

右臂录制时把 ``arm_target`` 改成 ``right``。录制和回放使用同一个 ``robot_db``，
否则回放节点无法找到刚保存的动作组。

.. code-block:: bash

   ros2 service list | grep -E 'start_teach|stop_teach|set_teach_mode'
   ros2 action info /agx_action_group/run_action_group
   ros2 topic hz /left/feedback/leader_joint_states

运行验证与停止
--------------

.. code-block:: bash

   ros2 node list
   ros2 topic list -t
   ros2 service list -t
   ros2 action list -t

Topic 存在但没有数据时，使用 ``ros2 topic info -v`` 检查发布者、订阅者和 QoS，
不要只根据名称判断模块已经正常工作。前台 Launch 使用 ``Ctrl+C`` 正常停止；涉及底盘、
机械臂或灵巧手时，应先停止运动，再关闭驱动节点。
