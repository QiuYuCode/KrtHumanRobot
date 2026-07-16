.. _about:

关于 KrtHumanRobot
==================

项目简介
--------

KrtHumanRobot 是一个基于 ROS 2 Humble 的人形机器人集成工作区。核心包
``krt_human_robot`` 使用行为树组织唤醒、语音理解、视觉、导航建图、机械臂、
灵巧手和任务流程，并通过 ROS topic、service 和 action 调用各功能包。

项目结构
--------

下面按功能展示工作区的主要目录。聚合仓库展开到 ROS 2 功能包层级，包内的源码、
配置、Launch 和测试文件不再逐项列出。

.. code-block:: text

   KrtHumanRobot/
   ├── docs/                              # Sphinx 项目文档
   ├── scripts/                           # 工作区依赖导入脚本
   ├── third_party/                       # 第三方依赖清单
   ├── src/                               # ROS 2 源码工作区
   │   ├── krt_human_robot/               # 整机行为树、任务规划与动作调度
   │   ├── voice_assistant/               # 语音栈统一启动、配置和模型环境
   │   ├── voice_stack/                   # 模块化语音处理链路
   │   │   ├── voice_interfaces/          # 语音消息、服务和动作接口
   │   │   ├── voice_audio_capture/       # 麦克风音频采集
   │   │   ├── voice_audio_process/       # 降噪、VAD 等音频预处理
   │   │   ├── voice_kws/                 # 唤醒词检测
   │   │   ├── voice_asr/                 # 自动语音识别
   │   │   ├── voice_tts/                 # 文本转语音
   │   │   ├── voice_playback/            # 音频播放
   │   │   └── voice_volume/              # 系统音量控制
   │   ├── voice_test_tools/              # 语音接口独立测试客户端
   │   ├── hand_camera_driver/            # 左右手 USB 相机图像发布
   │   ├── hands_control/                 # 双灵巧手控制节点
   │   ├── hands_control_interfaces/      # 灵巧手动作接口
   │   ├── agx_action_group_runner/       # 双臂动作组执行与示教
   │   ├── agx_action_group_interfaces/   # 动作组接口定义
   │   ├── agx_arm_ros/                   # AGX 机械臂 ROS 2 集成
   │   │   └── src/
   │   │       ├── agx_arm_ctrl/          # 机械臂控制节点
   │   │       ├── agx_arm_msgs/          # 机械臂消息接口
   │   │       ├── agx_arm_description/   # 机械臂 URDF 与模型资源
   │   │       └── agx_arm_moveit/        # MoveIt 配置与规划入口
   │   ├── krt_task/                      # 跨子系统任务流程执行
   │   ├── krt_task_interfaces/           # 任务动作接口
   │   ├── ranger_nav/                    # 底盘、建图、定位和 Nav2 组合入口
   │   ├── ranger_ros2/
   │   │   └── agx_bringup/               # Ranger 底盘驱动与启动配置
   │   ├── livox_ros_driver2/             # Livox MID-360 雷达驱动
   │   ├── FAST_LIO_ROS2/                 # Fast-LIO 激光惯性里程计与建图
   │   ├── spark-fast-lio/
   │   │   └── spark_fast_lio/            # Spark Fast-LIO 建图后端
   │   ├── KISS-Matcher/
   │   │   └── ros/                       # KISS Matcher 点云匹配节点
   │   ├── lidar_localization_ros2/       # 基于 PCL 的点云地图定位
   │   ├── ndt_omp_ros2/                  # NDT/GICP 点云配准
   │   ├── pcd2pgm/                       # PCD 点云转 Nav2 栅格地图
   │   └── realsense-ros/                 # Intel RealSense ROS 2 集成
   │       ├── realsense2_camera/         # 相机驱动节点
   │       ├── realsense2_camera_msgs/    # RealSense 消息和服务接口
   │       └── realsense2_description/    # 相机 URDF 与模型资源
   ├── build/                             # Colcon 和 Sphinx 构建产物
   ├── install/                           # ROS 2 工作区安装空间
   └── log/                               # Colcon 构建与测试日志

功能框架
--------

下图根据 :download:`原始 draw.io 框架图 <../krt_human_robot.drawio>` 重绘，并以
当前代码为准补充各模块的 ROS 2 实现。框架图表达能力关系，不表示所有模块都会在
默认启动时启用。

.. graphviz::
   :caption: KrtHumanRobot 功能模块
   :align: center

   digraph krt_human_robot {
     graph [rankdir=TB, bgcolor="transparent", pad=0.2, nodesep=0.35, ranksep=0.55];
     node [shape=box, style="rounded,filled", fillcolor="#f5f7fa", color="#667085", fontname="sans"];
     edge [color="#667085", fontname="sans", fontsize=10];

     user [label="用户", shape=ellipse, fillcolor="#ffffff"];
     tree [label="行为树\nkrt_human_robot_node", fillcolor="#dbeafe", color="#2563eb"];
     voice [label="语音\n麦克风 → KWS/VAD → ASR → TTS"];
     vision [label="视觉\n头部/手部相机 → VLM"];
     nav [label="导航与建图\n雷达 + 运动底盘"];
     action [label="动作组合\n机械臂 + 灵巧手 + 音频播放"];
     llm [label="LLM\n对话或任务规划"];
     speaker [label="扬声器", shape=ellipse, fillcolor="#ffffff"];

     user -> tree [label="唤醒词/指令"];
     tree -> voice;
     tree -> vision;
     tree -> nav;
     tree -> action;
     tree -> llm;
     voice -> speaker [label="TTS/音频"];
     vision -> speaker [label="描述"];
     nav -> speaker [label="执行反馈"];
     action -> speaker [label="执行反馈"];
     llm -> speaker [label="对话"];
   }

软硬件组成
------------

.. list-table::
   :header-rows: 1
   :widths: 18 28 26 28

   * - 能力
     - 主要包
     - 运行入口
     - 关键接口或数据
   * - 行为树
     - ``krt_human_robot``
     - ``robot.launch.py``
     - 唤醒、意图、规划和动作分发
   * - 语音
     - ``voice_assistant``、``voice_*``
     - ``voice_stack.launch.py``
     - ``/voice/audio/*``、``/voice/asr/*``、``/voice/tts/*``
   * - 视觉
     - ``realsense2_camera``、``hand_camera_driver``
     - RealSense 与双手相机 launch
     - 头部彩色/深度图像和双手图像
   * - 导航建图
     - ``ranger_nav``、``livox_ros_driver2``、``fast_lio``
     - ``mapping.launch.py``、``navigation*.launch.py``
     - ``/cmd_vel``、``/odom``、``/navigate_to_pose``
   * - 动作与任务
     - ``agx_action_group_runner``、``agx_arm_ctrl``、``hands_control``、
       ``krt_task``
     - 动作组、双臂、双手和任务 launch
     - 动作组、灵巧手和任务 action

传感器
------

代码中已接入 Livox MID-360、头部 RealSense 深度相机，以及左右手 USB 相机。当前
仓库没有统一的整机硬件规格表，因此这里不记录未经设备资料确认的量程、精度、视场角
或安装尺寸；补充这些数据时应以设备手册和实机标定结果为准。

坐标系
------

导航代码采用 ``map → odom → camera_init → body → base_footprint → base_link`` 的 TF
主链。Fast-LIO 提供 ``camera_init → body``，导航 launch 建立 ``odom → camera_init``，
URDF 提供 ``body → base_footprint → base_link``。雷达安装偏移集中配置在
``src/ranger_nav/urdf/ranger_mini.urdf.xacro``，修改后必须重新验证完整 TF 链。

.. code-block:: bash

   ros2 run tf2_ros tf2_echo map base_footprint
   ros2 run tf2_tools view_frames
