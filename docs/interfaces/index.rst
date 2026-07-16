.. _interfaces:

接口说明
========

核心行为树
----------

根节点依次执行唤醒词等待、唤醒回复和对话循环。``use_llm_planner`` 决定对话路径：

* ``false``：监听 → 关键词意图识别 → 单个动作 → 播报；
* ``true``：监听 → LLM 生成动作计划 → 顺序执行多个动作 → 播报。

现有动作覆盖固定回复、视觉描述、拍照、录像、灵巧手、机械臂、任务流程、导航、
LLM 对话和退出会话。

语音接口
--------

.. list-table::
   :header-rows: 1
   :widths: 18 35 47

   * - 类型
     - 名称
     - 用途
   * - Topic
     - ``/voice/audio/raw``
     - 麦克风原始音频帧
   * - Topic
     - ``/voice/kws/events``、``/voice/vad/events``
     - 唤醒词和语音活动事件
   * - Service
     - ``/voice/asr/recognize``
     - 一次性语音识别
   * - Action
     - ``/voice/asr/stream``
     - 流式语音识别
   * - Service
     - ``/voice/tts/synthesize``
     - 文字转语音
   * - Action
     - ``/voice/playback/play``
     - 可反馈和取消的音频播放
   * - Service
     - ``/voice/playback/stop``、``/voice/volume/*``
     - 停止播放和音量控制

视觉接口
--------

头部相机默认发布彩色、深度和对齐深度图像；左右手相机发布
``/left_gripper/image_raw`` 和 ``/right_gripper/image_raw``。核心节点提供
``/krt_human_robot/vision/describe_scene`` 服务，通过 camera ID 选择图像源。

导航与建图接口
--------------

.. list-table::
   :header-rows: 1
   :widths: 18 35 47

   * - 类型
     - 名称
     - 用途
   * - Topic
     - ``/cmd_vel``、``/odom``
     - 底盘速度命令和里程计
   * - Action
     - ``/navigate_to_pose``
     - Nav2 目标点导航
   * - Service
     - ``/map_save``
     - 保存 Fast-LIO 点云地图
   * - Topic
     - ``/km_sam/save_dir``
     - 请求 Spark/KISS-Matcher 保存地图

机械臂与灵巧手接口
------------------

.. list-table::
   :header-rows: 1
   :widths: 18 42 40

   * - 类型
     - 名称
     - 用途
   * - Action
     - ``/agx_action_group/run_action_group``
     - 执行已保存的双臂动作组
   * - Service
     - ``/agx_action_group/start_teach``、``/agx_action_group/stop_teach``
     - 开始和停止动作示教
   * - Action
     - ``/left/hand_control``、``/right/hand_control``
     - 左右灵巧手控制
   * - Action
     - ``/left/reset_hand``、``/right/reset_hand``
     - 重置左右灵巧手

任务与 Web 接口
---------------

``/krt_task/run_routine`` action 顺序编排 TTS、音频、机械臂、灵巧手、导航和等待步骤。
Web 控制台通过同一 action 运行流程，并使用独立 Web 数据库存储账号和审计信息。

配置约定
--------

launch 参数负责启动哪些进程和部署路径，``krt_human_robot.yaml`` 负责行为、接口名和
硬件标定。关键分组包括 ``asr_*``、``llm_*``、``cameras``、``gripper_*``、
``robot_arm_*``、``routine_*`` 和 ``adapters.navigation``。
