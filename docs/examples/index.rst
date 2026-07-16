.. _examples:

示例与命令
==========

子系统 launch
-------------

先追加 ``--show-args`` 查看当前安装版本支持的参数。

.. list-table::
   :header-rows: 1
   :widths: 23 49 28

   * - 能力
     - 命令
     - 说明
   * - 语音栈
     - ``ros2 launch voice_assistant voice_stack.launch.py``
     - 启动采集、KWS、VAD、ASR、TTS、播放和音量
   * - 头部相机
     - ``ros2 launch realsense2_camera rs_launch.py``
     - RealSense 驱动
   * - 双手相机
     - ``ros2 launch hand_camera_driver hand_cameras.launch.py``
     - 默认使用两个稳定设备别名
   * - 双手控制
     - ``ros2 launch hands_control hand_control_launch.py``
     - 默认设备 ID 为 1 和 2
   * - 任务流程
     - ``ros2 launch krt_task routine_runner.launch.py``
     - 提供流程 action
   * - 动作组示教
     - ``ros2 launch agx_action_group_runner teach_action_group.launch.py``
     - 动作组执行和示教服务
   * - 单机械臂
     - ``ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py``
     - 型号、命名空间和 CAN 参数通过 launch 传入
   * - Fast-LIO 建图
     - ``ros2 launch ranger_nav mapping.launch.py``
     - Livox、Fast-LIO、底盘、TF 和 RViz
   * - 2D 导航
     - ``ros2 launch ranger_nav navigation.launch.py``
     - Fast-LIO + Nav2
   * - 3D 定位导航
     - ``ros2 launch ranger_nav navigation_3dloc.launch.py``
     - 点云定位 + Nav2 + 碰撞监控

节点命令
--------

硬件节点直接运行时仍需传入与 launch 相同的参数，常规操作优先使用 launch。

.. code-block:: bash

   ros2 run krt_human_robot krt_human_robot_node --ros-args \
     -p config_file:="$PWD/src/krt_human_robot/config/krt_human_robot.yaml"
   ros2 run hand_camera_driver usb_camera_node --ros-args \
     -p device:=/dev/camera_left -p topic:=/left_gripper/image_raw

   ros2 run hands_control hand_control_server
   ros2 run hands_control hand_control_client
   ros2 run krt_task routine_runner
   ros2 run krt_task krt_robot_data --help
   ros2 run agx_action_group_runner action_group_runner
   ros2 run agx_action_group_runner teach_action_group
   ros2 run agx_arm_ctrl agx_arm_ctrl_single --ros-args -r __ns:=/left
   ros2 run ranger_nav waypoint_manager --help
   ros2 run ranger_nav nav_tf_diagnostics

语音 executable 依赖 ``src/voice_assistant/.venv`` 中的模型运行时，推荐通过
``voice_stack.launch.py`` 启动，不要绕过它的参数文件和 ``uv`` 环境。

典型流程
--------

建图并保存：

.. code-block:: bash

   ros2 launch ranger_nav mapping.launch.py
   ros2 service call /map_save std_srvs/srv/Trigger

使用已有地图导航：

.. code-block:: bash

   ros2 launch ranger_nav navigation.launch.py

启动动作组和任务编排：

.. code-block:: bash

   ros2 launch agx_action_group_runner teach_action_group.launch.py
   ros2 launch krt_task routine_runner.launch.py
