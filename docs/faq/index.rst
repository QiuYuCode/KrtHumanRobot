.. _faq:

FAQ 与故障排查
==============

找不到包或 executable
---------------------

重新构建并 source 当前工作区：

.. code-block:: bash

   colcon build --symlink-install
   source /opt/ros/humble/setup.bash
   source install/setup.bash

检查单个包的 executable：

.. code-block:: bash

   ros2 pkg executables krt_human_robot
   ros2 pkg executables ranger_nav

topic 存在但没有数据
--------------------

.. code-block:: bash

   ros2 topic info -v /topic_name
   ros2 topic hz /topic_name

确认发布者存在，并检查发布者与订阅者的 Reliability、Durability 和 History 是否兼容。
传感器流通常使用 ``BEST_EFFORT``，不能只按默认 ``RELIABLE`` 订阅判断数据是否存在。

相机无法打开
------------

确认设备别名、权限和占用进程：

.. code-block:: bash

   ls -l /dev/camera_left /dev/camera_right /dev/video*
   groups
   ps -ef | rg 'realsense|usb_camera'

TF 或导航失败
-------------

.. code-block:: bash

   ros2 run tf2_ros tf2_echo map base_footprint
   ros2 topic echo /scan --once --field header.frame_id
   ros2 lifecycle get /map_server

重点检查 ``map → odom → camera_init → body → base_footprint`` 是否连续，以及传感器
消息的 ``frame_id`` 是否在 TF 树中存在。

语音节点启动失败
----------------

确认 ``src/voice_assistant/.venv``、语音模型和环境变量已准备：

.. code-block:: bash

   cd src/voice_assistant
   bash scripts/setup_uv.sh
   bash scripts/download_voice_models.sh

CAN 设备或机械臂无响应
----------------------

.. code-block:: bash

   ip -br link

确认 launch 中的 ``can_left``、``can_right`` 与实际接口一致。不要在机械臂处于运动或
负载状态时反复重启驱动；先按设备规程进入安全状态。
