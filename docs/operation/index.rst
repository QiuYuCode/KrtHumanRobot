.. _operation:

操作指南
========

设备连接与检查
--------------

启动机器人软件前先确认设备节点和当前用户权限：

.. code-block:: bash

   ls -l /dev/video*
   ls -l /dev/camera_left /dev/camera_right
   groups
   ip -br link

相机要求当前用户属于 ``video`` 组。机械臂、灵巧手和底盘要求对应 CAN 设备已创建并
处于 ``UP`` 状态。设备名称必须与 launch 参数和
``src/krt_human_robot/config/krt_human_robot.yaml`` 一致。

启动与停止
----------

整机软件统一从顶层 launch 启动：

.. code-block:: bash

   cd /home/nvidia/WorkSpace/KrtHumanRobot
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch krt_human_robot robot.launch.py

停止时在 launch 终端按 ``Ctrl+C``，等待节点完成清理后再关闭设备电源。仓库没有描述
实体机器人的上电、急停复位和断电顺序；这些步骤必须使用机器人平台的正式操作规程，
不能从软件 launch 行为推断。

Web 控制台
----------

Web 控制台默认关闭。生产使用必须配置证书和私钥：

.. code-block:: bash

   ros2 launch krt_human_robot robot.launch.py \
     enable_web_console:=true web_port:=8443 \
     web_certfile:=/path/to/cert.pem web_keyfile:=/path/to/key.pem

首次使用可创建管理员账号：

.. code-block:: bash

   ros2 run krt_human_robot krt_web_create_admin

控制台使用单 Gunicorn worker，以保证内嵌 ROS 节点和动作客户端只有一个运行实例。

日常状态检查
------------

.. code-block:: bash

   ros2 node list
   ros2 topic list -t
   ros2 service list -t
   ros2 action list -t
   ros2 doctor --report
