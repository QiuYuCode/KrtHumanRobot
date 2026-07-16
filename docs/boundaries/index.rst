.. _boundaries:

二次开发边界与安全声明
======================

软件边界
--------

``krt_human_robot`` 负责整机行为编排，通过 ROS 2 接口调用功能包。语音、导航、相机、
机械臂和灵巧手功能包不得反向导入核心包。扩展能力时优先复用现有 ROS interface；
只有跨包数据结构确实不同，才新增 msg、srv 或 action。

代码职责
--------

.. list-table::
   :header-rows: 1
   :widths: 29 43 28

   * - 代码位置
     - 职责
     - 修改场景
   * - ``krt_human_robot/robot_node.py``
     - ROS 节点、配置和行为树生命周期
     - 整机级参数或服务
   * - ``krt_human_robot/tree_factory.py``
     - 关键词与 LLM 规划分支
     - 行为增减或排序
   * - ``krt_human_robot/behaviors/core``
     - 唤醒、监听、意图、规划和动作
     - 新增整机行为
   * - ``krt_human_robot/adapters``
     - 外部功能包的 ROS 边界
     - 替换功能实现
   * - 各功能包 ``launch/``
     - 子系统自包含启动
     - 节点、参数文件或 remap

硬件安全
--------

* 机械臂、灵巧手和底盘运行前必须确认急停、工作空间和周围人员安全；
* 参数示例不能替代实机标定，尤其是雷达外参、速度、电流、压力和动作回放速度；
* 修改控制、导航或动作组后先进行低速、无负载验证；
* 节点异常时优先执行设备规定的安全停机流程，不依赖进程退出自动保证硬件安全。

凭据与数据
----------

云服务密钥、Web 证书私钥和账号信息不得写入 RST、YAML 示例或 Git。语音凭据使用
未跟踪的 ``src/voice_assistant/.env``；Web 账号、机器人数据库、地图和媒体目录属于
运行数据，应备份但不提交到源码仓库。

兼容边界
--------

项目目标环境是 ROS 2 Humble。RealSense ROS 固定为 ``4.56.4`` 兼容线。升级 ROS、
librealsense、Nav2、MoveIt 或硬件 SDK 前，必须重新构建相关包并验证 launch 参数、
接口类型、QoS 和 TF。

最小验证
--------

.. code-block:: bash

   colcon build --packages-select krt_human_robot --symlink-install
   source install/setup.bash
   ROS_LOG_DIR=/tmp/ros-log-check \
     ros2 launch krt_human_robot robot.launch.py --show-args
   colcon test --packages-select krt_human_robot --event-handlers console_direct+
   colcon test-result --verbose
