# krthumanrobot_moveit_config

整机双臂的 MoveIt 2 演示配置，运行于 ROS 2 Humble。它使用
`krthumanrobot_urdf` 提供的完整模型，在 RViz 中规划并通过
`mock_components/GenericSystem` 模拟执行，不连接真实驱动。

## 启动

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select krthumanrobot_urdf krthumanrobot_moveit_config --symlink-install
source install/setup.bash
ros2 launch krthumanrobot_moveit_config demo.launch.py
```

使用本包自己的整机 URDF 控制两台 Nero 实机时，使用独立命名空间和 CAN
端口启动官方驱动桥接：

```bash
ros2 launch krthumanrobot_moveit_config demo.launch.py \
  execution_backend:=agx_topic \
  left_can_port:=can_left right_can_port:=can_right
```

该模式使用本包的 `left_arm_link*_joint` / `right_arm_link*_joint` 规划关节，
桥接到官方 Nero 驱动的 `joint1` 至 `joint7`。两臂独立规划；当前不接末端
执行器，也不提供双臂联合规划的碰撞约束。

无图形环境可用 `launch_rviz:=false`。演示使用地面根坐标系
`base_footprint` 作为模型规划坐标系和 RViz Fixed Frame，网格平面为
`z=0`；`base_link` 位于其上方约 0.667698 米，四轮轮底与地面对齐。
固定 TF 由 URDF 的 `base_footprint_joint` 发布，不需要额外静态 TF 节点。
在 RViz 的 MotionPlanning 面板中选择 `left_arm`、`right_arm` 或
`both_arms`，设置目标后使用 Plan 和 Execute。左右臂的末端分别为
`left_hand_base_link` 和 `right_hand_base_link`；两臂共享一个规划场景，
各自使用 `left_arm_controller`、`right_arm_controller` 的
`FollowJointTrajectory` 接口。车轮和灵巧手保留在整机模型中，但不参与本阶段规划。

验收时先选择 `left_arm`，将 Start State 设为当前状态，再拖动末端小幅改变
Goal State，点击 Plan，成功后再 Execute。`both_arms` 支持双臂联合规划；
全零关节目标和垂直向下的 `home` 姿态均已验证。`home` 姿态中左右第二关节为
`+pi/2`，其余关节为零；全零目标仍表示第二关节水平零位。更新模型和 SRDF 后必须退出旧的
演示进程并重新启动，已运行的节点不会自动重新加载文件。

RViz 的碰撞着色会检查整机。SRDF 排除同一固定连接组件内部的安装接触，
例如机身、头部及相机；机身与活动手臂、左右臂之间仍然进行碰撞检测。

## 冒烟验证

以下脚本独立启动无图形演示，检查初始碰撞、左右末端正逆运动学、
三个规划组的规划、模拟执行及关节状态反馈，以及双臂完整关节目标
从 `home` 到全零再返回 `home` 的运动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/krt_moveit_ros_logs \
  python3 src/krthumanrobot_moveit_config/scripts/smoke_demo.py
```

无需启动 ROS 节点的 FCL 回归测试会检查整机零位和初始位姿，并确认没有
屏蔽机身与活动手臂的碰撞对：

```bash
colcon test --packages-select krthumanrobot_moveit_config --event-handlers console_direct+
colcon test-result --test-result-base build/krthumanrobot_moveit_config --verbose
```

模拟控制器的硬件插件只用于演示。后续接入 Gazebo、Isaac Sim 和真机时，
要分别替换状态与轨迹执行后端，并用各阶段的真实关节限制、碰撞几何和
执行保护重新验证；本包当前不提供这些后端。
