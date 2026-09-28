# krthumanrobot_urdf

`robot.urdf`、`meshes/`、`parts.json`、`user_model.json` 和
`export_report.json` 是整机 CAD 导出结果。MoveIt 使用
`config/robot_moveit.urdf`：它保留相同的连杆、关节和可视网格，将网格路径改为
`package://`，并附加模拟用 `ros2_control` 描述。

安装版模型增加地面根坐标系 `base_footprint`。固定关节
`base_footprint_joint` 定义 `base_footprint → base_link` 的平移为
`[0, 0, 0.66769819]` 米、旋转为零，由 `robot_state_publisher` 发布 TF。
高度由四轮原始碰撞网格在零转角下的最低点计算；四轮最低点应处于同一
水平面（允许 1 mm 差异）。原有 CAD 连杆坐标及相对安装位置保持一致。
这是平地演示的坐标关系，后续导航/物理仿真需接入实际底盘位姿。

规划碰撞模型由原始碰撞网格生成凸包。`base_link` 按原始 CAD 零件分别
生成凸包，保留机身的凹槽及机械臂运动空间；不能将整个机身合成单凸包，
否则会把正常零位误判为手臂与机身碰撞。其他连杆按连杆生成凸包。
腕部第 5 和第 7 连杆的
单凸包会封闭正常运动所需的间隙，因此这四个连杆保留原始分块碰撞网格。
原始导出文件始终保留，方便后续重新校准碰撞几何。

修改原始 URDF 或网格后，在仓库根目录重新生成并检查：

```bash
src/voice_assistant/.venv/bin/python src/krthumanrobot_urdf/scripts/generate_moveit_model.py
src/voice_assistant/.venv/bin/python src/krthumanrobot_moveit_config/scripts/generate_semantics.py
src/voice_assistant/.venv/bin/python -m pytest -q \
  src/krthumanrobot_urdf/test/test_moveit_model.py \
  src/krthumanrobot_moveit_config/test/test_semantics.py
```

生成脚本需要与 NumPy 兼容的 SciPy；仓库现有的 `src/voice_assistant/.venv`
可以运行。系统 Python 当前安装的 NumPy/SciPy 版本不兼容。
