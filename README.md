# 人形机器人组件环境安装说明

## MID 360 雷达

### 参考资料

1. [览沃 ros2 驱动程序](https://github.com/Livox-SDK/livox_ros_driver2)
2. [FAST_LIO](https://github.com/hku-mars/FAST_LIO/tree/ROS2?tab=readme-ov-file)

## 双目相机 realsens

**需要将代码切换到 4.56.4 分支版本进行编译安装，否则会报错**
原因是通过 jsetson 安装的 sdk 版本低于 realsense-ros 最新分支要求的 2.57
系统默认的 sdk 版本是 2.56.4 

### 参考资料

1. [RealSense ros 代码](https://github.com/realsenseai/realsense-ros)
2. [Jetson 安装 realsense sdk](https://github.com/realsenseai/librealsense/blob/master/doc/installation_jetson.md)
