#!/usr/bin/env python3
"""
USB 摄像头测试节点
功能：测试普通USB摄像头，发布图像到ROS2话题
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import re


class USBCameraTestNode(Node):
    """USB摄像头测试节点"""

    def __init__(self):
        super().__init__('usb_camera_test_node')

        # 声明参数
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('show_preview', True)

        # 获取参数
        self.device_id = self.get_parameter('device_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.show_preview = self.get_parameter('show_preview').value

        # 创建发布者
        self.image_pub = self.create_publisher(Image, 'usb_camera/image_raw', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # 显示USB摄像头列表
        self._list_usb_cameras()

        # 初始化摄像头
        self.cap = None
        self._init_camera()

        # 创建定时器
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f'USB摄像头测试节点已启动 - 设备: /dev/video{self.device_id}')
        self.get_logger().info(f'分辨率: {self.width}x{self.height}, FPS: {self.fps}')

    def _list_usb_cameras(self):
        """列出USB摄像头设备（排除RealSense）"""
        self.get_logger().info('=== USB 摄像头设备 ===')
        try:
            result = subprocess.run(['v4l2-ctl', '--list-devices'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                lines = result.stdout.split('\n')
                current_name = ""
                for line in lines:
                    if not line.startswith('\t') and line.strip():
                        current_name = line.strip()
                    elif '/dev/video' in line:
                        # 排除 RealSense 设备
                        if 'RealSense' not in current_name and 'realsense' not in current_name.lower():
                            self.get_logger().info(f'  {line.strip()}: {current_name}')
        except Exception as e:
            self.get_logger().warn(f'获取设备列表失败: {e}')

    def _init_camera(self):
        """初始化摄像头"""
        # 使用 V4L2 后端而非 GStreamer
        self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'无法打开摄像头 /dev/video{self.device_id}')
            return False

        # 设置 MJPG 格式（大多数USB摄像头支持）
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        
        # 设置分辨率
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # 读取实际设置
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        self.get_logger().info(f'实际分辨率: {actual_width}x{actual_height}, FPS: {actual_fps}')
        
        # 预热：丢弃前几帧
        self.get_logger().info('摄像头预热中...')
        for _ in range(10):
            self.cap.read()
        
        return True

    def timer_callback(self):
        """定时器回调，读取并发布图像"""
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('读取帧失败，尝试重新打开摄像头')
            self.cap.release()
            self._init_camera()
            return
        
        if frame.size == 0:
            self.get_logger().warn('读取到空帧')
            return

        # 发布图像
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'usb_camera_frame'
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'发布图像失败: {e}')

        # 显示预览
        if self.show_preview:
            info_text = f'USB Camera /dev/video{self.device_id} | {frame.shape[1]}x{frame.shape[0]}'
            cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                       0.7, (0, 255, 0), 2)
            cv2.putText(frame, 'Press Q to quit', (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                       0.7, (0, 255, 0), 2)

            cv2.imshow(f'USB Camera - /dev/video{self.device_id}', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('用户请求退出')
                rclpy.shutdown()

    def destroy_node(self):
        """清理资源"""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = USBCameraTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
