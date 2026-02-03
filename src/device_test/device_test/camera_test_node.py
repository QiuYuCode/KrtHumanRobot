#!/usr/bin/env python3
"""
摄像头测试节点
功能：测试USB摄像头，发布图像到ROS2话题
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import re


class CameraTestNode(Node):
    """摄像头测试节点"""

    def __init__(self):
        super().__init__('camera_test_node')

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
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # 初始化摄像头
        self.cap = None
        self._init_camera()

        # 退出标志
        self.should_exit = False
        
        # 创建定时器
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f'摄像头测试节点已启动 - 设备: /dev/video{self.device_id}')
        self.get_logger().info(f'分辨率: {self.width}x{self.height}, FPS: {self.fps}')

    def _init_camera(self):
        """初始化摄像头"""
        # 使用设备路径
        device_path = f'/dev/video{self.device_id}'
        self.get_logger().info(f'尝试打开: {device_path}')
        
        self.cap = cv2.VideoCapture(device_path)
        if not self.cap.isOpened():
            self.get_logger().error(f'无法打开摄像头 {device_path}')
            self.get_logger().error('请检查: 1) 设备是否存在 2) 是否被其他程序占用 3) 用户是否在video组')
            return False

        # 设置 MJPG 格式（大多数USB摄像头支持，提高帧率）
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
        if self.should_exit:
            return
            
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('读取帧失败')
            return

        # 发布图像
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'发布图像失败: {e}')

        # 显示预览
        if self.show_preview:
            # 在图像上添加信息
            info_text = f'Device: /dev/video{self.device_id} | {frame.shape[1]}x{frame.shape[0]}'
            cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (0, 255, 0), 2)
            cv2.putText(frame, 'Press Q to quit', (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                       0.7, (0, 255, 0), 2)
            
            cv2.imshow(f'Camera Test - /dev/video{self.device_id}', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('用户请求退出')
                self.should_exit = True
                self.timer.cancel()
                if self.cap is not None:
                    self.cap.release()
                cv2.destroyAllWindows()
                raise SystemExit(0)

    def destroy_node(self):
        """清理资源"""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def list_cameras():
    """列出所有可用的摄像头设备"""
    try:
        result = subprocess.run(['v4l2-ctl', '--list-devices'], 
                              capture_output=True, text=True)
        print("=== 可用摄像头设备 ===")
        print(result.stdout)
        
        # 解析设备
        devices = []
        lines = result.stdout.split('\n')
        current_name = ""
        for line in lines:
            if not line.startswith('\t') and line.strip():
                current_name = line.strip()
            elif '/dev/video' in line:
                match = re.search(r'/dev/video(\d+)', line)
                if match:
                    devices.append({
                        'name': current_name,
                        'device': line.strip(),
                        'id': int(match.group(1))
                    })
        return devices
    except Exception as e:
        print(f"列出设备失败: {e}")
        return []


def main(args=None):
    rclpy.init(args=args)
    
    # 显示可用摄像头
    cameras = list_cameras()
    if cameras:
        print("\n检测到以下摄像头:")
        for cam in cameras:
            print(f"  - {cam['device']}: {cam['name']}")
        print()
    
    node = CameraTestNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
