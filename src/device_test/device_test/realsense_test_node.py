#!/usr/bin/env python3
"""
Intel RealSense 摄像头测试节点
功能：测试RealSense摄像头的RGB和深度流
支持通过 OpenCV 直接访问或通过 realsense-ros 包

设备映射 (Intel RealSense D435 via V4L2):
  /dev/video2: Depth (Z16 16-bit)
  /dev/video3: Depth Metadata
  /dev/video4: Infrared (GREY/UYVY)
  /dev/video5: Infrared Metadata  
  /dev/video6: RGB (YUYV)
  /dev/video7: RGB Metadata
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess


class RealSenseTestNode(Node):
    """RealSense摄像头测试节点"""

    def __init__(self):
        super().__init__('realsense_test_node')

        # 声明参数 - D435 V4L2 设备映射
        self.declare_parameter('rgb_device_id', 6)      # RGB流设备ID (YUYV)
        self.declare_parameter('depth_device_id', 2)    # 深度流设备ID (Z16)
        self.declare_parameter('ir_device_id', 4)       # 红外流设备ID (GREY)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('show_preview', True)
        self.declare_parameter('enable_rgb', True)
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('enable_ir', False)

        # 获取参数
        self.rgb_device_id = self.get_parameter('rgb_device_id').value
        self.depth_device_id = self.get_parameter('depth_device_id').value
        self.ir_device_id = self.get_parameter('ir_device_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.show_preview = self.get_parameter('show_preview').value
        self.enable_rgb = self.get_parameter('enable_rgb').value
        self.enable_depth = self.get_parameter('enable_depth').value
        self.enable_ir = self.get_parameter('enable_ir').value

        # 创建发布者
        self.rgb_pub = self.create_publisher(Image, 'realsense/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'realsense/depth/image_raw', 10)
        self.ir_pub = self.create_publisher(Image, 'realsense/infra/image_raw', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # 显示RealSense设备信息
        self._list_realsense_devices()

        # 初始化摄像头
        self.rgb_cap = None
        self.depth_cap = None
        self.ir_cap = None
        self._init_cameras()

        # 创建定时器
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('RealSense测试节点已启动')
        self.get_logger().info(f'RGB: video{self.rgb_device_id}, Depth: video{self.depth_device_id}, IR: video{self.ir_device_id}')

    def _list_realsense_devices(self):
        """列出RealSense设备"""
        self.get_logger().info('=== Intel RealSense 设备 ===')
        try:
            result = subprocess.run(['v4l2-ctl', '--list-devices'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                lines = result.stdout.split('\n')
                current_name = ""
                in_realsense = False
                for line in lines:
                    if not line.startswith('\t') and line.strip():
                        current_name = line.strip()
                        in_realsense = 'RealSense' in current_name or 'realsense' in current_name.lower()
                    elif '/dev/video' in line and in_realsense:
                        self.get_logger().info(f'  {line.strip()}')
                        
            # 显示设备映射说明
            self.get_logger().info('设备映射 (D435 V4L2):')
            self.get_logger().info('  video2: Depth (Z16)')
            self.get_logger().info('  video4: Infrared (GREY)')
            self.get_logger().info('  video6: RGB (YUYV)')
        except Exception as e:
            self.get_logger().warn(f'获取设备列表失败: {e}')

    def _init_cameras(self):
        """初始化摄像头"""
        # RGB - 使用设备路径
        if self.enable_rgb:
            rgb_path = f'/dev/video{self.rgb_device_id}'
            self.rgb_cap = cv2.VideoCapture(rgb_path, cv2.CAP_V4L2)
            if self.rgb_cap.isOpened():
                self.rgb_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.rgb_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.rgb_cap.set(cv2.CAP_PROP_FPS, self.fps)
                self.get_logger().info(f'RGB流已启用: {rgb_path}')
            else:
                self.get_logger().error(f'无法打开RGB流 {rgb_path}')
                self.rgb_cap = None

        # Depth - 使用设备路径
        if self.enable_depth:
            depth_path = f'/dev/video{self.depth_device_id}'
            self.depth_cap = cv2.VideoCapture(depth_path, cv2.CAP_V4L2)
            if self.depth_cap.isOpened():
                self.depth_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.depth_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.depth_cap.set(cv2.CAP_PROP_FPS, self.fps)
                self.get_logger().info(f'Depth流已启用: {depth_path}')
            else:
                self.get_logger().error(f'无法打开Depth流 {depth_path}')
                self.depth_cap = None

        # Infrared - 使用设备路径
        if self.enable_ir:
            ir_path = f'/dev/video{self.ir_device_id}'
            self.ir_cap = cv2.VideoCapture(ir_path, cv2.CAP_V4L2)
            if self.ir_cap.isOpened():
                self.ir_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.ir_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.ir_cap.set(cv2.CAP_PROP_FPS, self.fps)
                self.get_logger().info(f'IR流已启用: {ir_path}')
            else:
                self.get_logger().error(f'无法打开IR流 {ir_path}')
                self.ir_cap = None

    def timer_callback(self):
        """定时器回调，读取并发布图像"""
        frames = []
        
        # RGB
        if self.rgb_cap is not None and self.rgb_cap.isOpened():
            ret, frame = self.rgb_cap.read()
            if ret:
                try:
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'realsense_color_frame'
                    self.rgb_pub.publish(msg)
                except Exception as e:
                    self.get_logger().error(f'发布RGB图像失败: {e}')
                
                if self.show_preview:
                    cv2.putText(frame, 'RealSense RGB', (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    frames.append(('RGB', frame))

        # Depth
        if self.depth_cap is not None and self.depth_cap.isOpened():
            ret, frame = self.depth_cap.read()
            if ret:
                try:
                    # 深度图像通常是16位单通道
                    if len(frame.shape) == 2:
                        msg = self.bridge.cv2_to_imgmsg(frame, encoding='mono16')
                    else:
                        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'realsense_depth_frame'
                    self.depth_pub.publish(msg)
                except Exception as e:
                    self.get_logger().error(f'发布Depth图像失败: {e}')
                
                if self.show_preview:
                    # 转换深度图为可视化格式
                    if len(frame.shape) == 2:
                        depth_colormap = cv2.applyColorMap(
                            cv2.convertScaleAbs(frame, alpha=0.03), 
                            cv2.COLORMAP_JET
                        )
                    else:
                        depth_colormap = frame
                    cv2.putText(depth_colormap, 'RealSense Depth', (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    frames.append(('Depth', depth_colormap))

        # Infrared
        if self.ir_cap is not None and self.ir_cap.isOpened():
            ret, frame = self.ir_cap.read()
            if ret:
                try:
                    if len(frame.shape) == 2:
                        msg = self.bridge.cv2_to_imgmsg(frame, encoding='mono8')
                    else:
                        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'realsense_infra_frame'
                    self.ir_pub.publish(msg)
                except Exception as e:
                    self.get_logger().error(f'发布IR图像失败: {e}')
                
                if self.show_preview:
                    if len(frame.shape) == 2:
                        ir_display = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    else:
                        ir_display = frame
                    cv2.putText(ir_display, 'RealSense IR', (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    frames.append(('IR', ir_display))

        # 显示预览
        if self.show_preview and frames:
            # 水平拼接所有帧
            if len(frames) > 1:
                # 确保所有帧大小相同
                target_h = frames[0][1].shape[0]
                target_w = frames[0][1].shape[1]
                resized_frames = []
                for name, f in frames:
                    if f.shape[0] != target_h or f.shape[1] != target_w:
                        f = cv2.resize(f, (target_w, target_h))
                    resized_frames.append(f)
                combined = cv2.hconcat(resized_frames)
            else:
                combined = frames[0][1]
            
            cv2.putText(combined, 'Press Q to quit', (10, combined.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('RealSense Test', combined)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('用户请求退出')
                rclpy.shutdown()

    def destroy_node(self):
        """清理资源"""
        if self.rgb_cap is not None:
            self.rgb_cap.release()
        if self.depth_cap is not None:
            self.depth_cap.release()
        if self.ir_cap is not None:
            self.ir_cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    print("=== Intel RealSense 测试节点 ===")
    print("注意: 如需完整功能，建议使用 realsense-ros 包")
    print("  ros2 launch realsense2_camera rs_launch.py")
    print()

    node = RealSenseTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
