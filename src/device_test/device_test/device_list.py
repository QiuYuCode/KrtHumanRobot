#!/usr/bin/env python3
"""
设备列表工具
功能：列出所有可用的外部设备（摄像头、麦克风、扬声器）
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import re
import os
import json


class DeviceListNode(Node):
    """设备列表节点"""

    def __init__(self):
        super().__init__('device_list_node')

        # 创建发布者
        self.devices_pub = self.create_publisher(String, 'devices/list', 10)

        # 收集设备信息
        self.devices = self._collect_all_devices()

        # 显示设备信息
        self._print_devices()

        # 发布设备信息
        self._publish_devices()

        self.get_logger().info('设备列表已生成')

    def _collect_all_devices(self):
        """收集所有设备信息"""
        devices = {
            'cameras': self._get_cameras(),
            'audio_inputs': self._get_audio_inputs(),
            'audio_outputs': self._get_audio_outputs(),
            'usb_devices': self._get_usb_devices()
        }
        return devices

    def _get_cameras(self):
        """获取摄像头列表"""
        cameras = []
        
        try:
            # 使用 v4l2-ctl 获取设备列表
            result = subprocess.run(['v4l2-ctl', '--list-devices'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                lines = result.stdout.split('\n')
                current_name = ""
                for line in lines:
                    if not line.startswith('\t') and line.strip():
                        current_name = line.strip().rstrip(':')
                    elif '/dev/video' in line:
                        device_path = line.strip()
                        video_id = re.search(r'/dev/video(\d+)', device_path)
                        if video_id:
                            # 获取设备详细信息
                            caps = self._get_camera_caps(device_path)
                            cameras.append({
                                'name': current_name,
                                'device': device_path,
                                'id': int(video_id.group(1)),
                                'capabilities': caps
                            })
        except Exception as e:
            self.get_logger().warn(f'获取摄像头列表失败: {e}')
        
        return cameras

    def _get_camera_caps(self, device_path):
        """获取摄像头支持的格式"""
        caps = []
        try:
            result = subprocess.run(['v4l2-ctl', '-d', device_path, '--list-formats-ext'],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                # 解析支持的格式
                for line in result.stdout.split('\n'):
                    if 'Size:' in line:
                        match = re.search(r'(\d+)x(\d+)', line)
                        if match:
                            res = f"{match.group(1)}x{match.group(2)}"
                            if res not in caps:
                                caps.append(res)
        except Exception:
            pass
        return caps[:5]  # 只返回前5种分辨率

    def _get_audio_inputs(self):
        """获取音频输入设备（麦克风）"""
        inputs = []
        
        # PulseAudio sources
        try:
            result = subprocess.run(['pactl', 'list', 'sources', 'short'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                for line in result.stdout.strip().split('\n'):
                    if line and 'monitor' not in line.lower():  # 排除监听设备
                        parts = line.split('\t')
                        if len(parts) >= 2:
                            inputs.append({
                                'type': 'pulseaudio',
                                'id': parts[0],
                                'name': parts[1],
                                'full_info': line
                            })
        except Exception as e:
            self.get_logger().warn(f'获取PulseAudio输入设备失败: {e}')

        # ALSA capture devices
        try:
            result = subprocess.run(['arecord', '-l'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'card' in line.lower():
                        match = re.search(r'card (\d+).*device (\d+)', line)
                        if match:
                            inputs.append({
                                'type': 'alsa',
                                'card': match.group(1),
                                'device': match.group(2),
                                'alsa_device': f'hw:{match.group(1)},{match.group(2)}',
                                'name': line.strip()
                            })
        except Exception as e:
            self.get_logger().warn(f'获取ALSA输入设备失败: {e}')

        return inputs

    def _get_audio_outputs(self):
        """获取音频输出设备（扬声器）"""
        outputs = []
        
        # PulseAudio sinks
        try:
            result = subprocess.run(['pactl', 'list', 'sinks', 'short'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                for line in result.stdout.strip().split('\n'):
                    if line:
                        parts = line.split('\t')
                        if len(parts) >= 2:
                            outputs.append({
                                'type': 'pulseaudio',
                                'id': parts[0],
                                'name': parts[1],
                                'full_info': line
                            })
        except Exception as e:
            self.get_logger().warn(f'获取PulseAudio输出设备失败: {e}')

        # ALSA playback devices
        try:
            result = subprocess.run(['aplay', '-l'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'card' in line.lower():
                        match = re.search(r'card (\d+).*device (\d+)', line)
                        if match:
                            outputs.append({
                                'type': 'alsa',
                                'card': match.group(1),
                                'device': match.group(2),
                                'alsa_device': f'hw:{match.group(1)},{match.group(2)}',
                                'name': line.strip()
                            })
        except Exception as e:
            self.get_logger().warn(f'获取ALSA输出设备失败: {e}')

        return outputs

    def _get_usb_devices(self):
        """获取USB设备列表"""
        usb_devices = []
        
        try:
            result = subprocess.run(['lsusb'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                for line in result.stdout.strip().split('\n'):
                    if line:
                        # 解析 lsusb 输出
                        match = re.search(r'Bus (\d+) Device (\d+): ID ([0-9a-f:]+) (.+)', line)
                        if match:
                            usb_devices.append({
                                'bus': match.group(1),
                                'device': match.group(2),
                                'id': match.group(3),
                                'name': match.group(4)
                            })
        except Exception as e:
            self.get_logger().warn(f'获取USB设备失败: {e}')

        return usb_devices

    def _print_devices(self):
        """打印设备信息"""
        print("\n" + "=" * 60)
        print("               外部设备列表")
        print("=" * 60)

        # 摄像头
        print("\n[摄像头]")
        print("-" * 40)
        if self.devices['cameras']:
            for cam in self.devices['cameras']:
                print(f"  {cam['device']}: {cam['name']}")
                if cam.get('capabilities'):
                    print(f"    支持分辨率: {', '.join(cam['capabilities'])}")
        else:
            print("  未检测到摄像头")

        # 音频输入
        print("\n[麦克风/音频输入]")
        print("-" * 40)
        if self.devices['audio_inputs']:
            pa_devices = [d for d in self.devices['audio_inputs'] if d['type'] == 'pulseaudio']
            if pa_devices:
                print("  PulseAudio 设备:")
                for dev in pa_devices:
                    print(f"    [{dev['id']}] {dev['name']}")
        else:
            print("  未检测到音频输入设备")

        # 音频输出
        print("\n[扬声器/音频输出]")
        print("-" * 40)
        if self.devices['audio_outputs']:
            pa_devices = [d for d in self.devices['audio_outputs'] if d['type'] == 'pulseaudio']
            if pa_devices:
                print("  PulseAudio 设备:")
                for dev in pa_devices:
                    print(f"    [{dev['id']}] {dev['name']}")
        else:
            print("  未检测到音频输出设备")

        # USB设备
        print("\n[USB设备]")
        print("-" * 40)
        if self.devices['usb_devices']:
            for dev in self.devices['usb_devices']:
                print(f"  [{dev['id']}] {dev['name']}")
        else:
            print("  未检测到USB设备")

        print("\n" + "=" * 60)

    def _publish_devices(self):
        """发布设备信息到ROS话题"""
        msg = String()
        msg.data = json.dumps(self.devices, ensure_ascii=False, indent=2)
        self.devices_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = DeviceListNode()
    
    # 只运行一次就退出
    rclpy.spin_once(node, timeout_sec=1.0)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
