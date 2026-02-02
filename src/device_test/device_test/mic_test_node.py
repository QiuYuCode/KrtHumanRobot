#!/usr/bin/env python3
"""
麦克风测试节点
功能：测试麦克风录音，发布音频数据到ROS2话题
默认使用 Unitek Y-247A (C-Media USB Audio Device)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import time
import os
import tempfile


class MicTestNode(Node):
    """麦克风测试节点"""

    def __init__(self):
        super().__init__('mic_test_node')

        # 声明参数
        # 默认使用 C-Media USB Audio Device (Unitek Y-247A)
        default_mic = 'alsa_input.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.mono-fallback'
        self.declare_parameter('device', default_mic)
        self.declare_parameter('sample_rate', 44100)  # C-Media 设备采样率
        self.declare_parameter('channels', 1)
        self.declare_parameter('duration', 5)  # 录音时长（秒）
        self.declare_parameter('use_pulseaudio', True)

        # 获取参数
        self.device = self.get_parameter('device').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.duration = self.get_parameter('duration').value
        self.use_pulseaudio = self.get_parameter('use_pulseaudio').value

        # 创建发布者
        self.status_pub = self.create_publisher(String, 'mic/status', 10)

        # 录音状态
        self.is_recording = False
        self.recording_thread = None

        # 显示可用设备
        self._list_devices()

        # 创建服务定时器 - 发布状态
        self.timer = self.create_timer(1.0, self.status_callback)

        self.get_logger().info('麦克风测试节点已启动')
        self.get_logger().info(f'设备: {self.device}')
        self.get_logger().info(f'采样率: {self.sample_rate}Hz, 通道: {self.channels}')
        self.get_logger().info('按 Enter 开始录音测试...')

        # 启动命令监听线程
        self.command_thread = threading.Thread(target=self._command_listener, daemon=True)
        self.command_thread.start()

    def _list_devices(self):
        """列出可用的音频输入设备"""
        self.get_logger().info('=== 可用麦克风设备 ===')
        try:
            result = subprocess.run(['pactl', 'list', 'sources', 'short'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                for line in result.stdout.strip().split('\n'):
                    if line and 'monitor' not in line.lower():
                        parts = line.split('\t')
                        if len(parts) >= 2:
                            self.get_logger().info(f'  [{parts[0]}] {parts[1]}')
        except Exception as e:
            self.get_logger().warn(f'获取设备失败: {e}')

    def _command_listener(self):
        """监听用户命令"""
        while rclpy.ok():
            try:
                input()
                if not self.is_recording:
                    self.start_recording()
            except EOFError:
                break
            except Exception:
                break

    def start_recording(self):
        """开始录音"""
        if self.is_recording:
            self.get_logger().warn('正在录音中...')
            return
        self.is_recording = True
        self.recording_thread = threading.Thread(target=self._record, daemon=True)
        self.recording_thread.start()

    def _record(self):
        """执行录音"""
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp_file:
            output_file = tmp_file.name

        self.get_logger().info(f'开始录音 {self.duration} 秒...')
        
        msg = String()
        msg.data = 'recording'
        self.status_pub.publish(msg)

        try:
            if self.use_pulseaudio:
                # 使用 parecord (注意：不支持 --file-format 参数)
                cmd = ['parecord']
                if self.device and self.device != 'default':
                    cmd.extend(['-d', self.device])
                cmd.extend([
                    '--rate=' + str(self.sample_rate),
                    '--channels=' + str(self.channels),
                    '--format=s16le',
                    output_file
                ])
            else:
                # 使用 arecord
                cmd = [
                    'arecord', '-D', self.device,
                    '-f', 'S16_LE', '-r', str(self.sample_rate),
                    '-c', str(self.channels), '-d', str(self.duration),
                    output_file
                ]

            self.get_logger().info(f'执行: parecord -d ... {output_file}')
            
            if self.use_pulseaudio:
                process = subprocess.Popen(cmd, stderr=subprocess.PIPE)
                time.sleep(self.duration)
                process.terminate()
                process.wait()
            else:
                subprocess.run(cmd, check=True)

            self.get_logger().info(f'录音完成: {output_file}')
            
            if os.path.exists(output_file):
                file_size = os.path.getsize(output_file)
                self.get_logger().info(f'录音文件大小: {file_size} bytes')
                
                if file_size > 44:
                    self.get_logger().info('播放录音...')
                    self._play_audio(output_file)
                else:
                    self.get_logger().error('录音文件为空，请检查麦克风设备')
            
            if os.path.exists(output_file):
                os.remove(output_file)
            
        except Exception as e:
            self.get_logger().error(f'录音失败: {e}')
        finally:
            self.is_recording = False
            msg = String()
            msg.data = 'idle'
            self.status_pub.publish(msg)
            self.get_logger().info('按 Enter 开始新的录音测试...')

    def _play_audio(self, file_path):
        """播放音频文件"""
        output_device = 'alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo'
        try:
            subprocess.run(['paplay', '-d', output_device, file_path], check=True)
        except Exception:
            try:
                subprocess.run(['paplay', file_path], check=True)
            except Exception:
                try:
                    subprocess.run(['aplay', file_path], check=True)
                except Exception as e:
                    self.get_logger().error(f'播放失败: {e}')

    def status_callback(self):
        """发布状态"""
        msg = String()
        msg.data = 'recording' if self.is_recording else 'idle'
        self.status_pub.publish(msg)


def list_audio_devices():
    """列出所有音频设备"""
    print("=== PulseAudio 输入设备 ===")
    subprocess.run(['pactl', 'list', 'sources', 'short'])
    print("\n=== 推荐设备 (Unitek Y-247A) ===")
    print("  麦克风: alsa_input.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.mono-fallback")
    print("  扬声器: alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo")


def main(args=None):
    rclpy.init(args=args)
    
    print("=== 麦克风测试节点 ===")
    list_audio_devices()
    print()
    
    node = MicTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
