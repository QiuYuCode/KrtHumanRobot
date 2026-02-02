#!/usr/bin/env python3
"""
扬声器测试节点
功能：测试扬声器播放音频
默认使用 Unitek Y-247A (C-Media USB Audio Device)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import tempfile
import os
import math
import struct
import wave


class SpeakerTestNode(Node):
    """扬声器测试节点"""

    def __init__(self):
        super().__init__('speaker_test_node')

        # 声明参数
        # 默认使用 C-Media USB Audio Device (Unitek Y-247A)
        default_speaker = 'alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo'
        self.declare_parameter('device', default_speaker)
        self.declare_parameter('test_duration', 2.0)  # 测试音频时长
        self.declare_parameter('frequency', 440.0)  # 测试音频频率 (Hz)
        self.declare_parameter('volume', 0.5)  # 音量 (0.0-1.0)

        # 获取参数
        self.device = self.get_parameter('device').value
        self.test_duration = self.get_parameter('test_duration').value
        self.frequency = self.get_parameter('frequency').value
        self.volume = self.get_parameter('volume').value

        # 创建发布者
        self.status_pub = self.create_publisher(String, 'speaker/status', 10)

        # 播放状态
        self.is_playing = False

        # 显示可用设备
        self._list_devices()

        # 创建状态定时器
        self.timer = self.create_timer(1.0, self.status_callback)

        self.get_logger().info('扬声器测试节点已启动')
        self.get_logger().info(f'设备: {self.device}')
        self.get_logger().info('按 Enter 开始播放测试音频...')
        self.get_logger().info('  0: 单音 440Hz | 1: 扫频 | 2: 左右声道')

        # 启动命令监听线程
        self.command_thread = threading.Thread(target=self._command_listener, daemon=True)
        self.command_thread.start()

    def _list_devices(self):
        """列出可用的音频输出设备"""
        self.get_logger().info('=== 可用扬声器设备 ===')
        try:
            result = subprocess.run(['pactl', 'list', 'sinks', 'short'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                for line in result.stdout.strip().split('\n'):
                    if line:
                        parts = line.split('\t')
                        if len(parts) >= 2:
                            self.get_logger().info(f'  [{parts[0]}] {parts[1]}')
        except Exception as e:
            self.get_logger().warn(f'获取设备失败: {e}')

    def _command_listener(self):
        """监听用户命令"""
        while rclpy.ok():
            try:
                user_input = input()
                if not self.is_playing:
                    if user_input.strip().isdigit():
                        self.play_test_tone(int(user_input.strip()))
                    else:
                        self.play_test_tone()
            except EOFError:
                break
            except Exception:
                break

    def _generate_sine_wave(self, filename, duration, frequency, sample_rate=48000, volume=0.5):
        """生成正弦波音频文件"""
        num_samples = int(duration * sample_rate)
        
        with wave.open(filename, 'w') as wav_file:
            wav_file.setnchannels(2)  # 立体声
            wav_file.setsampwidth(2)  # 16位
            wav_file.setframerate(sample_rate)
            
            for i in range(num_samples):
                t = float(i) / sample_rate
                value = int(32767 * volume * math.sin(2 * math.pi * frequency * t))
                packed = struct.pack('<hh', value, value)
                wav_file.writeframes(packed)

    def _generate_sweep_tone(self, filename, duration, start_freq=200, end_freq=2000, 
                            sample_rate=48000, volume=0.5):
        """生成扫频音频"""
        num_samples = int(duration * sample_rate)
        
        with wave.open(filename, 'w') as wav_file:
            wav_file.setnchannels(2)
            wav_file.setsampwidth(2)
            wav_file.setframerate(sample_rate)
            
            for i in range(num_samples):
                t = float(i) / sample_rate
                freq = start_freq * math.pow(end_freq / start_freq, t / duration)
                value = int(32767 * volume * math.sin(2 * math.pi * freq * t))
                packed = struct.pack('<hh', value, value)
                wav_file.writeframes(packed)

    def _generate_left_right_test(self, filename, duration, frequency=440, 
                                  sample_rate=48000, volume=0.5):
        """生成左右声道测试音频"""
        num_samples = int(duration * sample_rate)
        half_samples = num_samples // 2
        
        with wave.open(filename, 'w') as wav_file:
            wav_file.setnchannels(2)
            wav_file.setsampwidth(2)
            wav_file.setframerate(sample_rate)
            
            # 前半段：左声道
            for i in range(half_samples):
                t = float(i) / sample_rate
                value = int(32767 * volume * math.sin(2 * math.pi * frequency * t))
                packed = struct.pack('<hh', value, 0)
                wav_file.writeframes(packed)
            
            # 后半段：右声道
            for i in range(half_samples):
                t = float(i) / sample_rate
                value = int(32767 * volume * math.sin(2 * math.pi * frequency * t))
                packed = struct.pack('<hh', 0, value)
                wav_file.writeframes(packed)

    def play_test_tone(self, test_type=0):
        """播放测试音频"""
        if self.is_playing:
            self.get_logger().warn('正在播放中...')
            return

        self.is_playing = True
        thread = threading.Thread(target=self._play, args=(test_type,), daemon=True)
        thread.start()

    def _play(self, test_type=0):
        """执行播放"""
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp_file:
            output_file = tmp_file.name

        try:
            if test_type == 1:
                self.get_logger().info('生成扫频测试音频 (200Hz - 2000Hz)...')
                self._generate_sweep_tone(output_file, self.test_duration, volume=self.volume)
            elif test_type == 2:
                self.get_logger().info('生成左右声道测试音频...')
                self._generate_left_right_test(output_file, self.test_duration,
                                              frequency=self.frequency, volume=self.volume)
            else:
                self.get_logger().info(f'生成 {self.frequency}Hz 正弦波测试音频...')
                self._generate_sine_wave(output_file, self.test_duration, 
                                        self.frequency, volume=self.volume)

            msg = String()
            msg.data = 'playing'
            self.status_pub.publish(msg)

            self.get_logger().info('播放中...')
            
            try:
                if self.device and self.device != 'default':
                    subprocess.run(['paplay', '-d', self.device, output_file], check=True)
                else:
                    subprocess.run(['paplay', output_file], check=True)
            except Exception:
                subprocess.run(['aplay', output_file], check=True)

            self.get_logger().info('播放完成')
            
        except Exception as e:
            self.get_logger().error(f'播放失败: {e}')
        finally:
            if os.path.exists(output_file):
                os.remove(output_file)
            
            self.is_playing = False
            msg = String()
            msg.data = 'idle'
            self.status_pub.publish(msg)
            
            self.get_logger().info('按 Enter 继续测试 (0:单音 1:扫频 2:左右声道)')

    def status_callback(self):
        """发布状态"""
        msg = String()
        msg.data = 'playing' if self.is_playing else 'idle'
        self.status_pub.publish(msg)


def list_audio_outputs():
    """列出所有音频输出设备"""
    print("=== PulseAudio 输出设备 ===")
    subprocess.run(['pactl', 'list', 'sinks', 'short'])
    print("\n=== 推荐设备 (Unitek Y-247A) ===")
    print("  扬声器: alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo")


def main(args=None):
    rclpy.init(args=args)
    
    print("=== 扬声器测试节点 ===")
    list_audio_outputs()
    print()
    
    node = SpeakerTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
