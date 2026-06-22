from __future__ import annotations

import uuid
import wave
from pathlib import Path

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from voice_interfaces.action import PlayAudio
from voice_interfaces.msg import VoiceAudioFrame
from voice_interfaces.srv import PlayMedia


class VoiceMediaNode(Node):
    """Load local WAV files and submit them to the shared playback action."""

    def __init__(self) -> None:
        super().__init__("voice_media")
        self._callback_group = ReentrantCallbackGroup()
        self._play_client = ActionClient(
            self,
            PlayAudio,
            "/voice/playback/play",
            callback_group=self._callback_group,
        )
        self._service = self.create_service(
            PlayMedia,
            "/voice/media/play",
            self._handle_play,
            callback_group=self._callback_group,
        )
        self.get_logger().info("voice_media ready: srv=/voice/media/play")

    @staticmethod
    def _read_wav(file_path: str) -> tuple[VoiceAudioFrame, float]:
        path = Path(file_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"媒体文件不存在: {path}")
        if path.suffix.lower() != ".wav":
            raise ValueError("当前仅支持 WAV 文件")

        with wave.open(str(path), "rb") as wav_file:
            if wav_file.getcomptype() != "NONE":
                raise ValueError("不支持压缩 WAV")
            channels = int(wav_file.getnchannels())
            sample_rate = int(wav_file.getframerate())
            sample_width = int(wav_file.getsampwidth())
            frame_count = int(wav_file.getnframes())
            audio = wav_file.readframes(frame_count)

        if channels < 1 or channels > 255:
            raise ValueError(f"不支持的声道数: {channels}")
        if sample_rate <= 0:
            raise ValueError(f"无效采样率: {sample_rate}")

        if sample_width == 1:
            pcm = (np.frombuffer(audio, dtype=np.uint8).astype(np.int16) - 128) << 8
            audio = pcm.astype("<i2").tobytes()
        elif sample_width == 2:
            audio = np.frombuffer(audio, dtype="<i2").tobytes()
        elif sample_width == 3:
            raw = np.frombuffer(audio, dtype=np.uint8).reshape(-1, 3)
            values = (
                raw[:, 0].astype(np.int32)
                | (raw[:, 1].astype(np.int32) << 8)
                | (raw[:, 2].astype(np.int32) << 16)
            )
            values = np.where(values & 0x800000, values | ~0xFFFFFF, values)
            audio = (values >> 8).astype("<i2").tobytes()
        elif sample_width == 4:
            values = np.frombuffer(audio, dtype="<i4")
            audio = (values >> 16).astype("<i2").tobytes()
        else:
            raise ValueError(f"不支持的 WAV 位深: {sample_width * 8} bit")

        frame = VoiceAudioFrame()
        frame.encoding = "pcm16"
        frame.sample_rate = sample_rate
        frame.channels = channels
        frame.data = list(audio)
        return frame, frame_count / float(sample_rate)

    def _handle_play(
        self, request: PlayMedia.Request, response: PlayMedia.Response
    ) -> PlayMedia.Response:
        try:
            frame, duration = self._read_wav(request.file_path)
            if not self._play_client.wait_for_server(timeout_sec=2.0):
                raise RuntimeError("playback action 不可用")

            request_id = uuid.uuid4().hex
            goal = PlayAudio.Goal()
            goal.chunks = [frame]
            goal.priority = request.priority
            goal.preempt_lower_priority = request.preempt_lower_priority
            future = self._play_client.send_goal_async(goal)
            future.add_done_callback(
                lambda done, rid=request_id: self._on_goal_response(rid, done)
            )

            response.accepted = True
            response.request_id = request_id
            response.duration_sec = float(duration)
            response.error_message = ""
            self.get_logger().info(
                f"媒体已提交播放: request_id={request_id} "
                f"file={request.file_path} duration={duration:.2f}s"
            )
        except Exception as exc:
            response.accepted = False
            response.request_id = ""
            response.duration_sec = 0.0
            response.error_message = str(exc)
            self.get_logger().error(f"媒体播放请求失败: {exc}")
        return response

    def _on_goal_response(self, request_id: str, future) -> None:
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning(f"媒体播放被拒绝: request_id={request_id}")
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda done, rid=request_id: self._on_play_result(rid, done)
            )
        except Exception as exc:
            self.get_logger().error(
                f"媒体播放提交失败: request_id={request_id} error={exc}"
            )

    def _on_play_result(self, request_id: str, future) -> None:
        try:
            result = future.result().result
            log = self.get_logger().info if result.success else self.get_logger().warning
            log(
                f"媒体播放完成: request_id={request_id} "
                f"success={result.success} error={result.error_message}"
            )
        except Exception as exc:
            self.get_logger().error(
                f"媒体播放结果异常: request_id={request_id} error={exc}"
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VoiceMediaNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
