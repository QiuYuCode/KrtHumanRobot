from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
import sounddevice as sd
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from voice_interfaces.action import PlayAudio
from voice_interfaces.srv import StopPlayback

from voice_audio_capture.audio_devices import configure_audio_devices


class VoicePlaybackNode(Node):
    """音频播放节点，支持优先级抢占与停播服务。"""

    def __init__(self) -> None:
        super().__init__("voice_playback")
        self.declare_parameter("output_device_hint", "")

        configure_audio_devices(
            output_hint=str(self.get_parameter("output_device_hint").value),
            configure_input=False,
            configure_output=True,
            log_info=self.get_logger().info,
            log_warning=self.get_logger().warning,
        )

        self._lock = threading.Lock()
        self._current_priority = -1
        self._cancel_requested = False

        self._action_server = ActionServer(
            self,
            PlayAudio,
            "/voice/playback/play",
            execute_callback=self._execute_play_audio,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self._stop_srv = self.create_service(
            StopPlayback, "/voice/playback/stop", self._handle_stop
        )
        self.get_logger().info("voice_playback ready: action=/voice/playback/play")

    def _goal_callback(self, goal_request: PlayAudio.Goal) -> GoalResponse:
        with self._lock:
            if self._current_priority < 0:
                return GoalResponse.ACCEPT
            if goal_request.preempt_lower_priority and goal_request.priority > self._current_priority:
                self._cancel_requested = True
                try:
                    sd.stop()
                except Exception:
                    pass
                return GoalResponse.ACCEPT
        return GoalResponse.REJECT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        with self._lock:
            self._cancel_requested = True
        try:
            sd.stop()
        except Exception:
            pass
        return CancelResponse.ACCEPT

    def _decode_chunks(self, chunks: list) -> tuple[np.ndarray, int]:
        pcm_list: list[np.ndarray] = []
        sample_rate = 16000
        for chunk in chunks:
            if chunk.sample_rate:
                sample_rate = int(chunk.sample_rate)
            if not chunk.data:
                continue
            pcm = np.frombuffer(bytes(chunk.data), dtype=np.int16).astype(np.float32)
            if chunk.channels and int(chunk.channels) > 1:
                pcm = pcm.reshape(-1, int(chunk.channels)).mean(axis=1)
            pcm_list.append(pcm / 32768.0)
        if not pcm_list:
            return np.array([], dtype=np.float32), sample_rate
        return np.concatenate(pcm_list), sample_rate

    def _execute_play_audio(self, goal_handle):
        goal = goal_handle.request
        result = PlayAudio.Result()
        feedback = PlayAudio.Feedback()
        feedback.played_chunks = 0

        samples, sample_rate = self._decode_chunks(goal.chunks)
        if len(samples) == 0:
            result.success = False
            result.error_message = "播放数据为空"
            goal_handle.abort()
            return result

        with self._lock:
            self._current_priority = int(goal.priority)
            self._cancel_requested = False

        duration = len(samples) / float(sample_rate)
        try:
            sd.play(samples, samplerate=sample_rate, blocking=False)
            start = time.time()
            while time.time() - start < duration:
                if goal_handle.is_cancel_requested:
                    sd.stop()
                    goal_handle.canceled()
                    result.success = False
                    result.error_message = "播放被取消"
                    return result
                with self._lock:
                    if self._cancel_requested:
                        sd.stop()
                        goal_handle.canceled()
                        result.success = False
                        result.error_message = "播放被抢占或停止"
                        return result
                time.sleep(0.02)
            sd.stop()
            feedback.played_chunks = len(goal.chunks)
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            result.success = True
            result.error_message = ""
            return result
        except Exception as exc:
            goal_handle.abort()
            result.success = False
            result.error_message = str(exc)
            return result
        finally:
            with self._lock:
                self._current_priority = -1
                self._cancel_requested = False

    def _handle_stop(self, request: StopPlayback.Request, response: StopPlayback.Response):
        del request
        with self._lock:
            self._cancel_requested = True
            self._current_priority = -1
        try:
            sd.stop()
            response.success = True
            response.error_message = ""
        except Exception as exc:
            response.success = False
            response.error_message = str(exc)
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VoicePlaybackNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
