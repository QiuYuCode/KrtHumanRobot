"""Focused resilience tests for the cloud TTS request boundary."""

import base64
import importlib
import json
import sys
import threading
import time as wall_time
import types
from unittest.mock import patch

import pytest


def _install_import_stubs():
    """Provide only unavailable ROS/cloud dependencies for this unit test."""
    rclpy = types.ModuleType("rclpy")
    action = types.ModuleType("rclpy.action")
    node = types.ModuleType("rclpy.node")

    class Node:
        pass

    action.ActionClient = object
    node.Node = Node
    rclpy.action = action
    rclpy.node = node

    sherpa_onnx = types.ModuleType("sherpa_onnx")
    websocket = types.ModuleType("websocket")

    class WebSocketException(Exception):
        pass

    class WebSocketAddressException(WebSocketException):
        pass

    class WebSocketConnectionClosedException(WebSocketException):
        pass

    class WebSocketTimeoutException(WebSocketException):
        pass

    websocket.WebSocketException = WebSocketException
    websocket.WebSocketAddressException = WebSocketAddressException
    websocket.WebSocketConnectionClosedException = (
        WebSocketConnectionClosedException
    )
    websocket.WebSocketTimeoutException = WebSocketTimeoutException
    websocket.create_connection = None

    interfaces = types.ModuleType("voice_interfaces")
    interfaces_action = types.ModuleType("voice_interfaces.action")
    interfaces_msg = types.ModuleType("voice_interfaces.msg")
    interfaces_srv = types.ModuleType("voice_interfaces.srv")

    class PlayAudio:
        class Goal:
            pass

    class VoiceAudioFrame:
        pass

    class SynthesizeSpeech:
        class Request:
            pass

        class Response:
            pass

    interfaces_action.PlayAudio = PlayAudio
    interfaces_msg.VoiceAudioFrame = VoiceAudioFrame
    interfaces_srv.SynthesizeSpeech = SynthesizeSpeech

    tenacity = types.ModuleType("tenacity")

    class _RetryCondition:
        def __init__(self, exception_types):
            self.exception_types = exception_types

    class _StopCondition:
        def __init__(self, max_attempts=None, max_delay=None):
            self.max_attempts = max_attempts
            self.max_delay = max_delay

        def __or__(self, other):
            return _StopCondition(
                max_attempts=self.max_attempts or other.max_attempts,
                max_delay=self.max_delay or other.max_delay,
            )

    class _WaitChain:
        def __init__(self, delays):
            self.delays = delays

    def retry_if_exception_type(exception_types):
        return _RetryCondition(exception_types)

    def stop_after_attempt(attempts):
        return _StopCondition(max_attempts=attempts)

    def stop_after_delay(seconds):
        return _StopCondition(max_delay=seconds)

    def wait_fixed(seconds):
        return seconds

    def wait_chain(*delays):
        return _WaitChain(delays)

    def retry(*, retry, stop, wait, reraise):
        del reraise

        def decorate(function):
            def wrapped(*args, **kwargs):
                for attempt in range(stop.max_attempts):
                    try:
                        return function(*args, **kwargs)
                    except retry.exception_types:
                        if attempt == stop.max_attempts - 1:
                            raise
                        tenacity.sleep_delays.append(wait.delays[attempt])
                raise AssertionError("unreachable")

            return wrapped

        return decorate

    tenacity.retry = retry
    tenacity.retry_if_exception_type = retry_if_exception_type
    tenacity.stop_after_attempt = stop_after_attempt
    tenacity.stop_after_delay = stop_after_delay
    tenacity.wait_chain = wait_chain
    tenacity.wait_fixed = wait_fixed
    tenacity.sleep_delays = []

    stubs = {
            "rclpy": rclpy,
            "rclpy.action": action,
            "rclpy.node": node,
            "sherpa_onnx": sherpa_onnx,
            "websocket": websocket,
            "voice_interfaces": interfaces,
            "voice_interfaces.action": interfaces_action,
            "voice_interfaces.msg": interfaces_msg,
            "voice_interfaces.srv": interfaces_srv,
            "tenacity": tenacity,
    }
    missing = object()
    originals = {name: sys.modules.get(name, missing) for name in stubs}
    sys.modules.update(stubs)
    return stubs, originals, missing


_stubs, _originals, _missing = _install_import_stubs()
try:
    tts_node = importlib.import_module("voice_tts.tts_node")
finally:
    for name, module in _originals.items():
        if module is _missing:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = module


class _Parameter:
    def __init__(self, value):
        self.value = value


class _Logger:
    def warning(self, _message):
        pass

    def info(self, _message):
        pass


class _Connection:
    def __init__(self, response):
        self._response = response
        self.timeouts = []

    def settimeout(self, timeout):
        self.timeouts.append(timeout)

    def send(self, _payload):
        pass

    def recv(self):
        return self._response

    def close(self, timeout=None):
        self.close_timeout = timeout


def _cloud_node():
    node = tts_node.VoiceTtsNode.__new__(tts_node.VoiceTtsNode)
    values = {
        "iflytek_tts_app_id": "app",
        "iflytek_tts_api_key": "key",
        "iflytek_tts_api_secret": "secret",
        "iflytek_tts_aue": "raw",
        "iflytek_tts_auf": "audio/L16;rate=16000",
        "iflytek_tts_request_text_encoding": "utf-8",
        "iflytek_tts_vcn": "speaker",
        "iflytek_tts_speed": 50,
        "iflytek_tts_tte": "UTF8",
    }
    node.get_parameter = lambda name: _Parameter(values[name])
    node.get_logger = lambda: _Logger()
    node._split_text_by_cloud_limit = lambda _text: ["你好"]
    node._build_ws_url = lambda _key, _secret: "wss://example.invalid/tts"
    return node


def test_cloud_transport_failure_retries_until_audio_is_synthesized():
    """Removing transient retrying would return an error instead of audio."""
    node = _cloud_node()
    attempts = []
    _stubs["tenacity"].sleep_delays.clear()
    completed = json.dumps(
        {
            "code": 0,
            "data": {"audio": base64.b64encode(b"pcm").decode(), "status": 2},
        }
    )

    def create_connection(*_args, **_kwargs):
        attempts.append(_kwargs)
        if len(attempts) < 3:
            raise tts_node.websocket.WebSocketConnectionClosedException(
                "temporary disconnect"
            )
        return _Connection(completed)

    with patch.object(
        tts_node.websocket,
        "create_connection",
        create_connection,
    ):
        pcm, sample_rate = node._generate_iflytek_tts("你好")

    assert pcm == b"pcm"
    assert sample_rate == 16000
    assert len(attempts) == 3
    assert all(3.9 < attempt["timeout"] <= 4.0 for attempt in attempts)
    assert _stubs["tenacity"].sleep_delays == [1.0, 2.0]


def test_cloud_business_error_is_not_retried():
    """Treating a cloud business response as transient would repeat it."""
    node = _cloud_node()
    attempts = []
    rejected = json.dumps({"code": 10110, "message": "invalid request"})

    def create_connection(*_args, **_kwargs):
        attempts.append(1)
        return _Connection(rejected)

    with patch.object(
        tts_node.websocket,
        "create_connection",
        create_connection,
    ):
        with pytest.raises(RuntimeError, match="讯飞 TTS 错误"):
            node._generate_iflytek_tts("你好")

    assert len(attempts) == 1


def test_expired_synthesis_deadline_rejects_request_without_playback():
    """Removing the final deadline check would submit a late PlayAudio goal."""
    node = _cloud_node()
    submitted_goals = []

    class _PlayClient:
        def wait_for_server(self, timeout_sec):
            assert timeout_sec == 1.0
            return True

        def send_goal_async(self, goal):
            submitted_goals.append(goal)

    class _Request:
        text = "你好"
        priority = 1

    class _Response:
        pass

    node._play_client = _PlayClient()
    node._synthesize_pcm16 = lambda _text, _deadline: (b"pcm", 16000, "cloud")
    with patch.object(tts_node.time, "monotonic", side_effect=[0.0, 18.0]):
        response = node._handle_synthesize(_Request(), _Response())

    assert response.accepted is False
    assert response.error_message == "TTS 合成超时"
    assert submitted_goals == []


def test_cloud_failure_falls_back_to_local_with_remaining_deadline():
    """A recoverable cloud failure uses the original request deadline."""
    node = _cloud_node()
    values = {
        "tts_backend": "iflytek_cloud",
        "cloud_tts_fallback_to_local": True,
    }
    node.get_parameter = lambda name: _Parameter(values[name])
    node.get_logger = _Logger

    def failed_cloud(*_args):
        raise RuntimeError("temporary cloud failure")

    node._generate_iflytek_tts = failed_cloud
    node._ensure_tts_ready = lambda: True
    node._generate_local_tts = lambda _text: (object(), 16000)
    node._float32_to_pcm16 = lambda _samples: b"pcm"

    assert node._synthesize_pcm16("你好", deadline=wall_time.monotonic() + 1.0) == (
        b"pcm", 16000, "local"
    )


def test_expired_cloud_deadline_does_not_extend_for_local_fallback():
    node = _cloud_node()
    node.get_parameter = lambda name: _Parameter({
        "tts_backend": "iflytek_cloud",
        "cloud_tts_fallback_to_local": True,
    }[name])
    node.get_logger = _Logger
    node._generate_iflytek_tts = lambda *_args: (_ for _ in ()).throw(
        tts_node.SynthesisDeadlineExceeded("TTS 合成超时")
    )
    node._ensure_tts_ready = lambda: True
    node._generate_local_tts = lambda _text: (object(), 16000)
    node._float32_to_pcm16 = lambda _samples: b"pcm"

    with pytest.raises(tts_node.SynthesisDeadlineExceeded):
        node._synthesize_pcm16("你好", deadline=wall_time.monotonic() - 1.0)


def test_cloud_cleanup_receives_the_remaining_attempt_timeout():
    """An unbounded close could exceed the 4-second cloud attempt deadline."""
    node = _cloud_node()
    completed = json.dumps(
        {
            "code": 0,
            "data": {"audio": base64.b64encode(b"pcm").decode(), "status": 2},
        }
    )

    class _BoundedCloseConnection(_Connection):
        def __init__(self):
            super().__init__(completed)
            self.close_timeout = None

        def close(self, timeout):
            self.close_timeout = timeout

    connection = _BoundedCloseConnection()
    with patch.object(
        tts_node.websocket,
        "create_connection",
        return_value=connection,
    ):
        pcm, _sample_rate = node._generate_iflytek_tts("你好")

    assert pcm == b"pcm"
    assert 0.0 < connection.close_timeout <= 4.0


def test_expiry_during_goal_building_does_not_submit_playback():
    """A missing post-build deadline check would submit a late goal."""
    node = _cloud_node()
    submitted_goals = []

    class _Clock:
        value = 0.0

        @classmethod
        def monotonic(cls):
            return cls.value

    class _PlayClient:
        def wait_for_server(self, timeout_sec):
            assert timeout_sec == 1.0
            return True

        def send_goal_async(self, goal):
            submitted_goals.append(goal)

    class _Request:
        text = "你好"
        priority = 1

    class _Response:
        pass

    def build_goal(*_args):
        _Clock.value = 18.0
        return object()

    node._play_client = _PlayClient()
    node._synthesize_pcm16 = lambda _text, _deadline: (b"pcm", 16000, "cloud")
    node._build_play_goal_from_pcm = build_goal
    with patch.object(tts_node.time, "monotonic", _Clock.monotonic):
        response = node._handle_synthesize(_Request(), _Response())

    assert response.accepted is False
    assert response.error_message == "TTS 合成超时"
    assert submitted_goals == []


def test_recv_timeout_at_attempt_deadline_retries_after_cleanup():
    """Cleanup must not turn a retryable recv timeout into a deadline error."""
    node = _cloud_node()
    attempts = []
    completed = json.dumps(
        {
            "code": 0,
            "data": {"audio": base64.b64encode(b"pcm").decode(), "status": 2},
        }
    )

    class _Clock:
        value = 0.0

        @classmethod
        def monotonic(cls):
            return cls.value

    class _TimedOutConnection(_Connection):
        def __init__(self):
            super().__init__(completed)
            self.close_timeouts = []
            self.socket_closed = False

            class _Socket:
                def close(socket_self):
                    del socket_self
                    self.socket_closed = True

            self.sock = _Socket()

        def recv(self):
            _Clock.value = 4.0
            raise tts_node.websocket.WebSocketTimeoutException(
                "receive timed out"
            )

        def close(self, timeout=None):
            self.close_timeouts.append(timeout)

    timed_out = _TimedOutConnection()
    succeeded = _Connection(completed)

    def create_connection(*_args, **_kwargs):
        attempts.append(1)
        return timed_out if len(attempts) == 1 else succeeded

    with patch.object(tts_node.time, "monotonic", _Clock.monotonic):
        with patch.object(
            tts_node.websocket,
            "create_connection",
            create_connection,
        ):
            pcm, _sample_rate = node._generate_iflytek_tts("你好")

    assert pcm == b"pcm"
    assert len(attempts) == 2
    assert timed_out.close_timeouts == []
    assert timed_out.socket_closed is True


def test_cloud_business_error_survives_cleanup_failure_without_retry():
    """A cleanup error must not hide an actionable cloud business rejection."""
    node = _cloud_node()
    attempts = []
    rejected = json.dumps({"code": 10110, "message": "invalid request"})

    class _BrokenCloseConnection(_Connection):
        def close(self, timeout=None):
            del timeout
            raise tts_node.websocket.WebSocketTimeoutException("close failed")

    def create_connection(*_args, **_kwargs):
        attempts.append(1)
        return _BrokenCloseConnection(rejected)

    with patch.object(
        tts_node.websocket,
        "create_connection",
        create_connection,
    ):
        with pytest.raises(RuntimeError, match="讯飞 TTS 错误"):
            node._generate_iflytek_tts("你好")

    assert len(attempts) == 1


def test_local_generation_timeout_returns_before_service_deadline():
    """Synchronous local generation would exceed the service deadline."""
    node = _cloud_node()
    release_generation = threading.Event()
    submitted_goals = []
    values = {"tts_backend": "local"}
    node.get_parameter = lambda name: _Parameter(values[name])
    node._ensure_tts_ready = lambda: True

    def slow_local_generation(_text):
        release_generation.wait(1.0)
        return None, 16000

    class _PlayClient:
        def wait_for_server(self, _timeout_sec):
            return True

        def send_goal_async(self, goal):
            submitted_goals.append(goal)

    class _Request:
        text = "你好"
        priority = 1

    class _Response:
        pass

    node._generate_local_tts = slow_local_generation
    node._play_client = _PlayClient()
    start = wall_time.monotonic()
    try:
        with patch.object(tts_node, "CLOUD_REQUEST_TIMEOUT_SEC", 0.02):
            response = node._handle_synthesize(_Request(), _Response())
    finally:
        release_generation.set()

    assert wall_time.monotonic() - start < 0.5
    assert response.accepted is False
    assert response.error_message == "TTS 合成超时"
    assert submitted_goals == []


def test_goal_preparation_timeout_returns_before_service_deadline():
    """Synchronous goal conversion would exceed the service deadline."""
    node = _cloud_node()
    release_goal_build = threading.Event()
    submitted_goals = []

    class _PlayClient:
        def wait_for_server(self, timeout_sec):
            assert timeout_sec <= 0.02
            return True

        def send_goal_async(self, goal):
            submitted_goals.append(goal)

    class _Request:
        text = "你好"
        priority = 1

    class _Response:
        pass

    def slow_goal_build(*_args):
        release_goal_build.wait(1.0)
        return object()

    node._play_client = _PlayClient()
    node._synthesize_pcm16 = lambda _text, _deadline: (b"pcm", 16000, "cloud")
    node._build_play_goal_from_pcm = slow_goal_build
    start = wall_time.monotonic()
    try:
        with patch.object(tts_node, "CLOUD_REQUEST_TIMEOUT_SEC", 0.02):
            response = node._handle_synthesize(_Request(), _Response())
    finally:
        release_goal_build.set()

    assert wall_time.monotonic() - start < 0.5
    assert response.accepted is False
    assert response.error_message == "TTS 合成超时"
    assert submitted_goals == []


def test_local_pcm_conversion_timeout_returns_before_service_deadline():
    """Synchronous local float-to-PCM conversion would exceed the deadline."""
    node = _cloud_node()
    release_conversion = threading.Event()
    submitted_goals = []
    values = {"tts_backend": "local"}
    node.get_parameter = lambda name: _Parameter(values[name])
    node._ensure_tts_ready = lambda: True
    node._generate_local_tts = lambda _text: (object(), 16000)

    def slow_conversion(_samples):
        release_conversion.wait(1.0)
        return b"pcm"

    class _PlayClient:
        def wait_for_server(self, _timeout_sec):
            return True

        def send_goal_async(self, goal):
            submitted_goals.append(goal)

    class _Request:
        text = "你好"
        priority = 1

    class _Response:
        pass

    node._float32_to_pcm16 = slow_conversion
    node._play_client = _PlayClient()
    start = wall_time.monotonic()
    try:
        with patch.object(tts_node, "CLOUD_REQUEST_TIMEOUT_SEC", 0.02):
            response = node._handle_synthesize(_Request(), _Response())
    finally:
        release_conversion.set()

    assert wall_time.monotonic() - start < 0.5
    assert response.accepted is False
    assert response.error_message == "TTS 合成超时"
    assert submitted_goals == []
