"""Tests for host audio device configuration."""

import subprocess
import sys
from types import SimpleNamespace

sys.modules.setdefault("sounddevice", SimpleNamespace())

from voice_audio_capture import audio_devices


def _pulse_sounddevice():
    devices = [
        {
            "name": "pulse",
            "max_input_channels": 32,
            "max_output_channels": 32,
        }
    ]

    def query_devices(index=None):
        return devices if index is None else devices[index]

    return SimpleNamespace(
        default=SimpleNamespace(device=(0, 0)),
        query_devices=query_devices,
    )


def test_configure_audio_devices_sets_requested_output_port(monkeypatch):
    """The configured PulseAudio port is selected before playback."""
    calls = []
    monkeypatch.setattr(audio_devices, "sd", _pulse_sounddevice())
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda command, **kwargs: calls.append((command, kwargs)),
    )

    audio_devices.configure_audio_devices(
        output_port="analog-output-headphones",
        configure_input=False,
    )

    assert calls == [
        (
            [
                "pactl",
                "set-sink-port",
                "@DEFAULT_SINK@",
                "analog-output-headphones",
            ],
            {
                "check": True,
                "capture_output": True,
                "text": True,
                "timeout": 3.0,
            },
        )
    ]


def test_configure_audio_devices_continues_when_port_switch_fails(monkeypatch):
    """A missing PulseAudio port does not prevent device selection."""
    warnings = []
    info = []
    monkeypatch.setattr(audio_devices, "sd", _pulse_sounddevice())

    def fail_port_switch(command, **kwargs):
        del kwargs
        raise subprocess.CalledProcessError(1, command, stderr="No such port")

    monkeypatch.setattr(subprocess, "run", fail_port_switch)

    audio_devices.configure_audio_devices(
        output_port="analog-output-headphones",
        configure_input=False,
        log_info=info.append,
        log_warning=warnings.append,
    )

    assert warnings == [
        "[Audio] 切换输出端口 analog-output-headphones 失败: No such port"
    ]
    assert info == ["[Audio] 输出设备: pulse"]


def test_configure_audio_devices_skips_empty_output_port(monkeypatch):
    """An omitted port keeps the existing device-only behavior."""
    info = []
    monkeypatch.setattr(audio_devices, "sd", _pulse_sounddevice())

    def unexpected_port_switch(*args, **kwargs):
        del args, kwargs
        raise AssertionError("pactl must not run without an output port")

    monkeypatch.setattr(subprocess, "run", unexpected_port_switch)

    audio_devices.configure_audio_devices(
        output_port="",
        configure_input=False,
        log_info=info.append,
    )

    assert info == ["[Audio] 输出设备: pulse"]
