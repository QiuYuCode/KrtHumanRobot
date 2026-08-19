"""Deployment contracts for x86 audio device selection."""

from pathlib import Path

import yaml


CONFIG = Path(__file__).parents[1] / "config" / "voice_assistant.yaml"


def test_x86_audio_hints_match_deployed_devices():
    """Voice capture and playback use the migrated x86 device names."""
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))

    assert config["input_device_hint"] == "XFM-DP"
    assert config["output_device_hint"] == "headphones"
    assert config["output_port"] == "analog-output-headphones"


def test_kws_uses_one_inference_thread():
    """KWS must not create enough ONNX workers to saturate the x86 CPU."""
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    launch_source = (
        CONFIG.parents[1] / "launch" / "voice_stack.launch.py"
    ).read_text(encoding="utf-8")

    assert config["kws_num_threads"] == 1
    assert '"num_threads": int(cfg.get("kws_num_threads", 1))' in launch_source
