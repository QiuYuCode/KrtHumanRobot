"""Deployment contracts for x86 audio device selection."""

from pathlib import Path

import yaml


CONFIG = Path(__file__).parents[1] / "config" / "voice_assistant.yaml"


def test_x86_audio_hints_match_deployed_devices():
    """Voice capture and playback use the migrated x86 device names."""
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))

    assert config["input_device_hint"] == "XFM-DP"
    assert config["output_device_hint"] == "HDA Intel PCH"
