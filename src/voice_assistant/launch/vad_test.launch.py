"""Minimal microphone capture and voice activity detection stack."""

from voice_assistant.unit_launch import make_unit_launch


def generate_launch_description():
    return make_unit_launch({"capture", "vad"})
