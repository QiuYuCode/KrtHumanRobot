from pathlib import Path

import pytest

from krt_human_robot.config import load_config


def test_ollama_disables_reasoning_for_realtime_actions():
    pytest.importorskip("langchain_ollama")
    from krt_human_robot.behaviors.core.actions.llm_dialog import _create_chat_model

    model = _create_chat_model(
        provider="ollama",
        model="qwen3.5:0.8b",
        base_url="http://localhost:11434",
    )

    assert model.reasoning is False


def test_target_config_uses_cloud_first_with_jetson_fallbacks():
    config_path = Path(__file__).parents[1] / "config" / "krt_human_robot.yaml"

    config = load_config(config_path)

    assert config.llm_provider != "ollama"
    assert config.vlm_provider != "ollama"
    assert config.cloud_llm_fallback_to_local is True
    assert config.cloud_vlm_fallback_to_local is True
    assert config.local_llm_model == "qwen2.5:0.5b"
    assert config.local_vlm_model == "qwen3.5:0.8b"
    assert config.local_llm_base_url == "http://10.168.1.101:11434"
    assert config.local_vlm_base_url == "http://10.168.1.101:11434"
