from krt_human_robot.behaviors.core.actions.llm_dialog import _create_chat_model


def test_ollama_disables_reasoning_for_realtime_actions():
    model = _create_chat_model(
        provider="ollama",
        model="qwen3.5:0.8b",
        base_url="http://localhost:11434",
    )

    assert model.reasoning is False
