"""语音子系统拆分后的基础节点栈启动文件。"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _resolve_pkg_base_dir() -> Path:
    """定位含 model/voice_models 的 voice_assistant 包目录（避免误 import 其他工作区）。"""
    share = Path(get_package_share_directory("voice_assistant"))
    install_prefix = share.parent.parent
    candidates = [
        install_prefix / "lib" / "python3.10" / "site-packages" / "voice_assistant",
        install_prefix.parent / "src" / "voice_assistant" / "voice_assistant",
        install_prefix / "lib" / "voice_assistant" / "voice_assistant",
    ]
    for candidate in candidates:
        resolved = candidate.resolve()
        if (resolved / "model" / "voice_models").is_dir():
            return resolved
    try:
        import voice_assistant

        fallback = Path(voice_assistant.__file__).resolve().parent
        if (fallback / "model" / "voice_models").is_dir():
            return fallback
    except ImportError:
        pass
    raise RuntimeError(
        "无法定位 voice_assistant 模型目录，请确认 model/voice_models 已下载"
    )


def _resolve_path(value: str, base_dir: Path) -> str:
    path = Path(value)
    if not value:
        return ""
    if path.is_absolute():
        return str(path)
    return str((base_dir / path).resolve())


def _load_stack_config(config_file: str) -> dict:
    if config_file and Path(config_file).is_file():
        yaml_path = Path(config_file)
    else:
        share = get_package_share_directory("voice_assistant")
        yaml_path = Path(share) / "config" / "voice_assistant.yaml"
    with open(yaml_path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    base_dir = _resolve_pkg_base_dir()
    for key in (
        "asr_model_dir",
        "kws_model_dir",
        "tts_model_dir",
        "vad_model_path",
        "kws_keywords_file",
    ):
        if key in data and isinstance(data[key], str):
            data[key] = _resolve_path(data[key], base_dir)
    return data


def _write_params_file(params: dict) -> str:
    """生成 ROS2 参数文件并返回路径。"""
    fd, path = tempfile.mkstemp(prefix="voice_stack_", suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as handle:
        yaml.dump(
            {"/**": {"ros__parameters": params}},
            handle,
            default_flow_style=False,
            allow_unicode=True,
        )
    return path


def _make_uv_process(
    run_script: str,
    module: str,
    node_name: str,
    params: dict | None = None,
) -> ExecuteProcess:
    """通过 uv 虚拟环境启动 voice_stack 节点。"""
    cmd = [run_script, module, "--ros-args", "-r", f"__node:={node_name}"]
    if params:
        cmd.extend(["--params-file", _write_params_file(params)])
    return ExecuteProcess(cmd=cmd, output="screen", name=node_name)


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    cfg = _load_stack_config(config_file)

    share = get_package_share_directory("voice_assistant")
    run_script = os.path.join(share, "scripts", "run_voice_stack_node.sh")

    sample_rate = str(cfg.get("sample_rate", 16000))
    chunk_size = str(int(16000 * 0.1))
    num_threads = str(cfg.get("num_threads", 5))
    onnx_provider = str(cfg.get("onnx_provider", "cpu"))

    common_voice_params = {
        "sample_rate": int(sample_rate),
        "num_threads": int(num_threads),
        "onnx_provider": onnx_provider,
    }

    return [
        _make_uv_process(
            run_script,
            "voice_audio_capture.capture_node",
            "voice_audio_capture",
            {
                "sample_rate": int(sample_rate),
                "chunk_size": int(chunk_size),
                "input_device_hint": cfg.get("input_device_hint", ""),
            },
        ),
        _make_uv_process(
            run_script,
            "voice_kws.kws_node",
            "voice_kws",
            {
                **common_voice_params,
                "kws_model_dir": cfg.get("kws_model_dir", ""),
                "kws_keywords_file": cfg.get("kws_keywords_file", ""),
                "kws_keywords_score": float(cfg.get("kws_keywords_score", 1.0)),
                "kws_keywords_threshold": float(cfg.get("kws_keywords_threshold", 0.25)),
                "kws_num_trailing_blanks": int(cfg.get("kws_num_trailing_blanks", 1)),
            },
        ),
        _make_uv_process(
            run_script,
            "voice_audio_process.process_node",
            "voice_audio_process",
            {
                "sample_rate": int(sample_rate),
                "vad_model_path": cfg.get("vad_model_path", ""),
                "vad_threshold": float(cfg.get("vad_threshold", 0.5)),
                "vad_min_silence_duration": float(cfg.get("vad_min_silence_duration", 0.25)),
                "vad_min_speech_duration": float(cfg.get("vad_min_speech_duration", 0.25)),
            },
        ),
        _make_uv_process(
            run_script,
            "voice_asr.asr_node",
            "voice_asr",
            {
                **common_voice_params,
                "asr_model_dir": cfg.get("asr_model_dir", ""),
                "asr_backend": cfg.get("asr_backend", "local"),
                "cloud_asr_fallback_to_local": bool(cfg.get("cloud_asr_fallback_to_local", True)),
                "cloud_asr_strategy": cfg.get("cloud_asr_strategy", "streaming"),
                "iflytek_iat_app_id": os.environ.get(
                    "XFYUN_IAT_APPID", os.environ.get("XFYUN_APPID", "")
                ),
                "iflytek_iat_api_key": os.environ.get(
                    "XFYUN_IAT_API_KEY", os.environ.get("XFYUN_API_KEY", "")
                ),
                "iflytek_iat_api_secret": os.environ.get(
                    "XFYUN_IAT_API_SECRET", os.environ.get("XFYUN_API_SECRET", "")
                ),
                "iflytek_iat_language": cfg.get("iflytek_iat_language", "zh_cn"),
                "iflytek_iat_domain": cfg.get("iflytek_iat_domain", "iat"),
                "iflytek_iat_accent": cfg.get("iflytek_iat_accent", "mandarin"),
                "iflytek_iat_eos_ms": int(cfg.get("iflytek_iat_eos_ms", 1800)),
                "iflytek_iat_ptt": int(cfg.get("iflytek_iat_ptt", 1)),
                "iflytek_iat_audio_format": cfg.get(
                    "iflytek_iat_audio_format", "audio/L16;rate=16000"
                ),
                "iflytek_iat_encoding": cfg.get("iflytek_iat_encoding", "raw"),
            },
        ),
        _make_uv_process(
            run_script,
            "voice_tts.tts_node",
            "voice_tts",
            {
                "tts_model_dir": cfg.get("tts_model_dir", ""),
                "tts_speaker_id": int(cfg.get("tts_speaker_id", 0)),
                "tts_speed": float(cfg.get("tts_speed", 1.0)),
                "tts_volume": float(cfg.get("tts_volume", 1.0)),
                "tts_max_chars_per_chunk": int(cfg.get("tts_max_chars_per_chunk", 80)),
                "tts_sentence_pause": float(cfg.get("tts_sentence_pause", 0.40)),
                "tts_clause_pause": float(cfg.get("tts_clause_pause", 0.15)),
                "num_threads": int(num_threads),
                "onnx_provider": onnx_provider,
            },
        ),
        _make_uv_process(
            run_script,
            "voice_playback.playback_node",
            "voice_playback",
            {
                "output_device_hint": cfg.get("output_device_hint", ""),
            },
        ),
        _make_uv_process(
            run_script,
            "voice_volume.volume_node",
            "voice_volume",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("voice_assistant")
    default_config = os.path.join(share, "config", "voice_assistant.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="voice_assistant.yaml 路径（用于 voice_stack 参数透传）",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
