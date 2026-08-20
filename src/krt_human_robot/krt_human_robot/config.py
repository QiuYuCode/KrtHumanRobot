"""
KrtHumanRobot 核心配置文件

所有可配置参数集中在 RobotConfig dataclass 中。
运行时配置从 config.yaml 加载；敏感字段（API Key 等）始终从环境变量读取。
"""
from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

try:
    from dotenv import load_dotenv
except ImportError:
    load_dotenv = None

try:
    from loguru import logger
except ImportError:
    logger = logging.getLogger(__name__)


def _read_env(*keys: str) -> str:
    """按顺序读取环境变量，并做去空白/去包裹引号清洗。"""
    if load_dotenv is not None:
        load_dotenv()
    for key in keys:
        value = os.getenv(key)
        if value is None:
            continue
        cleaned = value.strip().strip('"').strip("'").strip()
        if cleaned:
            return cleaned
    return ""


# Python 包目录（logs/ 相对此路径）
base_dir = Path(__file__).parent


def _default_config_yaml() -> Path:
    """解析默认 YAML：环境变量 > install share > 源码 config/。"""
    override = os.getenv("KRT_HUMAN_ROBOT_CONFIG", "").strip()
    if override:
        return Path(override).expanduser()
    try:
        from ament_index_python.packages import get_package_share_directory

        share_yaml = (
            Path(get_package_share_directory("krt_human_robot"))
            / "config"
            / "krt_human_robot.yaml"
        )
        if share_yaml.is_file():
            return share_yaml
    except Exception:
        pass
    src_yaml = Path(__file__).resolve().parent.parent / "config" / "krt_human_robot.yaml"
    return src_yaml


# ============================================================================
# 模型路径 (根据实际下载位置修改)
# ============================================================================

MODELS_BASE = base_dir / "model" / "voice_models"

# sherpa-onnx 流式 ASR
ASR_DIR = MODELS_BASE / "sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20"

# sherpa-onnx 关键词检测
KWS_DIR = MODELS_BASE / "sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20"

# sherpa-onnx VITS TTS
TTS_DIR = MODELS_BASE / "vits-zh-hf-fanchen-C"

# Silero VAD 模型
VAD_DIR = MODELS_BASE / "silero_vad.onnx"


# ============================================================================
# 音频常量
# ============================================================================

SAMPLE_RATE = 16000
CHUNK_SIZE = int(SAMPLE_RATE * 0.1)  # 0.1 秒 = 1600 采样点


# ============================================================================
# 配置 dataclass
# ============================================================================

@dataclass
class RobotConfig:
    """机器人语音助手的全部可配置参数"""

    # --- 模型路径 ---
    # 相对路径以项目根目录为基准；绝对路径直接使用
    asr_model_dir: str = str(ASR_DIR)
    kws_model_dir: str = str(KWS_DIR)
    tts_model_dir: str = str(TTS_DIR)
    vad_model_path: str = str(VAD_DIR)

    # --- 唤醒模式 ---
    # "software": sherpa-onnx KWS 软件唤醒（唯一支持的模式）
    wake_mode: str = "software"

    # --- 唤醒词 (KWS) ---
    # 替换 keywords_file 路径即可更换唤醒词
    kws_keywords_file: str = str(KWS_DIR / "keywords.txt")
    kws_keywords_score: float = 1.0
    kws_keywords_threshold: float = 0.25
    kws_num_trailing_blanks: int = 1

    # --- ASR 后端 ---
    # "local": 本地 sherpa-onnx 流式识别
    # "iflytek_cloud": 讯飞云识别
    asr_backend: str = "local"
    asr_streaming_enabled: bool = True
    asr_stream_timeout: float = 12.0
    asr_stream_preroll_seconds: float = 0.5
    # "streaming": 逐帧上传(40ms)；"endpoint_once": 端点后一次性上传
    cloud_asr_strategy: str = "streaming"
    cloud_asr_fallback_to_local: bool = True

    # --- 讯飞云 ASR 配置 (/v2/iat) ---
    iflytek_iat_app_id: str = _read_env("XFYUN_IAT_APPID", "XFYUN_APPID")
    iflytek_iat_api_key: str = _read_env("XFYUN_IAT_API_KEY", "XFYUN_API_KEY")
    iflytek_iat_api_secret: str = _read_env("XFYUN_IAT_API_SECRET", "XFYUN_API_SECRET")
    iflytek_iat_language: str = "zh_cn"
    iflytek_iat_domain: str = "iat"
    iflytek_iat_accent: str = "mandarin"
    iflytek_iat_eos_ms: int = 1800
    iflytek_iat_ptt: int = 1
    iflytek_iat_audio_format: str = "audio/L16;rate=16000"
    iflytek_iat_encoding: str = "raw"
    cloud_asr_preroll_seconds: float = 0.3

    # --- TTS 音色 ---
    # fanchen-C 单说话人模型，sid 固定为 0
    tts_speaker_id: int = 0
    tts_speed: float = 1.2
    tts_volume: float = 1.0  # 本地 TTS 音量增益，1.0=原始，>1.0 放大，超出 [-1,1] 自动截断
    tts_max_chars_per_chunk: int = 80
    tts_sentence_pause: float = 0.40
    tts_clause_pause: float = 0.15
    tts_backend: str = "iflytek_cloud"  # "local" | "iflytek_cloud" | "mimo_cloud"
    cloud_tts_fallback_to_local: bool = True

    # --- 小米 MiMo 云 TTS (mimo-v2.5-tts, /v1/chat/completions) ---
    mimo_tts_model: str = "mimo-v2.5-tts"
    mimo_tts_voice: str = "苏打"
    mimo_tts_style: str = (
        "用成年男性、平稳、略带机械与金属质感的机器人播报口吻朗读，不要夸张情绪。"
    )
    mimo_tts_audio_format: str = "wav"
    mimo_tts_base_url: str = ""
    mimo_tts_api_key: str = ""
    mimo_tts_timeout: float = 30.0

    # --- 讯飞云 TTS 配置 (/v2/tts) ---
    iflytek_tts_app_id: str = _read_env("XFYUN_TTS_APPID", "XFYUN_APPID")
    iflytek_tts_api_key: str = _read_env("XFYUN_TTS_API_KEY", "XFYUN_API_KEY")
    iflytek_tts_api_secret: str = _read_env("XFYUN_TTS_API_SECRET", "XFYUN_API_SECRET")
    iflytek_tts_vcn: str = "x4_yezi"
    iflytek_tts_aue: str = "raw"
    iflytek_tts_auf: str = "audio/L16;rate=16000"
    iflytek_tts_speed: int = 50
    iflytek_tts_tte: str = "UTF8"
    iflytek_tts_request_text_encoding: str = "utf-8"
    iflytek_tts_max_bytes: int = 8000

    # --- VAD (语音活动检测) ---
    vad_threshold: float = 0.5
    vad_min_silence_duration: float = 0.25
    vad_min_speech_duration: float = 0.25

    # --- 音频设备选择 ---
    # 优先按子串匹配设备名；为空则使用系统默认设备。
    # 例如: "XFM-DP"、"C-Media"、"USB Audio"
    input_device_hint: str = "XFM-DP"
    output_device_hint: str = "C-Media"

    # --- 对话 ---
    dialog_timeout: float = 15.0  # 秒，无活动后超时回到 idle
    continuous_dialog: bool = False  # True 时回复后继续聆听；False 时回复后回到唤醒
    post_tts_listen_delay: float = 0.6  # TTS 播放结束后延迟进入聆听，降低回声误识别
    interrupt_min_speech_seconds: float = 0.0  # TTS 启动后延迟启用打断；0 表示唤醒词命中立即打断

    # --- 相机 ---
    # 三路相机统一管理: head (头部 RealSense) / left_palm / right_palm
    # backend: "local" (本机 USB) | "http" (远端 FastAPI) | "ros" (rclpy 订阅)
    camera_backend: str = "ros"
    default_camera: str = "head"  # describe_scene / take_photo / record_video 默认使用的相机

    # HTTP 后端: 远端 FastAPI 服务基址，URL 形如 {base}/snapshot/{camera_id}
    camera_http_base_url: str = ""          # e.g. http://10.168.1.101:8080
    camera_http_timeout: float = 5.0

    # ROS 后端 (rclpy + cv_bridge)
    camera_ros_node_name: str = "smart_voice_robot_camera_sub"
    camera_ros_transport: str = "raw"
    camera_ros_qos_depth: int = 5
    camera_ros_warmup_seconds: float = 2.0  # 启动时等待首帧的超时
    head_depth_topic: str = "/camera/camera/depth/image_rect_raw"
    head_depth_info_topic: str = "/camera/camera/depth/camera_info"
    head_aligned_depth_topic: str = "/camera/camera/aligned_depth_to_color/image_raw"
    head_aligned_depth_info_topic: str = (
        "/camera/camera/aligned_depth_to_color/camera_info"
    )

    # 每台相机的规格; YAML 中通过 cameras.{id}.{field} 覆盖
    cameras: dict[str, dict[str, Any]] = field(default_factory=lambda: {
        "head": {
            "index": 4,
            "width": 640,
            "height": 480,
            "ros_topic": "/camera/camera/color/image_raw",
            "ros_compressed_topic": "/camera/camera/color/image_jpeg",
            "record_fps": 30.0,
        },
        "left_palm": {
            "index": 0,
            "width": 640,
            "height": 480,
            "ros_topic": "/left_gripper/image_raw",
            "record_fps": 30.0,
        },
        "right_palm": {
            "index": 2,
            "width": 640,
            "height": 480,
            "ros_topic": "/right_gripper/image_raw",
            "record_fps": 30.0,
        },
    })

    # 通用相机参数
    camera_save_dir: str = "captures"           # 照片/视频保存目录 (相对于项目根目录)
    camera_record_seconds: float = 10.0         # 视频录制时长 (秒)

    # --- 夹爪 (DexHand021S) ---
    gripper_enabled: bool = True
    # 适配器类型: "zlg_mini" | "zlg_200u" | "lys_mini"
    gripper_adapter_type: str = "zlg_mini"
    gripper_finger_ids: list[int] = field(default_factory=lambda: [0x01, 0x02, 0x03])
    gripper_control_mode: int = 0x55
    # 合拢/张开可用不同速度；数值越小动作越慢（与 SDK move_finger 的 motion_velocity 一致）
    gripper_default_speed: int = 500
    gripper_speed_close: int = 280
    gripper_speed_open: int = 240
    gripper_open_value: int = 0
    gripper_close_value: int = 1000
    gripper_shake_cycles: int = 1
    gripper_shake_pause_close: float = 0.55
    gripper_shake_pause_open: float = 0.55
    gripper_second_hand_init_delay: float = 0.5
    gripper_inter_finger_delay: float = 0.04
    gripper_post_reset_sleep: float = 0.45
    gripper_exec_delay_ms: int = 10
    gripper_set_safe_current: bool = True
    gripper_safe_current: int = 250

    left_gripper: dict[str, Any] = field(default_factory=lambda: {
        "adapter_index": 1,
        "device_id": 0x01,
        "has_pressure_sensor": False,
    })
    right_gripper: dict[str, Any] = field(default_factory=lambda: {
        "adapter_index": 0,
        "device_id": 0x02,
        "has_pressure_sensor": True,
    })

    # --- LLM ---
    # provider: "ollama" | "openai" | "deepseek" | "anthropic"
    llm_provider: str = "openai"
    llm_model: str = "mimo-v2.5"
    llm_base_url: str = "https://token-plan-cn.xiaomimimo.com/v1"
    llm_api_key: str = _read_env("LLM_API_KEY")  # 在线模型的 API Key (ollama 不需要)
    llm_system_prompt: str = (
        "你是一个机器人语音助手。回答要求：\n"
        "1. 用简短自然的口语化中文回答，像人在说话一样。\n"
        "2. 善用逗号断句，避免一口气说完长句子。\n"
        "3. 禁止输出 emoji、表情符号、括号注释、markdown 格式。\n"
        "4. 禁止输出列表编号，改用自然语言衔接，比如用首先、然后、最后。\n"
        "5. 数字用中文读法，比如三百二十，而非320。\n"
    )
    llm_max_history: int = 10  # 保留最近 N 轮对话历史
    llm_request_timeout: float = 12.0  # 单次 LLM 请求超时(秒)
    llm_max_retries: int = 1
    cloud_llm_fallback_to_local: bool = True
    local_llm_provider: str = "ollama"
    local_llm_model: str = "qwen2.5:0.5b"
    local_llm_base_url: str = "http://localhost:11434"

    # --- VLM (视觉语言模型) ---
    # provider: "ollama" | "openai" | "deepseek" | "anthropic"
    vlm_provider: str = "openai"
    vlm_model: str = "mimo-v2.5"
    vlm_base_url: str = "https://token-plan-cn.xiaomimimo.com/v1"
    vlm_api_key: str = _read_env("VLM_API_KEY", "LLM_API_KEY")
    vlm_system_prompt: str = (
        "你是一个机器人的视觉系统。根据图片内容，用简短自然的中文描述你看到的场景。\n"
        "用第一人称：我看到了。\n"
        "禁止输出 emoji、表情符号、markdown 格式。\n"
    )
    vlm_request_timeout: float = 30.0
    vlm_max_retries: int = 1
    cloud_vlm_fallback_to_local: bool = True
    local_vlm_provider: str = "ollama"
    local_vlm_model: str = "qwen3.5:0.8b"
    local_vlm_base_url: str = "http://localhost:11434"

    # --- 任务规划器 (LLM Function Calling) ---
    # True 时使用 LLM 解析多步指令，False 时使用关键词匹配（原有行为）
    use_llm_planner: bool = False
    planner_system_prompt: str = (
        "你是一个机器人任务规划器。根据用户的自然语言指令，调用合适的工具来完成任务。\n"
        "规则：\n"
        "1. 如果用户给出多个指令，请按顺序调用多个工具。\n"
        "2. 如果用户只是闲聊或提问，直接用简短中文回复，不要调用任何工具。\n"
        "3. 夹爪控制必须显式指定 hand=left 或 hand=right，不允许省略。\n"
        "4. 不要输出 emoji 表情和其他任何表情符号。\n"
    )

    # --- 意图关键词映射 ---
    # key: 意图名称, value: 触发该意图的关键词列表
    intent_patterns: dict[str, list[str]] = field(default_factory=lambda: {
        "describe_left_palm": [
            "描述一下你左手", "描述一下左手", "左手看到了什么", "左手看到什么",
            "看一看左手", "看一下左手", "看看左手", "左手摄像头", "用你的左手摄像头", "左爪",
            "左夹爪", "左掌心", "左手拿", "看看左",
        ],
        "describe_right_palm": [
            "描述一下你右手", "描述一下右手", "右手看到了什么", "右手看到什么",
            "看一看右手", "看一下右手", "看看右手", "右手摄像头", "用你的右手摄像头", "右爪",
            "右夹爪", "右掌心", "右手拿", "看看右",
        ],
        "describe_scene": ["看一下", "看看", "这是什么", "前面有什么", "描述一下", "看到了什么"],
        "take_photo": ["拍照", "拍张照", "拍个照", "拍一张", "照片"],
        "record_video": ["录像", "录制视频", "录视频", "录一段", "摄像", "拍个视频", "拍视频"],
        "gripper_control": [
            "左手张开", "右手张开", "左手握手", "右手握手", "动动左手", "动动右手", "左手", "右手", "夹爪"
        ],
        "robot_arm": [
            "机械臂", "抓取", "拿起", "放下",
            "进入示教", "开始示教", "开启示教",
            "退出示教", "结束示教", "停止示教",
            "示教动作", "动作组", "回放轨迹",
            "挥手", "打招呼", "你好",
        ],
        "routine": ["唱跳表演", "开始表演", "执行流程", "运行流程", "表演"],
        "introduce_self": ["介绍一下你自己", "介绍一下机器人", "介绍一下自己", "你是谁", "你能做什么"],
        "introduce_krt": ["介绍一下湖南科瑞特", "介绍湖南科瑞特"],
        "save_mapping": ["保存建图", "保存地图", "结束建图", "保存电图", "结束电图"],
        "start_mapping": ["开始建图", "启动建图", "建图", "开始电图", "启动电图", "电图"],
        "stop_navigation": ["结束导航", "停止导航", "退出导航", "关闭导航"],
        "start_navigation": ["开始导航", "启动导航", "进入导航模式"],
        "mark_waypoint": ["打点", "保存点位", "记录点位", "标记点位"],
        "loop_cruise": ["循环巡航", "一直巡航"],
        "repeat_cruise": ["巡航一遍", "巡航两遍", "巡航二遍", "巡航三遍", "巡航四遍", "巡航五遍"],
        "stop_cruise": ["停止巡航", "结束巡航", "取消巡航"],
        "continue_waypoint": ["继续巡航", "继续执行", "继续"],
        "remove_waypoint": ["删除点位", "移除点位"],
        "list_waypoints": ["列出点位", "有哪些点位", "查看点位"],
        "start_cruise": ["开始巡航", "启动巡航"],
        "cruise_waypoints": ["巡航指定点", "巡航点位"],
        "navigation": ["导航", "前往", "去", "带我去"],
        "exit": ["没事了", "拜拜", "再见"],
    })

    # --- 机械臂关键词动作组映射 ---
    # 命中后统一转为 run_group 执行；priority 越大优先级越高
    robot_arm_keyword_actions: list[dict[str, Any]] = field(default_factory=lambda: [
        {
            "keywords": ["挥手", "打招呼"],
            "arm_side": "right",
            "group_name": "wave",
            "response_text": "",
            "priority": 100,
        },
        {
            "keywords": ["你好"],
            "arm_side": "right",
            "group_name": "wave",
            "response_text": "你好，很高兴见到你。",
            "priority": 120,
        },
    ])

    # --- 命名流程 ---
    routine_action: str = "/krt_task/run_routine"
    routine_action_timeout_s: float = 120.0
    routine_keyword_actions: list[dict[str, Any]] = field(default_factory=lambda: [
        {
            "keywords": ["唱跳表演", "开始表演", "表演一下"],
            "routine_name": "唱跳表演",
            "response_text": "",
            "priority": 100,
        },
    ])

    # --- 机械臂示教 ---
    robot_arm_enabled: bool = True
    robot_arm_robot: str = "nero"
    robot_arm_comm: str = "can"
    robot_arm_firmware: str = "v111"
    robot_arm_channels: dict[str, str] = field(default_factory=lambda: {
        "left": "can_left",
        "right": "can_right",
    })
    robot_arm_teach_save_dir: str = "captures/arm_teach"
    robot_arm_teach_file_template: str = "{arm}_teach_records.json"
    robot_arm_teach_start_service: str = "/agx_action_group/start_teach"
    robot_arm_teach_stop_service: str = "/agx_action_group/stop_teach"
    robot_arm_teach_service_timeout_s: float = 8.0
    robot_arm_action_group_action: str = "/agx_action_group/run_action_group"
    robot_arm_action_timeout_s: float = 60.0
    robot_arm_action_repeat_count: int = 1
    robot_arm_sample_interval_s: float = 0.005
    robot_arm_replay_speed_percent: int = 50
    robot_arm_replay_use_timing: bool = True
    robot_arm_replay_min_interval_s: float = 0.02
    robot_arm_replay_send_retries: int = 5
    robot_arm_replay_retry_backoff_s: float = 0.01
    robot_arm_replay_max_seconds: float = 45.0
    robot_arm_replay_max_frames: int = 3000
    robot_arm_replay_min_delta_rad: float = 0.002
    robot_arm_joint_limits: list[list[float]] = field(default_factory=lambda: [
        [-3.2, 3.2],
        [-2.8, 2.8],
        [-2.8, 2.8],
        [-1.012291, 2.146755],
        [-3.2, 3.2],
        [-3.2, 3.2],
        [-3.2, 3.2],
    ])
    robot_arm_enable_timeout: float = 10.0

    # 掌心视觉前预置姿态：先回放示教 JSON（robot_arm_teach_file_template）中指定动作组再拍照+VLM
    describe_left_palm_preset_group: str | None = None
    describe_left_palm_preset_arm_side: str = "left"
    describe_right_palm_preset_group: str | None = None
    describe_right_palm_preset_arm_side: str = "right"
    # 左/右掌心预置：False=后台回放开始后仅等待对应 capture_delay_s 即拍照；True=整段回放完再拍
    describe_left_palm_preset_wait_replay_finish: bool = False
    describe_left_palm_preset_capture_delay_s: float = 2.5
    describe_right_palm_preset_wait_replay_finish: bool = False
    describe_right_palm_preset_capture_delay_s: float = 2.5

    # --- TTS 响应模板 ---
    tts_responses: dict[str, str] = field(default_factory=lambda: {
        "wakeup": "我在，请说。",
        "timeout": "没有听到您的命令，有需要可以再叫我。",
        "gripper_missing_side": "请说明左手还是右手。",
        "introduce_self": (
            "我是一个基于行为树的机器人语音助手，通过唤醒词、语音识别、意图分发、动作执行和语音合成完成交互。"
            "我支持关键词指令和大模型多指令规划，可配合相机、机械臂、导航等模块，完成拍照、录像、场景描述、机械臂控制和自然对话等任务。"
        ),
        "introduce_krt": (
            "湖南科瑞特科技有限公司创立于2004年，是一家以人工智能技术、机器人技术、物联网技术等先进技术为核心，"
            "集产品研究开发、生产制造、市场营销、工程服务及技术培训为一体，为工业企业、科研院所、高校与职校等在人工智能及应用、"
            "智能机器人、工业机器人与智能制造、集成电路设计制造与封测、电子工艺与制造、无人机及应用等泛人工智能领域提供整体解决方案的专业提供商。"
            "公司为国家级高新技术企业、信息化建设重点试点企业、智能制造示范企业、省级专精特新小巨人企业；"
            "公司已通过ISO9001国际质量管理体系认证、ISO14001国际环境管理体系认证、ISO45001职业健康安全管理体系认证、"
            "GB/T 23001与GB/T 23006两化融合管理体系认证；在科技创新方面，公司先后获得80多项专利与软件著作权等自主知识产权，"
            "30多个科技项目获得部级、省市级重大、重点项目认定。公司在全国共设立15个办事处，拥有150家以上合作伙伴、"
            "1400家以上终端客户，经营与服务网络覆盖全国各省市。"
        ),
    })
    interrupt_wakeup_responses: list[str] = field(default_factory=lambda: [
        "怎么了",
        "我在，有什么问题吗",
        "我在呢",
    ])

    # --- 启动提示音 ---
    startup_sound_enabled: bool = True
    startup_sound_text: str = "系统启动完成"

    # --- 系统 ---
    tick_interval: float = 0.05  # 主循环心跳间隔 (秒)
    num_threads: int = 5  # 模型推理线程数
    onnx_provider: str = "cpu"  # ONNX 推理设备: "cuda" | "cpu"
    verbose: bool = True

    # --- 日志 ---
    log_dir: str = "logs"           # 日志文件保存目录
    log_retention: str = "7 days"   # 日志保留时长
    log_level: str = "DEBUG"        # 文件日志级别

    # --- 功能包 ROS 接口预留 ---
    adapters: dict[str, dict[str, Any]] = field(default_factory=lambda: {
        "camera": {
            "enabled": False,
            "semantic_topic_prefix": "/krtHumanRobot/vision",
            "image_topic": "/camera/camera/color/image_raw",
        },
        "lidar": {
            "enabled": False,
            "launch_package": "spark_fast_lio",
            "pointcloud_topic": "/livox/lidar",
        },
        "base": {
            "enabled": False,
            "cmd_vel_topic": "/cmd_vel",
            "odom_topic": "/odom",
        },
        "navigation": {
            "enabled": True,
            "launch_package": "ranger_nav",
            "navigate_action": "/navigate_to_pose",
            "mapping_backend": "fast_lio",
            "mapping_launch_fast_lio": "mapping.launch.py",
            "mapping_launch_spark_sam": "mapping_sam.launch.py",
            "navigation_mode": "3dloc",
            "navigation_launch": "navigation.launch.py",
            "navigation_launch_2d": "navigation.launch.py",
            "navigation_launch_3d": "navigation_3dloc.launch.py",
            "fast_lio_save_service": "/map_save",
            "spark_sam_save_topic": "/km_sam/save_dir",
            "map_dir": "~/maps",
            "fast_lio_pcd": "~/maps/scans.pcd",
            "spark_sam_pcd": "~/maps/ranger/ranger_map.pcd",
            "map_yaml": "~/maps/map.yaml",
            "pcd_map_path": "~/maps/scans.pcd",
            "map_archive_dir": "~/maps",
            "map_timestamp_format": "%Y%m%d_%H%M%S",
            "session_cloud_filename": "cloud.pcd",
            "session_map_prefix": "map",
            "latest_map_yaml": "~/maps/map.yaml",
            "latest_map_pgm": "~/maps/map.pgm",
            "fast_lio_temp_pcd": "~/maps/scans.pcd",
            "spark_sam_sequence_name": "ranger",
            "map_save_wait_s": 30.0,
            "pcd2pgm_lidar_height": 0.30,
            "pcd2pgm_z_min": 0.15,
            "pcd2pgm_z_max": 1.2,
            "pcd2pgm_resolution": 0.05,
            "pcd2pgm_ror_radius": 0.1,
            "pcd2pgm_ror_min_pts": 10,
            "robot_db": "~/maps/krt_robot.db",
            "waypoint_routine_action": "/krt_task/run_routine",
            "waypoint_input_topic": "/input_at_waypoint/input",
            "default_waypoint_wait_ms": 200,
            "auto_convert_pcd": True,
            "mapping_stop_delay_s": 3.0,
        },
        "arm": {
            "enabled": False,
            "action_group_action": "/agx_action_group/run_action_group",
        },
        "hand": {
            "enabled": False,
            "control_action": "/hands_control",
        },
    })

    # --- py_trees_ros 监控 ---
    enable_monitor: bool = True  # 是否启用 tree/blackboard 快照发布


# YAML 配置文件路径
CONFIG_YAML_PATH: Path = _default_config_yaml()

# 不写入 YAML 的敏感字段，始终从环境变量读取
_SECRET_FIELDS: frozenset[str] = frozenset({
    "iflytek_iat_app_id",
    "iflytek_iat_api_key",
    "iflytek_iat_api_secret",
    "iflytek_tts_app_id",
    "iflytek_tts_api_key",
    "iflytek_tts_api_secret",
    "llm_api_key",
    "vlm_api_key",
})

# 相对路径字段：YAML 中若为相对路径，自动解析为相对项目根目录的绝对路径
_PATH_FIELDS: frozenset[str] = frozenset({
    "kws_keywords_file",
    "asr_model_dir",
    "kws_model_dir",
    "tts_model_dir",
    "vad_model_path",
})


def _normalize_robot_arm_keyword_actions(value: Any) -> list[dict[str, Any]]:
    """归一化机械臂关键词映射配置。"""
    if not isinstance(value, list):
        logger.warning("[config] robot_arm_keyword_actions 应为 list，已忽略。")
        return []

    normalized: list[dict[str, Any]] = []
    for idx, item in enumerate(value):
        if not isinstance(item, dict):
            logger.warning(f"[config] robot_arm_keyword_actions[{idx}] 应为 dict，已忽略。")
            continue
        keywords_raw = item.get("keywords", [])
        if isinstance(keywords_raw, str):
            keywords_raw = [keywords_raw]
        if not isinstance(keywords_raw, list):
            logger.warning(
                f"[config] robot_arm_keyword_actions[{idx}].keywords 应为 list[str]，已忽略。"
            )
            continue
        keywords = [str(k).strip() for k in keywords_raw if str(k).strip()]
        arm_side = str(item.get("arm_side", "")).strip().lower()
        group_name = str(item.get("group_name", "")).strip()
        response_text = str(item.get("response_text", "")).strip()
        priority = item.get("priority", 0)
        try:
            priority = int(priority)
        except Exception:
            logger.warning(
                f"[config] robot_arm_keyword_actions[{idx}].priority 非法，已降级为 0。"
            )
            priority = 0

        if not keywords or arm_side not in {"left", "right"} or not group_name:
            logger.warning(
                f"[config] robot_arm_keyword_actions[{idx}] 缺少必要字段，已忽略。"
            )
            continue

        normalized.append({
            "keywords": keywords,
            "arm_side": arm_side,
            "group_name": group_name,
            "response_text": response_text,
            "priority": priority,
        })
    return normalized


def load_config(path: Path | str | None = None) -> RobotConfig:
    """从 YAML 文件加载配置，敏感字段始终从环境变量覆盖。

    Args:
        path: YAML 文件路径，默认为项目根目录下的 config.yaml。

    Returns:
        填充好的 RobotConfig 实例。
    """
    cfg = RobotConfig()
    yaml_path = Path(path) if path is not None else CONFIG_YAML_PATH

    if yaml_path.exists():
        with open(yaml_path, encoding="utf-8") as f:
            data: dict[str, Any] = yaml.safe_load(f) or {}

        # 旧版平铺相机字段 → 迁移到 cameras.head.*
        _LEGACY_CAMERA_MAP = {
            "camera_index": "index",
            "camera_width": "width",
            "camera_height": "height",
            "camera_record_fps": "record_fps",
        }
        legacy_head_overrides: dict[str, Any] = {}
        for legacy_key, new_key in _LEGACY_CAMERA_MAP.items():
            if legacy_key in data:
                legacy_head_overrides[new_key] = data.pop(legacy_key)
                logger.warning(
                    f"[config] {legacy_key!r} 已废弃，已自动映射到 cameras.head.{new_key}；"
                    f"请在 config.yaml 中迁移到新结构。"
                )

        for key, value in data.items():
            if key in _SECRET_FIELDS:
                continue
            if not hasattr(cfg, key):
                logger.warning(f"[config] 忽略未知配置项: {key!r}")
                continue
            value = _prepare_config_value(key, value, cfg)
            setattr(cfg, key, value)

        if legacy_head_overrides:
            head_spec = dict(cfg.cameras.get("head", {}))
            head_spec.update(legacy_head_overrides)
            cfg.cameras["head"] = head_spec

        logger.debug(f"[config] 已从 {yaml_path} 加载配置")
    else:
        logger.debug(f"[config] 配置文件不存在，使用内置默认值: {yaml_path}")

    # 敏感字段始终从环境变量读取（覆盖 YAML 或 dataclass 默认值）
    cfg.iflytek_iat_app_id = _read_env("XFYUN_IAT_APPID", "XFYUN_APPID")
    cfg.iflytek_iat_api_key = _read_env("XFYUN_IAT_API_KEY", "XFYUN_API_KEY")
    cfg.iflytek_iat_api_secret = _read_env("XFYUN_IAT_API_SECRET", "XFYUN_API_SECRET")
    cfg.iflytek_tts_app_id = _read_env("XFYUN_TTS_APPID", "XFYUN_APPID")
    cfg.iflytek_tts_api_key = _read_env("XFYUN_TTS_API_KEY", "XFYUN_API_KEY")
    cfg.iflytek_tts_api_secret = _read_env("XFYUN_TTS_API_SECRET", "XFYUN_API_SECRET")
    cfg.llm_api_key = _read_env("LLM_API_KEY")
    cfg.vlm_api_key = _read_env("VLM_API_KEY", "LLM_API_KEY")

    return cfg


def _prepare_config_value(key: str, value: Any, cfg: RobotConfig) -> Any:
    """将 YAML/JSON 中的值转换为 RobotConfig 字段类型（与 load_config 一致）。"""
    if key == "cameras" and isinstance(value, dict):
        merged = {cid: dict(spec) for cid, spec in cfg.cameras.items()}
        for cid, spec in value.items():
            if not isinstance(spec, dict):
                logger.warning(f"[config] cameras.{cid} 应为 dict，忽略: {spec!r}")
                continue
            merged.setdefault(cid, {}).update(spec)
        return merged
    if key == "robot_arm_keyword_actions":
        return _normalize_robot_arm_keyword_actions(value)
    if key in _PATH_FIELDS and isinstance(value, str) and not Path(value).is_absolute():
        return str(base_dir / value)
    return value


# 全局默认配置实例（从 config.yaml 加载，文件不存在时使用内置默认值）
default_config = load_config()
