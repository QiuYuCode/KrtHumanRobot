"""LLM 对话动作节点 (支持多 provider)"""

from __future__ import annotations

import concurrent.futures
import os
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from voice_assistant.config import RobotConfig


_PROXY_ENV_KEYS = (
    "HTTP_PROXY",
    "HTTPS_PROXY",
    "ALL_PROXY",
    "http_proxy",
    "https_proxy",
    "all_proxy",
)


class _without_proxy_env:
    """临时屏蔽代理环境变量，避免本地 Ollama 被无效 socks:// 代理影响。"""

    def __enter__(self):
        self._saved = {key: os.environ.get(key) for key in _PROXY_ENV_KEYS}
        for key in _PROXY_ENV_KEYS:
            os.environ.pop(key, None)

    def __exit__(self, exc_type, exc, tb):
        del exc_type, exc, tb
        for key, value in self._saved.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


def _create_llm(config: RobotConfig):
    """根据 config.llm_provider 创建对应的 LangChain Chat Model 实例。"""
    provider = config.llm_provider.lower()

    if provider == "ollama":
        with _without_proxy_env():
            from langchain_ollama import ChatOllama

            return ChatOllama(
                model=config.llm_model,
                base_url=config.llm_base_url,
                client_kwargs={"trust_env": False},
                sync_client_kwargs={"trust_env": False},
                async_client_kwargs={"trust_env": False},
            )

    elif provider == "openai":
        from langchain_openai import ChatOpenAI

        base_url = config.llm_base_url
        if not base_url or "localhost" in base_url or "127.0.0.1" in base_url:
            base_url = None

        return ChatOpenAI(
            model=config.llm_model,
            api_key=config.llm_api_key,
            base_url=base_url,
        )

    elif provider == "deepseek":
        from langchain_openai import ChatOpenAI

        base_url = config.llm_base_url
        if not base_url or "localhost" in base_url or "127.0.0.1" in base_url:
            base_url = "https://api.deepseek.com"

        return ChatOpenAI(
            model=config.llm_model,
            api_key=config.llm_api_key,
            base_url=base_url,
        )

    elif provider == "anthropic":
        from langchain_anthropic import ChatAnthropic

        return ChatAnthropic(
            model=config.llm_model,
            api_key=config.llm_api_key,
        )

    else:
        raise ValueError(f"不支持的 LLM provider: {provider}")


class LLMDialogAction(Behaviour):
    """
    LLM 自由对话 (fallback)。

    当 intent == "chat" 时执行 (所有关键词 action 都 FAILURE 后)。
    通过 llm_provider 配置切换本地 Ollama / 在线大模型 (OpenAI, DeepSeek, Anthropic 等)，
    维护对话历史实现多轮会话。
    """

    def __init__(self, name: str, config: RobotConfig):
        super().__init__(name)
        self.config = config
        self.conversation_history: list = []
        self.llm = None

        self.blackboard = self.attach_blackboard_client(
            name="LLMDialogAction", namespace="dialog"
        )
        self.blackboard.register_key(
            key="intent", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="user_command", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="response_text", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        """延迟初始化 LLM 连接"""
        try:
            self.llm = _create_llm(self.config)
            self.logger.info(
                f"LLM 已连接: provider={self.config.llm_provider}, "
                f"model={self.config.llm_model}"
            )
        except ImportError as e:
            self.logger.warning(
                f"LLM 依赖未安装 ({self.config.llm_provider}): {e}。"
                "请根据 provider 安装对应包，例如: "
                "uv add langchain-ollama / langchain-openai / langchain-anthropic"
            )
        except Exception as e:
            self.logger.warning(f"LLM 初始化失败: {e}")

    def update(self):
        if self.blackboard.intent != "chat":
            return Status.FAILURE

        if self.llm is None:
            self.blackboard.response_text = "大模型未就绪，请稍后再试。"
            return Status.SUCCESS

        command = self.blackboard.user_command
        self.logger.info(f"LLM 对话: {command}")

        try:
            from langchain_core.messages import HumanMessage, SystemMessage

            messages = [
                SystemMessage(content=self.config.llm_system_prompt),
                *self.conversation_history,
                HumanMessage(content=command),
            ]

            with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
                future = executor.submit(self.llm.invoke, messages)
                response = future.result(timeout=self.config.llm_request_timeout)

            # 更新对话历史
            self.conversation_history.append(HumanMessage(content=command))
            self.conversation_history.append(response)

            # 限制历史长度 (每轮 2 条: Human + AI)
            max_msgs = self.config.llm_max_history * 2
            if len(self.conversation_history) > max_msgs:
                self.conversation_history = self.conversation_history[-max_msgs:]

            self.blackboard.response_text = response.content

        except concurrent.futures.TimeoutError:
            self.logger.error(
                f"LLM 调用超时 ({self.config.llm_request_timeout}s): "
                f"provider={self.config.llm_provider}, model={self.config.llm_model}"
            )
            self.blackboard.response_text = "我刚才走神了，麻烦您再说一次。"
            # 某些 provider 在超时后连接可能进入异常状态，重建实例以提高后续成功率
            try:
                self.llm = _create_llm(self.config)
            except Exception as rebuild_err:
                self.logger.warning(f"LLM 重建失败: {rebuild_err}")
        except Exception as e:
            self.logger.error(f"LLM 调用失败: {e}")
            self.blackboard.response_text = "抱歉，我暂时无法回答。"

        return Status.SUCCESS
