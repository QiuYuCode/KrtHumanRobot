"""LLM 对话动作节点 (支持多 provider)"""

from __future__ import annotations

import concurrent.futures
import os
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from krt_human_robot.config import RobotConfig

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


def _is_local_base_url(base_url: str | None) -> bool:
    """判断 base_url 是否指向本机服务。"""
    return bool(base_url) and (
        "localhost" in base_url or "127.0.0.1" in base_url
    )


def _create_chat_model(
    *,
    provider: str,
    model: str,
    base_url: str = "",
    api_key: str = "",
    timeout: float | None = None,
    max_retries: int | None = None,
):
    """创建 LangChain Chat Model，支持 OpenAI-compatible 云端和本地 Ollama。"""
    provider = provider.lower()

    if provider == "ollama":
        with _without_proxy_env():
            from langchain_ollama import ChatOllama

            return ChatOllama(
                model=model,
                base_url=base_url,
                client_kwargs={"trust_env": False},
                sync_client_kwargs={"trust_env": False},
                async_client_kwargs={"trust_env": False},
            )

    elif provider == "openai":
        import httpx
        from langchain_openai import ChatOpenAI

        resolved_base_url = (
            None if _is_local_base_url(base_url) else (base_url or None)
        )

        return ChatOpenAI(
            model=model,
            api_key=api_key,
            base_url=resolved_base_url,
            timeout=timeout,
            max_retries=max_retries,
            http_client=httpx.Client(trust_env=False),
            http_async_client=httpx.AsyncClient(trust_env=False),
        )

    elif provider == "deepseek":
        import httpx
        from langchain_openai import ChatOpenAI

        resolved_base_url = base_url
        if not resolved_base_url or _is_local_base_url(resolved_base_url):
            resolved_base_url = "https://api.deepseek.com"

        return ChatOpenAI(
            model=model,
            api_key=api_key,
            base_url=resolved_base_url,
            timeout=timeout,
            max_retries=max_retries,
            http_client=httpx.Client(trust_env=False),
            http_async_client=httpx.AsyncClient(trust_env=False),
        )

    elif provider == "anthropic":
        from langchain_anthropic import ChatAnthropic

        kwargs = {"model": model, "api_key": api_key}
        if timeout is not None:
            kwargs["timeout"] = timeout
        if max_retries is not None:
            kwargs["max_retries"] = max_retries
        return ChatAnthropic(**kwargs)

    else:
        raise ValueError(f"不支持的 LLM provider: {provider}")


def _create_llm(config: RobotConfig):
    """根据 config.llm_provider 创建对应的 LangChain Chat Model 实例。"""
    return _create_chat_model(
        provider=config.llm_provider,
        model=config.llm_model,
        base_url=config.llm_base_url,
        api_key=config.llm_api_key,
        timeout=config.llm_request_timeout,
        max_retries=config.llm_max_retries,
    )


def _create_local_llm(config: RobotConfig):
    """创建本地 LLM 回退模型。"""
    return _create_chat_model(
        provider=config.local_llm_provider,
        model=config.local_llm_model,
        base_url=config.local_llm_base_url,
        timeout=config.llm_request_timeout,
        max_retries=0,
    )


def _invoke_with_timeout(llm, messages, timeout: float):
    """在线程中调用模型，超时后不阻塞等待底层网络请求结束。"""
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    future = executor.submit(llm.invoke, messages)
    try:
        return future.result(timeout=timeout)
    finally:
        executor.shutdown(wait=False, cancel_futures=True)


def _is_cloud_llm(config: RobotConfig) -> bool:
    return config.llm_provider.lower() != config.local_llm_provider.lower()


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
        self.local_llm = None

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
                f"model={self.config.llm_model}, base_url={self.config.llm_base_url}"
            )
        except ImportError as e:
            self.logger.warning(
                f"LLM 依赖未安装 ({self.config.llm_provider}): {e}。"
                "请根据 provider 安装对应包，例如: "
                "uv add langchain-ollama / langchain-openai / langchain-anthropic"
            )
        except Exception as e:
            self.logger.warning(f"LLM 初始化失败: {e}")

    def _ensure_local_llm(self):
        """确保本地回退 LLM 已创建。"""
        if self.local_llm is None:
            self.local_llm = _create_local_llm(self.config)
            self.logger.info(
                "本地 LLM 回退已连接: "
                f"provider={self.config.local_llm_provider}, "
                f"model={self.config.local_llm_model}, "
                f"base_url={self.config.local_llm_base_url}"
            )
        return self.local_llm

    def update(self):
        if self.blackboard.intent != "chat":
            return Status.FAILURE

        if self.llm is None:
            if not (
                self.config.cloud_llm_fallback_to_local
                and _is_cloud_llm(self.config)
            ):
                self.blackboard.response_text = "大模型未就绪，请稍后再试。"
                return Status.SUCCESS
            try:
                self._ensure_local_llm()
                self.logger.warning("云端 LLM 未就绪，直接使用本地回退。")
            except Exception as fallback_err:
                self.logger.error(f"本地 LLM 回退初始化失败: {fallback_err}")
                self.blackboard.response_text = "抱歉，我暂时无法回答。"
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

            if self.llm is None:
                response = _invoke_with_timeout(
                    self.local_llm,
                    messages,
                    self.config.llm_request_timeout,
                )
            else:
                try:
                    response = _invoke_with_timeout(
                        self.llm,
                        messages,
                        self.config.llm_request_timeout,
                    )
                except Exception as primary_err:
                    if not (
                        self.config.cloud_llm_fallback_to_local
                        and _is_cloud_llm(self.config)
                    ):
                        raise
                    self.logger.warning(
                        "云端 LLM 调用失败，尝试本地回退: "
                        f"provider={self.config.llm_provider}, "
                        f"model={self.config.llm_model}, error={primary_err}"
                    )
                    response = _invoke_with_timeout(
                        self._ensure_local_llm(),
                        messages,
                        self.config.llm_request_timeout,
                    )

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
