"""唤醒词检测节点"""

import numpy as np
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from voice_assistant.config import SAMPLE_RATE


class WaitForWakeWord(Behaviour):
    """
    等待唤醒词。

    持续监听麦克风，检测到唤醒词后返回 SUCCESS。
    通过 config.kws_keywords_file 替换唤醒词，无需修改代码。
    """

    def __init__(self, name: str, engine):
        super().__init__(name)
        self.engine = engine
        self.kws_stream = None

    def setup(self, **kwargs):
        self.kws_stream = self.engine.kws.create_stream()

    def initialise(self):
        self.logger.info("待机中，等待唤醒词...")
        # 重置 KWS 流状态
        if self.kws_stream is not None:
            self.engine.kws.reset_stream(self.kws_stream)
        # 清空队列，避免处理旧数据
        self.engine.clear_kws_queue()

    def update(self):
        while not self.engine.kws_audio_queue.empty():
            data = self.engine.kws_audio_queue.get()
            samples = np.frombuffer(data, dtype=np.float32)

            self.kws_stream.accept_waveform(SAMPLE_RATE, samples)
            while self.engine.kws.is_ready(self.kws_stream):
                self.engine.kws.decode_stream(self.kws_stream)
                keyword = self.engine.kws.get_result(self.kws_stream)
                if keyword:
                    self.logger.info(f"唤醒成功! 关键词: {keyword.strip()}")
                    self.engine.kws.reset_stream(self.kws_stream)
                    return Status.SUCCESS

        # 待机阶段不需要保留 ASR 队列中的历史音频，避免积压。
        self.engine.clear_dialog_queue()
        return Status.RUNNING

    def terminate(self, new_status):
        pass
