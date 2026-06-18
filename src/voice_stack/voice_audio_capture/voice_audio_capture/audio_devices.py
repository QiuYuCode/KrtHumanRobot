"""sounddevice 输入/输出设备选择（从 smart-voice-robot/engine.py 迁移）。"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import sounddevice as sd


def configure_audio_devices(
    *,
    input_hint: str = "",
    output_hint: str = "",
    configure_input: bool = True,
    configure_output: bool = True,
    log_info: Callable[[str], Any] | None = None,
    log_warning: Callable[[str], Any] | None = None,
) -> None:
    """根据 hint 选择麦克风/扬声器；优先 pulse/default，其次 hint，最后 USB。"""
    info = log_info or (lambda _msg: None)
    warn = log_warning or (lambda _msg: None)

    try:
        devices = sd.query_devices()
    except Exception as exc:
        warn(f"[Audio] 查询设备失败，使用系统默认: {exc}")
        return

    pulse_idx = _find_pulse_device(
        devices,
        need_input=configure_input,
        need_output=configure_output,
    )
    if pulse_idx is not None:
        current_in, current_out = sd.default.device
        new_in = pulse_idx if configure_input else current_in
        new_out = pulse_idx if configure_output else current_out
        sd.default.device = (new_in, new_out)
        try:
            if configure_input:
                info(f"[Audio] 输入设备: {sd.query_devices(new_in)['name']}")
            if configure_output:
                info(f"[Audio] 输出设备: {sd.query_devices(new_out)['name']}")
        except Exception:
            pass
        return

    input_hint_l = (input_hint or "").strip().lower()
    output_hint_l = (output_hint or "").strip().lower()
    selected_input = None
    selected_output = None

    for idx, dev in enumerate(devices):
        name_l = str(dev.get("name", "")).lower()
        max_in = int(dev.get("max_input_channels", 0) or 0)
        max_out = int(dev.get("max_output_channels", 0) or 0)

        if (
            configure_input
            and selected_input is None
            and input_hint_l
            and input_hint_l in name_l
            and max_in > 0
        ):
            selected_input = idx
        if (
            configure_output
            and selected_output is None
            and output_hint_l
            and output_hint_l in name_l
            and max_out > 0
        ):
            selected_output = idx

    if configure_input and selected_input is None:
        selected_input = _find_usb_device(devices, for_input=True)
    if configure_output and selected_output is None:
        selected_output = _find_usb_device(devices, for_input=False)

    current_in, current_out = sd.default.device
    new_in = selected_input if configure_input and selected_input is not None else current_in
    new_out = selected_output if configure_output and selected_output is not None else current_out
    sd.default.device = (new_in, new_out)

    try:
        if configure_input:
            info(f"[Audio] 输入设备: {sd.query_devices(sd.default.device[0])['name']}")
        if configure_output:
            info(f"[Audio] 输出设备: {sd.query_devices(sd.default.device[1])['name']}")
    except Exception:
        pass


def _find_pulse_device(
    devices: list,
    *,
    need_input: bool,
    need_output: bool,
) -> int | None:
    for preferred in ("pulse", "default"):
        for idx, dev in enumerate(devices):
            name_l = str(dev.get("name", "")).lower().strip()
            if name_l != preferred:
                continue
            max_in = int(dev.get("max_input_channels", 0) or 0)
            max_out = int(dev.get("max_output_channels", 0) or 0)
            if need_input and need_output and max_in > 0 and max_out > 0:
                return idx
            if need_input and not need_output and max_in > 0:
                return idx
            if need_output and not need_input and max_out > 0:
                return idx
    return None


def _find_usb_device(devices: list, *, for_input: bool) -> int | None:
    for idx, dev in enumerate(devices):
        name_l = str(dev.get("name", "")).lower()
        if "usb" not in name_l:
            continue
        if for_input and int(dev.get("max_input_channels", 0) or 0) > 0:
            return idx
        if not for_input and int(dev.get("max_output_channels", 0) or 0) > 0:
            return idx
    return None
