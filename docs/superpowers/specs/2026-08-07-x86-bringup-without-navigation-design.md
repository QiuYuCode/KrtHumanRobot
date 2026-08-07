# x86 常驻服务移除导航设计

## 目标

`krt-x86.service` 启动时仅拉起常驻的机器人控制、语音、任务和 Web 服务，不启动 Nav2、定位或建图节点。导航继续由现有语音功能按需启动独立的 `ranger_nav` launch。

## 改动

- 从 `x86_bringup.launch.py` 删除 `_navigation`、导航相关 import 和 `OpaqueFunction`。
- 删除 `map`、`pcd_map_path`、`navigation_mode` launch 参数。
- 从 `krt-x86.service` 删除上述导航参数，只传 Web 参数。
- 更新双机 launch 静态测试，断言 x86 bringup 不再引用 `ranger_nav` 或导航 launch。
- 更新双机部署文档中的手动启动示例，导航改为独立命令。

## 验证

- `x86_bringup.launch.py --show-args` 不包含导航参数。
- 启动 `krt-x86.service` 后不存在 Nav2、定位和建图节点。
- 现有独立导航 launch 保持不变，语音按需启动路径不受影响。
