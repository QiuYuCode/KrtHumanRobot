# x86 常驻服务移除导航设计

## 目标

`krt-x86.service` 启动时仅拉起常驻的机器人控制、语音、任务和 Web 服务，默认不启动 Nav2、定位或建图节点。保留命令行显式启用导航的能力，现有语音功能也可继续按需启动导航。

## 改动

- 在 `x86_bringup.launch.py` 新增 `enable_navigation` 参数，默认值为 `false`。
- 保留 `_navigation`、`map`、`pcd_map_path` 和 `navigation_mode`；仅当 `enable_navigation:=true` 时 include 对应导航 launch。
- `krt-x86.service` 不传 `enable_navigation`，因此服务启动不拉起导航。
- 更新双机 launch 静态测试，断言导航默认关闭且仍可显式启用。
- 更新双机部署文档中的手动启动示例，明确显式启用导航。

## 验证

- `x86_bringup.launch.py --show-args` 显示 `enable_navigation` 默认值为 `false`，并保留导航参数。
- 启动 `krt-x86.service` 后不存在 Nav2、定位和建图节点。
- 使用 `enable_navigation:=true` 可从命令行启动原有导航组合。
- 语音按需启动路径不受影响。
