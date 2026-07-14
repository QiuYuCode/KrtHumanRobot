# Routine、点位与 Web 控制台测试复现

运行时数据统一保存到 `~/maps/krt_robot.db`。YAML 只用于旧数据导入和备份导出。

## 1. 安装并构建

```bash
cd ~/WorkSpace/KrtHumanRobot
cd src/voice_assistant
uv sync
cd ../..

colcon build --packages-select \
  voice_interfaces voice_playback krt_task_interfaces krt_task \
  ranger_nav krt_human_robot --symlink-install
source install/setup.bash
```

不要使用系统 Flask。Web launch 会通过 `src/voice_assistant/.venv` 中锁定的
Flask 和 Gunicorn 运行。

## 2. 导入旧 YAML

仅在目标数据库为空时执行：

```bash
ros2 run krt_task krt_robot_data \
  --db ~/maps/krt_robot.db import-yaml \
  --waypoints ~/maps/waypoints.yaml \
  --routines ~/maps/routines.yaml \
  --media-dir ~/music
```

如果没有旧 routine 文件，先创建空文件：

```yaml
routines: {}
```

导入不会删除原 YAML。备份导出：

```bash
ros2 run krt_task krt_robot_data \
  --db ~/maps/krt_robot.db export-yaml ~/maps/krt_robot_backup.yaml
```

## 3. 创建管理员和 HTTPS 证书

```bash
ros2 run krt_human_robot krt_web_create_admin admin \
  --db ~/.local/share/krt_human_robot/web.db

mkdir -p ~/.config/krt_human_robot
openssl req -x509 -newkey rsa:2048 -nodes -days 365 \
  -keyout ~/.config/krt_human_robot/web.key \
  -out ~/.config/krt_human_robot/web.crt \
  -subj "/CN=$(hostname)"
chmod 600 ~/.config/krt_human_robot/web.key
```

浏览器首次访问自签名证书会显示警告。正式环境替换为局域网可信证书。

## 4. 启动

只启动 routine runner 和 Web：

```bash
ros2 launch krt_task routine_runner.launch.py \
  robot_db:=$HOME/maps/krt_robot.db media_dir:=$HOME/music

ros2 launch krt_human_robot web_console.launch.py \
  robot_db:=$HOME/maps/krt_robot.db \
  media_dir:=$HOME/music \
  certfile:=$HOME/.config/krt_human_robot/web.crt \
  keyfile:=$HOME/.config/krt_human_robot/web.key
```

整机启动：

```bash
ros2 launch krt_human_robot robot.launch.py \
  robot_db:=$HOME/maps/krt_robot.db \
  media_dir:=$HOME/music \
  enable_web_console:=true \
  web_certfile:=$HOME/.config/krt_human_robot/web.crt \
  web_keyfile:=$HOME/.config/krt_human_robot/web.key
```

访问 `https://机器人IP:8443`。

## 5. Web 完整闭环

1. 管理员登录，在“账号”中创建操作员。
2. 在“音乐”中上传不超过 100MB 的 16-bit PCM WAV。
3. 在“动作编排”中新建 routine。
4. 添加语音、机械臂、夹爪和等待步骤。
5. 添加并行组，将音乐和机械臂动作放入同一个并行组。
6. 保存并点击“试运行”，确认当前步骤变化且“停止”可取消任务。
7. 将机器人移动到目标位置，在“点位”中填写名称并记录当前位置。
8. 为点位绑定 routine，点击“执行”。
9. 确认 Nav2 到点后调用 `/krt_task/run_routine`。

## 6. CLI 与 ROS 接口测试

点位：

```bash
ros2 run ranger_nav waypoint_manager --robot-db ~/maps/krt_robot.db mark 展示点
ros2 run ranger_nav waypoint_manager --robot-db ~/maps/krt_robot.db list
ros2 run ranger_nav waypoint_manager --robot-db ~/maps/krt_robot.db \
  bind 展示点 --routine 唱跳表演
ros2 run ranger_nav waypoint_manager --robot-db ~/maps/krt_robot.db cruise 展示点
```

Routine Action：

```bash
ros2 action send_goal /krt_task/run_routine \
  krt_task_interfaces/action/RunRoutine \
  "{routine_name: 唱跳表演}" --feedback
```

## 7. 自动测试

```bash
source install/setup.bash
cd src/voice_assistant
UV_CACHE_DIR=/tmp/uv-cache PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  uv run --no-sync pytest \
  ../krt_task/test/test_robot_db.py \
  ../krt_human_robot/test/test_web_app.py -q
```

## 8. 常见故障

- `routine 不存在`：数据库尚未导入或 Web 中尚未保存该 routine。
- `媒体不存在`：数据库记录对应的 `~/music/<media_key>.wav` 被手工删除。
- `routine action 不可用`：确认 `routine_runner` 已启动。
- `Nav2 action 不可用`：先启动导航并完成定位。
- Web 启动后退出：检查证书路径、`uv sync` 和 `source install/setup.bash`。
- 系统 Flask 导入错误不影响 Web；Web 固定从 uv 虚拟环境启动。
