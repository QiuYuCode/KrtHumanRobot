# Web 首次管理员初始化 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让空用户数据库的机器人控制台在 Web 端创建首个管理员并自动登录，同时把所有账号密码下限统一为 6 个字符。

**Architecture:** 复用现有 `AuthDatabase`、Flask session、登录 modal 和 CSRF token。数据库通过 SQLite `BEGIN IMMEDIATE` 串行化“检查为空并插入管理员”，前端根据 `/api/session.needs_setup` 在同一表单中切换初始化和登录模式。

**Tech Stack:** Python 3.10、Flask、SQLite、Werkzeug、原生 HTML/JavaScript、pytest。

## Global Constraints

- 空数据库时，同一局域网内任何能访问控制台的人都可以创建首个管理员。
- 首个管理员创建后，初始化接口必须返回 HTTP 409，不提供重新初始化能力。
- 所有账号密码至少 6 个字符；不提供默认密码或一次性初始化码。
- 保留 `krt_web_create_admin` 维护入口，不新增页面、第三方依赖或数据库迁移。
- 使用现有 `src/voice_assistant/.venv` 运行测试。

---

### Task 1: 原子首建管理员与初始化 API

**Files:**
- Modify: `src/krt_human_robot/krt_human_robot/web_auth.py`
- Modify: `src/krt_human_robot/krt_human_robot/web_app.py`
- Test: `src/krt_human_robot/test/test_web_app.py`

**Interfaces:**
- Produces: `AuthDatabase.needs_setup() -> bool`。
- Produces: `AuthDatabase.create_initial_admin(username: str, password: str) -> dict[str, Any] | None`；空库时创建并返回完整用户，已有用户时返回 `None`。
- Produces: `GET /api/session` 响应字段 `needs_setup: bool`。
- Produces: `POST /api/setup`，消费 `username`、`password`、`password_confirmation`，成功返回 `user` 与 `csrf_token`。

- [ ] **Step 1: 写 API 和密码边界失败测试**

在 `test_web_app.py` 增加一个最小应用工厂辅助函数，并写以下行为测试：

```python
def web_test_app(tmp_path):
    return create_app({
        "TESTING": True,
        "SECRET_KEY": "test",
        "SESSION_COOKIE_SECURE": False,
        "ROS_ENABLED": False,
        "ROBOT_DB": str(tmp_path / "robot.db"),
        "WEB_DB": str(tmp_path / "web.db"),
        "MEDIA_DIR": str(tmp_path / "media"),
    })


def test_first_admin_setup_creates_session_and_then_closes(tmp_path):
    app = web_test_app(tmp_path)
    client = app.test_client()

    assert client.get("/api/session").get_json()["needs_setup"] is True
    setup = client.post("/api/setup", json={
        "username": "admin", "password": "123456",
        "password_confirmation": "123456",
    })
    assert setup.status_code == 200
    assert setup.get_json()["user"] == {"id": 1, "username": "admin", "role": "admin"}
    assert setup.get_json()["csrf_token"]
    assert client.get("/api/session").get_json()["needs_setup"] is False
    assert client.post("/api/setup", json={
        "username": "second", "password": "123456",
        "password_confirmation": "123456",
    }).status_code == 409


def test_setup_validates_confirmation_and_six_character_password(tmp_path):
    client = web_test_app(tmp_path).test_client()
    assert client.post("/api/setup", json={
        "username": "admin", "password": "12345",
        "password_confirmation": "12345",
    }).status_code == 400
    assert client.post("/api/setup", json={
        "username": "admin", "password": "123456",
        "password_confirmation": "654321",
    }).status_code == 400


def test_all_user_password_paths_use_six_character_minimum(tmp_path):
    app = web_test_app(tmp_path)
    auth = app.extensions["auth_db"]
    auth.create_user("admin", "123456", "admin")
    user_id = auth.list_users()[0]["id"]
    auth.update_user(user_id, password="654321")
    assert auth.authenticate("admin", "654321") is not None
    with pytest.raises(ValueError, match="至少 6"):
        auth.create_user("short", "12345", "operator")
```

- [ ] **Step 2: 运行测试并确认按预期失败**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 src/voice_assistant/.venv/bin/python -m pytest \
  src/krt_human_robot/test/test_web_app.py \
  -k 'first_admin_setup or setup_validates or all_user_password' -q
```

Expected: FAIL，原因是 `/api/session` 缺少 `needs_setup`、`/api/setup` 不存在且 6 位密码仍被拒绝。

- [ ] **Step 3: 实现最小数据库行为**

在 `AuthDatabase` 中把两个密码检查改为 6，并增加：

```python
def needs_setup(self) -> bool:
    with self.connect() as connection:
        return connection.execute("SELECT 1 FROM users LIMIT 1").fetchone() is None

def create_initial_admin(self, username: str, password: str) -> dict[str, Any] | None:
    username = username.strip()
    if not username or len(username) > 64:
        raise ValueError("用户名无效")
    if len(password) < 6:
        raise ValueError("密码至少 6 个字符")
    with self.connect() as connection:
        connection.execute("BEGIN IMMEDIATE")
        if connection.execute("SELECT 1 FROM users LIMIT 1").fetchone():
            return None
        cursor = connection.execute(
            "INSERT INTO users(username,password_hash,role,created_at) VALUES(?,?,?,?)",
            (username, generate_password_hash(password), "admin", now()),
        )
        connection.commit()
        return self.get_user(cursor.lastrowid)
```

同步修改命令行提示为“至少 6 位”。不新增密码常量或异常类。

- [ ] **Step 4: 实现 session 字段和 setup 路由**

在 `session_info` 返回值增加 `needs_setup=auth.needs_setup()`。在 login 路由前增加：

```python
@app.post("/api/setup")
def setup():
    payload = request.get_json(silent=True) or {}
    password = str(payload.get("password", ""))
    if password != str(payload.get("password_confirmation", "")):
        return jsonify(error="两次密码不一致"), 400
    user = auth.create_initial_admin(str(payload.get("username", "")), password)
    if user is None:
        return jsonify(error="控制台初始化已完成"), 409
    session.clear()
    session["user_id"] = user["id"]
    session["csrf_token"] = secrets.token_urlsafe(24)
    auth.audit(user["username"], request.remote_addr or "-", "setup", user["username"], True)
    return jsonify(user=public_user(user), csrf_token=session["csrf_token"])
```

- [ ] **Step 5: 增加并发初始化测试**

使用两个独立 test client 和 `threading.Barrier` 同时 POST `/api/setup`，断言排序后的状态码为 `[200, 409]`，并断言 `auth.list_users()` 只有一个 `admin`。该测试必须调用真实 SQLite 文件，不 mock 数据库。

- [ ] **Step 6: 运行聚焦测试并确认通过**

Run: Step 2 命令，并把 `-k` 改为：

```text
first_admin_setup or setup_validates or all_user_password or concurrent_setup
```

Expected: 4 passed。

- [ ] **Step 7: 提交后端变更**

```bash
git add src/krt_human_robot/krt_human_robot/web_auth.py \
  src/krt_human_robot/krt_human_robot/web_app.py \
  src/krt_human_robot/test/test_web_app.py
git commit -m "feat(web): 支持首次管理员初始化"
```

### Task 2: 登录页初始化模式

**Files:**
- Modify: `src/krt_human_robot/templates/console.html`
- Test: `src/krt_human_robot/test/test_console_template.py`

**Interfaces:**
- Consumes: `/api/session.needs_setup` 和 `POST /api/setup`。
- Produces: 同一个 `#loginForm` 在 setup/login 两种模式切换；初始化成功后复用现有控制台加载流程。

- [ ] **Step 1: 写模板行为失败测试**

在 `test_console_template.py` 增加：

```python
def test_console_supports_first_admin_setup_in_login_modal():
    html = TEMPLATE.read_text(encoding="utf-8")
    for element_id in ("loginTitle", "loginConfirmation", "loginSubmit"):
        assert f'id="{element_id}"' in html
    assert 's.needs_setup' in html
    assert "needsSetup?'/api/setup':'/api/login'" in html
    assert "password_confirmation" in html
    assert 'minlength="6"' in html
```

- [ ] **Step 2: 运行测试并确认按预期失败**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 src/voice_assistant/.venv/bin/python -m pytest \
  src/krt_human_robot/test/test_console_template.py \
  -k first_admin_setup -q
```

Expected: FAIL，缺少初始化表单元素和模式切换逻辑。

- [ ] **Step 3: 最小扩展现有登录表单**

给标题和按钮增加 `loginTitle`、`loginSubmit` ID；给密码输入增加 `minlength="6"`。在密码字段后加入默认隐藏的确认密码容器：

```html
<div class="field hidden" id="loginConfirmation">
  <label>确认密码</label>
  <input id="loginPasswordConfirmation" type="password" minlength="6">
</div>
```

增加全局 `needsSetup=false`。在 `boot()` 中读取 `s.needs_setup`，设置标题、按钮、确认密码容器和 `required`；如果已有 session 用户则直接进入控制台。

提交表单时根据 `needsSetup` 选择 `/api/setup` 或 `/api/login`，初始化请求额外发送 `password_confirmation`。成功处理继续复用现有设置 `csrf`、`user`、隐藏 modal、`tabs()` 和 `refreshAll()` 的代码。

如果 `/api/setup` 返回 409，显示错误后重新调用 `boot()`，使页面切回登录模式。

- [ ] **Step 4: 运行模板测试和 Web API 测试**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 src/voice_assistant/.venv/bin/python -m pytest -q \
  src/krt_human_robot/test/test_console_template.py \
  src/krt_human_robot/test/test_web_app.py
```

Expected: 全部通过，0 failures。

- [ ] **Step 5: 验证代码质量和构建安装空间**

Run:

```bash
git diff --check
colcon build --packages-select krt_human_robot --symlink-install
```

Expected: 两条命令退出码均为 0。

- [ ] **Step 6: 重启并进行浏览器验收**

先备份实际 Web 数据库，不删除现有文件；使用临时数据库启动单独测试实例，验证初始化、自动登录、刷新后不再显示初始化模式。随后重启 `krt-x86.service`，确认 `systemctl --user is-active krt-x86.service` 为 `active`。

- [ ] **Step 7: 提交前端变更**

```bash
git add src/krt_human_robot/templates/console.html \
  src/krt_human_robot/test/test_console_template.py
git commit -m "feat(web): 增加首次管理员设置界面"
```

### Task 3: 最终回归验证

**Files:**
- Modify only if verification exposes a regression.

**Interfaces:**
- Consumes: Task 1 与 Task 2 的完整首次初始化流程。
- Produces: 可交付的 Web 首次管理员创建能力。

- [ ] **Step 1: 运行完整 Web 测试**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 src/voice_assistant/.venv/bin/python -m pytest -q \
  src/krt_human_robot/test/test_web_app.py \
  src/krt_human_robot/test/test_console_template.py
```

Expected: 0 failures。

- [ ] **Step 2: 运行迁移相关回归测试**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 src/voice_assistant/.venv/bin/python -m pytest -q \
  src/krt_human_robot/test/test_dual_device_launch.py \
  src/krt_human_robot/test/test_systemd_units.py
git diff --check
```

Expected: 0 failures，且 `git diff --check` 无输出。

- [ ] **Step 3: 审查工作区范围**

运行 `git status --short` 和 `git diff --stat`，确认只提交本计划涉及文件；保留并忽略既有迁移改动和未跟踪 `fishros`。
