# Gripper Web Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add reusable named gripper actions, routine integration, direct Web control, and live hand telemetry to the robot console.

**Architecture:** Extend the shared SQLite repository with named gripper targets and keep the existing `gripper` routine type backward compatible. Reuse the existing `HandControl` actions and hand query services through the Web ROS bridge; use polling and native Canvas rendering rather than new ROS interfaces or frontend dependencies.

**Tech Stack:** Python 3.10, ROS 2 Humble/rclpy, Flask, SQLite, vanilla JavaScript/Canvas, pytest.

## Global Constraints

- Preserve existing inline gripper routine specifications.
- Do not add dependencies or modify `hands_control_interfaces`.
- Keep routine, waypoint, and direct-gripper execution mutually exclusive and cancellable.
- Poll telemetry at 2 Hz only while monitoring is active and the page is visible.
- Store named actions, but keep 30 seconds of chart history only in browser memory.

---

### Task 1: Named gripper action storage

**Files:**
- Modify: `src/krt_task/krt_task/robot_db.py`
- Test: `src/krt_task/test/test_robot_db.py`

**Interfaces:**
- Produces: `list_gripper_actions()`, `get_gripper_action(name)`, `save_gripper_action(name, targets)`, `rename_gripper_action(name, new_name)`, and `delete_gripper_action(name)`.
- Produces routine form: `{"type": "gripper", "action_name": "双手半握"}` while retaining inline targets.

- [ ] Add failing migration, validation, CRUD, rename-reference, delete-protection, and legacy compatibility tests.
- [ ] Run `UV_CACHE_DIR=/tmp/uv-cache uv run --no-sync --project src/voice_assistant python -m pytest src/krt_task/test/test_robot_db.py -q` and confirm the new tests fail for missing behavior.
- [ ] Upgrade the schema to v3, add the minimal repository methods, and validate named references from `save_routine()`.
- [ ] Run the focused database tests and confirm they pass.

### Task 2: Routine execution of named actions

**Files:**
- Modify: `src/krt_task/krt_task/routine_runner.py`
- Modify: `src/krt_task/launch/routine_runner.launch.py`
- Test: `src/krt_task/test/test_routine_runner.py`

**Interfaces:**
- Consumes: `RobotDatabase.get_gripper_action(name)`.
- Produces: one `ParallelJob` per saved target and aggregates single/both-hand completion and cancellation.

- [ ] Add failing tests for named single-hand and both-hand resolution, adapter-index mapping, partial failure, and cancellation.
- [ ] Run focused runner tests and verify expected failures.
- [ ] Add left/right adapter-index parameters and minimally generalize job waiting/parallel flattening for multiple hand goals.
- [ ] Run focused runner and database tests.

### Task 3: Web APIs, direct execution, and telemetry

**Files:**
- Modify: `src/krt_human_robot/krt_human_robot/web_app.py`
- Modify: `src/krt_human_robot/launch/web_console.launch.py`
- Test: `src/krt_human_robot/test/test_web_app.py`

**Interfaces:**
- Produces CRUD endpoints under `/api/gripper-actions`, defaults at `/api/gripper-defaults`, direct execution at `/api/gripper/run`, and monitoring endpoints under `/api/gripper`.
- Extends `/api/execution` with per-hand progress, position, status, and message while `mode == "gripper"`.

- [ ] Add failing API tests for auth/CSRF/audit, validation, CRUD, mutual exclusion, direct feedback, monitoring, partial telemetry failure, and lease restoration.
- [ ] Run focused Web tests and verify expected failures.
- [ ] Extend `RosBridge` and `WebRuntime` with existing Action/Service/parameter clients, a non-overlapping telemetry snapshot, and an inactivity restore timer.
- [ ] Add the minimal Flask endpoints and pass the shared robot config into the Web process.
- [ ] Run focused Web tests.

### Task 4: Console editor and live debug panel

**Files:**
- Modify: `src/krt_human_robot/templates/console.html`
- Test: `src/krt_human_robot/test/test_console_template.py`

**Interfaces:**
- Consumes the Task 3 JSON APIs.
- Produces a named-action editor, routine action selector, direct run controls, numeric telemetry, and native Canvas charts.

- [ ] Add failing template contract tests for navigation, named gripper selection, legacy inline editing, monitoring controls, and Canvas history.
- [ ] Run the template tests and verify expected failures.
- [ ] Add the smallest vanilla-JavaScript UI, retaining old inline rendering and storing at most 30 seconds of samples.
- [ ] Run template and Web tests.

### Task 5: Full verification

**Files:**
- Modify only if verification exposes a feature regression.

- [ ] Run `UV_CACHE_DIR=/tmp/uv-cache uv run --no-sync --project src/voice_assistant python -m pytest src/krt_task/test src/krt_human_robot/test -q`.
- [ ] Run `colcon build --packages-select krt_task krt_human_robot --symlink-install`.
- [ ] Run package lint/tests with `colcon test --packages-select krt_task krt_human_robot --event-handlers console_direct+` and inspect `colcon test-result --verbose`.
- [ ] Review `git diff --check`, the final diff, schema compatibility, API authentication, and all requirements above.
