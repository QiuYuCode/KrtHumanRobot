# x86 Navigation Opt-In Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make navigation opt-in for `x86_bringup.launch.py` while preserving its existing command-line navigation modes.

**Architecture:** Keep the existing `_navigation` selector and navigation arguments. Add one boolean launch argument whose default disables the returned navigation include; systemd relies on that default while manual and voice-driven callers can explicitly enable navigation.

**Tech Stack:** ROS 2 Humble launch, Python, pytest, systemd user units.

## Global Constraints

- `enable_navigation` defaults to `false`.
- `map`, `pcd_map_path`, and `navigation_mode` remain available.
- No new dependency or abstraction.
- Existing unrelated working-tree changes must remain untouched.

---

### Task 1: Make navigation opt-in

**Files:**
- Modify: `src/krt_human_robot/test/test_dual_device_launch.py`
- Modify: `src/krt_human_robot/launch/x86_bringup.launch.py`
- Modify: `docs/dual_device_deployment.md`

**Interfaces:**
- Consumes: existing `_navigation(context)` selector and `ranger_nav` launch files.
- Produces: `enable_navigation` ROS launch argument with default `false`.

- [ ] **Step 1: Write the failing test**

Update the x86 launch contract to require `DeclareLaunchArgument("enable_navigation", default_value="false")`, `IfCondition(LaunchConfiguration("enable_navigation"))`, and both existing navigation launch filenames.

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest -q src/krt_human_robot/test/test_dual_device_launch.py`

Expected: x86 contract fails because `enable_navigation` and its condition are absent.

- [ ] **Step 3: Write minimal implementation**

Add `IfCondition` to the imports, apply it to the navigation `IncludeLaunchDescription`, and declare `enable_navigation` with default `false`. Update the deployment example to pass `enable_navigation:=true` when navigation is desired.

- [ ] **Step 4: Run focused verification**

Run:

```bash
python3 -m pytest -q src/krt_human_robot/test/test_dual_device_launch.py
source /opt/ros/humble/setup.bash
colcon build --packages-select krt_human_robot --symlink-install
source install/setup.bash
ros2 launch krt_human_robot x86_bringup.launch.py --show-args
```

Expected: tests and build pass; `enable_navigation` is shown with default `false`.

- [ ] **Step 5: Restart and verify systemd default**

Restart `krt-x86.service`, then verify it is active and its cgroup contains no Nav2, localization, mapping, Livox, or FAST-LIO process launched through the navigation include.

- [ ] **Step 6: Commit**

Commit only the task files with: `fix(deploy): x86 服务默认不启动导航`.
