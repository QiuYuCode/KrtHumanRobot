from pathlib import Path


TEMPLATE = Path(__file__).parents[1] / "templates" / "console.html"


def test_console_uses_offline_material_sidebar():
    html = TEMPLATE.read_text(encoding="utf-8")

    assert 'class="sidebar"' in html
    assert 'id="menuBtn"' in html
    assert 'id="drawerScrim"' in html
    assert '<svg' in html
    assert "https://" not in html
    for element_id in (
        "nav", "execStatus", "cancelBtn", "who", "logoutBtn", "waypoints",
        "routines", "actionGroups", "media", "users", "login", "message",
    ):
        assert f'id="{element_id}"' in html


def test_console_navigation_order_and_arm_action_label():
    html = TEMPLATE.read_text(encoding="utf-8")

    expected = (
        "const names=[['waypoints','点位'],['actionGroups','机械臂动作'],"
        "['grippers','夹爪动作'],['media','音乐'],['routines','动作编排'],"
        "...(user?.role==='admin'?[['users','账号']]:[])]"
    )
    assert expected in html
    assert "动作组" not in html


def test_login_inputs_stay_inside_modal():
    html = TEMPLATE.read_text(encoding="utf-8")

    assert ".modal input{width:100%;min-width:0}" in html


def test_console_supports_first_admin_setup_in_login_modal():
    html = TEMPLATE.read_text(encoding="utf-8")

    for element_id in ("loginTitle", "loginConfirmation", "loginSubmit"):
        assert f'id="{element_id}"' in html
    assert "s.needs_setup" in html
    assert "needsSetup?'/api/setup':'/api/login'" in html
    assert "password_confirmation" in html
    assert 'minlength="6"' in html


def test_console_has_named_gripper_editor_and_live_telemetry():
    html = TEMPLATE.read_text(encoding="utf-8")

    for element_id in (
        "grippers", "gripperActionList", "gripperName", "gripperSide",
        "gripperTargets", "newGripper", "saveGripper", "runGripper",
        "deleteGripper", "monitorGripper", "gripperTelemetry",
        "positionChart", "normalChart", "tangentChart", "approachChart",
    ):
        assert f'id="{element_id}"' in html
    assert "const TELEMETRY_WINDOW_MS=30000" in html
    assert "function drawTelemetry" in html
    assert "setInterval(pollGripperTelemetry,500)" in html
    assert "if(telemetryPolling||!gripperMonitoring" in html
    assert "finally{telemetryPolling=false}" in html
    assert "visibilitychange" in html
    assert "document.hidden)stopGripperMonitor()" in html


def test_right_hand_sensor_charts_show_finger_color_legends():
    html = TEMPLATE.read_text(encoding="utf-8")

    assert html.count('class="telemetry-legend"') == 3
    for finger_id, color in (
        (1, "#002fa7"), (2, "#e4002b"), (3, "#5f6368")
    ):
        assert html.count(f'data-finger="{finger_id}"') == 3
        assert f'--finger-color:{color}' in html


def test_console_has_gripper_lifecycle_cards_and_settings():
    html = TEMPLATE.read_text(encoding="utf-8")

    for element_id in (
        "gripperSystem", "startAllGrippers", "stopAllGrippers",
        "leftGripperSystem", "rightGripperSystem",
    ):
        assert f'id="{element_id}"' in html
    assert "function loadGripperSystem" in html
    assert "function controlGripperSystem" in html
    assert "function saveGripperHardware" in html
    assert "/api/gripper/system/" in html
    assert "!document.querySelector('#gripperSystem details[open]')" in html


def test_routine_editor_uses_named_gripper_actions_and_keeps_legacy_fields():
    html = TEMPLATE.read_text(encoding="utf-8")

    assert "action_name:gripperActions[0]?.name||''" in html
    assert "step.action_name" in html
    assert "finger_id" in html
    assert "position" in html
