from pathlib import Path

TEMPLATE = Path(__file__).parents[1] / "templates" / "console.html"
MAP_EDITOR = Path(__file__).parents[1] / "static" / "map_editor.html"


def compact(html: str) -> str:
    return "".join(html.split())


def test_console_uses_offline_material_sidebar():
    html = TEMPLATE.read_text(encoding="utf-8")

    assert 'class="sidebar"' in html
    assert 'id="menuBtn"' in html
    assert 'id="drawerScrim"' in html
    assert "<svg" in html
    assert "https://" not in html
    for element_id in (
        "nav",
        "execStatus",
        "cancelBtn",
        "who",
        "logoutBtn",
        "navigation",
        "waypoints",
        "routines",
        "actionGroups",
        "media",
        "users",
        "login",
        "message",
    ):
        assert f'id="{element_id}"' in html


def test_console_has_navigation_controls():
    html = TEMPLATE.read_text(encoding="utf-8")

    for element_id in (
        "startMapping",
        "finishMapping",
        "startNavigation",
        "stopNavigation",
        "startCruise",
        "stopCruise",
        "resumeCruise",
    ):
        assert f'id="{element_id}"' in html
    for path in (
        "/api/navigation/mapping/start",
        "/api/navigation/mapping/finish",
        "/api/navigation/start",
        "/api/navigation/stop",
        "/api/navigation/cruise/start",
        "/api/navigation/cruise/stop",
        "/api/navigation/cruise/resume",
    ):
        assert path in html


def test_dynamic_table_html_keeps_table_and_select_context():
    html = TEMPLATE.read_text(encoding="utf-8")

    assert 'target.tagName === "TBODY"' in html
    assert '<table><tbody id="${rootId}">${markup}</tbody></table>' in html
    assert 'target.tagName === "SELECT"' in html
    assert '<select id="${rootId}">${markup}</select>' in html
    assert "target.replaceChildren(...root.childNodes)" in html


def test_console_has_map_catalog_and_map_scoped_waypoints():
    html = TEMPLATE.read_text(encoding="utf-8")

    for element_id in (
        "mapName",
        "mappingBackend",
        "navigationMode",
        "navigationState",
        "selectedMapSummary",
        "mapRuntimeStatus",
        "mapRows",
        "refreshMaps",
        "waypointMap",
    ):
        assert f'id="{element_id}"' in html
    for path in (
        "/api/maps",
        "/api/navigation/status",
        "/api/maps/${mapId}/select",
        "/api/maps/${button.dataset.delmap}",
        "/api/waypoints?map_id=",
        "/map-editor?map_id=",
    ):
        assert path in html
    assert "rviz" not in html.lower() or "RViz" in html


def test_local_map_editor_overwrites_selected_map_without_keepout():
    html = MAP_EDITOR.read_text(encoding="utf-8")

    assert "646104e" in html
    assert "/static/vendor/jquery-3.4.1.min.js" in html
    assert "https://" not in html
    assert "保存并覆盖所选地图" in html
    assert "/editor/yaml" in html and "/editor/pgm" in html
    assert 'method:"PUT"' in compact(html)
    assert 'data-tool="mask"' not in html
    assert "btnDownloadMask" not in html
    assert 'class="fa ' not in html
    assert html.count('class="toolbar-icon"') == 9
    assert "墙体" in html and "擦除" in html and "未知区" in html


def test_console_navigation_order_and_arm_action_label():
    html = TEMPLATE.read_text(encoding="utf-8")

    source = compact(html)
    entries = (
        '["navigation","导航"]',
        '["waypoints","点位"]',
        '["actionGroups","机械臂动作"]',
        '["grippers","夹爪动作"]',
        '["media","音乐"]',
        '["routines","动作编排"]',
    )
    positions = [source.index(entry) for entry in entries]
    assert positions == sorted(positions)
    assert 'user?.role==="admin"?[["users","账号"]]:[]' in source
    assert "动作组" not in html


def test_login_inputs_stay_inside_modal():
    html = TEMPLATE.read_text(encoding="utf-8")

    assert ".auth-modalinput{width:100%;min-width:0;}" in compact(html)


def test_console_uses_local_bootstrap_and_modal_editors():
    html = TEMPLATE.read_text(encoding="utf-8")
    source = compact(html)

    assert '/static/vendor/bootstrap-4.4.1.min.css' in source
    assert '/static/vendor/jquery-3.4.1.min.js' in source
    assert '/static/vendor/bootstrap-4.4.1.bundle.min.js' in source
    assert 'id="routineEditorModal"' in html
    assert 'id="gripperEditorModal"' in html
    assert 'class="auth-modal"' in html
    assert 'class="table table-hover list-table mb-0"' in html
    assert 'data-edit-routine' in html
    assert 'data-edit-gripper' in html


def test_gripper_page_puts_monitoring_before_action_list_controls():
    html = TEMPLATE.read_text(encoding="utf-8")
    source = compact(html)
    assert "#grippers.active>.panel:not(#gripperSystem){order:3;}" in source
    assert "#grippers.active>.card{order:4;}" in source
    list_header = html.index("夹爪动作列表")
    assert html.index('id="newGripper"', list_header) > list_header
    telemetry = html.index("实时检测")
    assert html.index('id="monitorGripper"', telemetry) > telemetry


def test_robot_system_cards_keep_routine_status_on_routine_page():
    html = TEMPLATE.read_text(encoding="utf-8")
    source = compact(html)
    assert '.filter(([name])=>name!=="routine")' in source
    assert 'id="routineSystemBadge"' in html


def test_console_supports_first_admin_setup_in_login_modal():
    html = TEMPLATE.read_text(encoding="utf-8")

    for element_id in ("loginTitle", "loginConfirmation", "loginSubmit"):
        assert f'id="{element_id}"' in html
    assert "s.needs_setup" in html
    assert 'needsSetup?"/api/setup":"/api/login"' in compact(html)
    assert "password_confirmation" in html
    assert 'minlength="6"' in html


def test_console_has_named_gripper_editor_and_live_telemetry():
    html = TEMPLATE.read_text(encoding="utf-8")

    for element_id in (
        "grippers",
        "gripperActionList",
        "gripperName",
        "gripperSide",
        "gripperTargets",
        "newGripper",
        "saveGripper",
        "runGripper",
        "deleteGripper",
        "monitorGripper",
        "gripperTelemetry",
        "positionChart",
        "normalChart",
        "tangentChart",
        "approachChart",
    ):
        assert f'id="{element_id}"' in html
    source = compact(html)
    assert "constTELEMETRY_WINDOW_MS=30000" in source
    assert "functiondrawTelemetry" in source
    assert "setInterval(pollGripperTelemetry,500)" in source
    assert "if(telemetryPolling||!gripperMonitoring" in source
    assert "finally{telemetryPolling=false;}" in source
    assert "visibilitychange" in html
    assert "document.hidden)stopGripperMonitor()" in source


def test_right_hand_sensor_charts_show_finger_color_legends():
    html = TEMPLATE.read_text(encoding="utf-8")

    assert html.count('class="telemetry-legend"') == 3
    for finger_id, color in ((1, "#002fa7"), (2, "#e4002b"), (3, "#5f6368")):
        assert html.count(f'data-finger="{finger_id}"') == 3
        assert f"--finger-color:{color}" in compact(html)


def test_console_has_gripper_lifecycle_cards_and_settings():
    html = TEMPLATE.read_text(encoding="utf-8")

    for element_id in (
        "gripperSystem",
        "startAllGrippers",
        "stopAllGrippers",
        "leftGripperSystem",
        "rightGripperSystem",
    ):
        assert f'id="{element_id}"' in html
    assert "function loadGripperSystem" in html
    assert "function controlGripperSystem" in html
    assert "function saveGripperHardware" in html
    assert "/api/gripper/system/" in html
    assert '!document.querySelector("#gripperSystemdetails[open]")' in compact(html)


def test_routine_editor_uses_named_gripper_actions_and_keeps_legacy_fields():
    html = TEMPLATE.read_text(encoding="utf-8")

    assert 'action_name:gripperActions[0]?.name||""' in compact(html)
    assert "step.action_name" in html
    assert "finger_id" in html
    assert "position" in html
