import subprocess
from pathlib import Path

TEMPLATE = Path(__file__).parents[1] / "templates" / "console.html"
MAP_EDITOR = Path(__file__).parents[1] / "static" / "map_editor.html"


def compact(html: str) -> str:
    return "".join(html.split())


def test_console_execution_terminal_states_are_idle_and_notified_once_per_run():
    """A poll must not repeat a terminal error, but a later run may report one."""
    html = TEMPLATE.read_text(encoding="utf-8")
    start = html.find("function renderExecutionStatus")
    assert start >= 0, "execution status rendering should be separately testable"
    end = html.find("async function status", start)
    renderer = html[start:end]

    script = """
const elements = {
  execStatus: { textContent: "" },
  cancelBtn: { disabled: true },
};
const notices = [];
const $ = (id) => elements[id];
const msg = (message) => notices.push(message);
let terminalExecutionNotified = false;
%s
const poll = (status, message = "") => renderExecutionStatus({
  status, mode: "routine", current_step: "步骤", message,
});
poll("running");
if (elements.execStatus.textContent !== "routine · 步骤" || elements.cancelBtn.disabled)
  throw new Error("running state was not rendered");
poll("failed", "执行失败");
if (elements.execStatus.textContent !== "空闲" || !elements.cancelBtn.disabled)
  throw new Error("terminal state was not rendered as idle");
poll("failed", "执行失败");
if (notices.join("|") !== "执行失败")
  throw new Error("terminal failure was notified more than once");
poll("running");
poll("canceled", "已取消");
if (notices.join("|") !== "执行失败|已取消")
  throw new Error("a later run did not reset terminal notification dedupe");
""" % renderer

    result = subprocess.run(
        ["node", "-e", script], text=True, capture_output=True, check=False
    )
    assert result.returncode == 0, result.stderr


def test_console_successful_start_resets_terminal_notification_without_polling_running():
    """A short run can finish between polls and must still notify independently."""
    html = TEMPLATE.read_text(encoding="utf-8")
    renderer_start = html.find("function renderExecutionStatus")
    start_execution = html.find("async function startExecution", renderer_start)
    status_start = html.find("async function status", start_execution)
    assert renderer_start >= 0 and start_execution >= 0 and status_start >= 0
    renderer = html[renderer_start:start_execution]
    starter = html[start_execution:status_start]

    script = """
const elements = {
  execStatus: { textContent: "" },
  cancelBtn: { disabled: true },
};
const $ = (id) => elements[id];
const notices = [];
const msg = (message) => notices.push(message);
let terminalExecutionNotified = false;
let executionEpoch = 0;
%s
%s
const terminal = () => renderExecutionStatus({
  status: "failed", mode: "routine", current_step: "", message: "执行失败",
});
terminal();
let resolveStart;
const start = startExecution(() => new Promise((resolve) => { resolveStart = resolve; }));
terminal();
if (notices.join("|") !== "执行失败")
  throw new Error("a pending start reset terminal notification too early");
resolveStart();
await start;
terminal();
if (notices.join("|") !== "执行失败|执行失败")
  throw new Error("a successful start did not reset terminal notification without a running poll");
""" % (renderer, starter)

    result = subprocess.run(
        ["node", "-e", script], text=True, capture_output=True, check=False
    )
    assert result.returncode == 0, result.stderr


def test_console_discards_pre_start_terminal_poll_response():
    """A terminal response from before a successful start belongs to an old run."""
    html = TEMPLATE.read_text(encoding="utf-8")
    renderer_start = html.find("function renderExecutionStatus")
    start_execution = html.find("async function startExecution", renderer_start)
    status_start = html.find("async function status", start_execution)
    status_end = html.find("document.querySelectorAll(\"[data-robot-control]\")", status_start)
    assert renderer_start >= 0 and start_execution >= 0 and status_end >= 0
    renderer = html[renderer_start:start_execution]
    starter = html[start_execution:status_start]
    status = html[status_start:status_end]

    script = """
const elements = {
  execStatus: { textContent: "" },
  cancelBtn: { disabled: true },
  grippers: { classList: { contains: () => false } },
  actionGroups: { classList: { contains: () => false } },
  routines: { classList: { contains: () => false } },
};
const $ = (id) => elements[id];
const notices = [];
const msg = (message) => notices.push(message);
let user = {};
let terminalExecutionNotified = false;
let executionStatusPolling = false;
let executionEpoch = 0;
const responses = [];
const api = () => new Promise((resolve) => responses.push(resolve));
const loadNavigationStatus = () => {};
const loadGripperSystem = () => {};
const loadRobotSystems = () => {};
%s
%s
%s
const oldPoll = status();
await startExecution(() => Promise.resolve());
responses[0]({ status: "failed", mode: "routine", current_step: "", message: "旧失败" });
await oldPoll;
const newPoll = status();
responses[1]({ status: "failed", mode: "routine", current_step: "", message: "新失败" });
await newPoll;
if (notices.join("|") !== "新失败")
  throw new Error("a pre-start terminal response was rendered instead of being discarded");
""" % (renderer, starter, status)

    result = subprocess.run(
        ["node", "-e", script], text=True, capture_output=True, check=False
    )
    assert result.returncode == 0, result.stderr


def test_console_status_polling_does_not_overlap_terminal_state_updates():
    """An older running response must not reset terminal notification state."""
    html = TEMPLATE.read_text(encoding="utf-8")
    renderer_start = html.find("function renderExecutionStatus")
    status_start = html.find("async function status", renderer_start)
    status_end = html.find("document.querySelectorAll(\"[data-robot-control]\")", status_start)
    assert renderer_start >= 0 and status_start >= 0 and status_end >= 0
    renderer = html[renderer_start:status_start]
    status = html[status_start:status_end]

    script = """
const elements = {
  execStatus: { textContent: "" },
  cancelBtn: { disabled: true },
  grippers: { classList: { contains: () => false } },
  actionGroups: { classList: { contains: () => false } },
  routines: { classList: { contains: () => false } },
};
const $ = (id) => elements[id];
const notices = [];
const msg = (message) => notices.push(message);
let user = {};
let terminalExecutionNotified = false;
let executionStatusPolling = false;
let executionEpoch = 0;
const responses = [];
const api = () => new Promise((resolve) => responses.push(resolve));
const loadNavigationStatus = () => {};
const loadGripperSystem = () => {};
const loadRobotSystems = () => {};
%s
%s
const olderPoll = status();
const overlappingPoll = status();
if (responses.length !== 1)
  throw new Error("an overlapping poll can let a stale running response reset a terminal notification");
responses[0]({ status: "failed", mode: "routine", current_step: "", message: "执行失败" });
await Promise.all([olderPoll, overlappingPoll]);
const repeatPoll = status();
responses[1]({ status: "failed", mode: "routine", current_step: "", message: "执行失败" });
await repeatPoll;
if (notices.join("|") !== "执行失败")
  throw new Error("terminal failure was notified more than once");
""" % (renderer, status)

    result = subprocess.run(
        ["node", "-e", script], text=True, capture_output=True, check=False
    )
    assert result.returncode == 0, result.stderr


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


def test_waypoint_table_columns_match_rendered_rows():
    html = TEMPLATE.read_text(encoding="utf-8")
    assert "<th>名称</th><th>坐标</th><th>编排动作</th><th>操作</th>" in html
    assert '<td>${w.x.toFixed(2)}, ${w.y.toFixed(2)}</td><td><select data-bind=' in html
    assert '<td>${esc(w.name)}</td><td>${esc(maps.find(' not in html
    assert 'colspan="4" class="muted">当前地图暂无点位。' in html


def test_console_supports_offline_waypoint_initialization_for_3d_navigation():
    html = TEMPLATE.read_text(encoding="utf-8")

    for element_id in ("mapCanvas", "initialWaypointMenu", "initialWaypointSummary"):
        assert f'id="{element_id}"' in html
    assert "/api/waypoints/map-position" in html
    assert "initial_waypoint" in html
    assert "pointerdown" in html
    assert "rviz" not in html.lower() or "RViz" in html


def test_console_prompts_for_map_waypoint_name_and_allows_menu_deletion():
    html = TEMPLATE.read_text(encoding="utf-8")

    for element_id in (
        "mapWaypointSaveModal",
        "mapWaypointModalName",
        "initialWaypointMenu",
    ):
        assert f'id="{element_id}"' in html
    assert "data-delete-map-waypoint" in html
    assert "pendingMapWaypoint" in html


def test_console_draws_compact_circle_and_open_arrow_waypoints():
    html = TEMPLATE.read_text(encoding="utf-8")
    start = html.index("const drawWaypoint")
    drawing = html[start:html.index("mapWaypoints.filter", start)]

    assert "ctx.arc(" in drawing
    assert "const arrowShaftLength = 8" in drawing
    assert "const arrowWingLength = 3" in drawing
    assert "ctx.closePath()" not in drawing
    assert "ctx.fill()" not in drawing


def test_console_keeps_map_preview_proportional_and_offers_large_view():
    html = TEMPLATE.read_text(encoding="utf-8")

    assert 'id="mapPreviewModal"' in html
    assert 'id="mapCanvasModal"' in html
    assert "fitMapCanvas" in html
    assert "\n        width: 100%;" not in html[html.index(".map-picker canvas"):html.index("table {")]


def test_console_map_metadata_parser_accepts_block_style_origin():
    html = TEMPLATE.read_text(encoding="utf-8")

    assert "originBlock" in html


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


def test_map_flow_isolated_by_page_mode_and_modal_is_root_level():
    html = TEMPLATE.read_text(encoding="utf-8")
    navigation = html.index('<section id="navigation"')
    modal = html.index('<div id="mapCreatePanel"')
    navigation_end = html.index("</section>", navigation)
    assert modal > html.index("</main>")
    assert modal > navigation_end
    assert 'class="map-directory panel home-only"' in html
    assert 'id="openMapCreate" class="primary home-only"' in html
    assert 'body.home-mode .detail-only' in html
    assert 'body.detail-mode .home-only' in html
    assert 'body.detail-mode #mapCreatePanel' not in html
    assert 'href="/" class="home-link">返回地图目录' in html
    assert 'data-map-detail="${esc(map.id)}"' in html
    assert 'event.stopPropagation()' in html


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


def test_routine_editor_manages_voice_keywords_and_success_response():
    html = TEMPLATE.read_text(encoding="utf-8")
    source = compact(html)

    assert 'id="routineKeywords"' in html
    assert 'id="routineResponseText"' in html
    assert "r.voice_trigger?.keywords" in html
    assert "functionparseRoutineKeywords" in source
    assert "voice_trigger:{keywords:parseRoutineKeywords()" in source
