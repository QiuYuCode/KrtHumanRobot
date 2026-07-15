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
        "['media','音乐'],['routines','动作编排'],"
        "...(user?.role==='admin'?[['users','账号']]:[])]"
    )
    assert expected in html
    assert "动作组" not in html


def test_login_inputs_stay_inside_modal():
    html = TEMPLATE.read_text(encoding="utf-8")

    assert ".modal input{width:100%;min-width:0}" in html
