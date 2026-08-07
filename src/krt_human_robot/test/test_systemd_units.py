"""Static contracts for dual-device systemd units."""

from pathlib import Path


WORKSPACE = Path(__file__).parents[3]
X86_UNIT = WORKSPACE / "deploy" / "systemd" / "krt-x86.service"


def test_x86_unit_omits_empty_optional_tls_arguments():
    """An HTTP deployment must not pass malformed empty ROS launch arguments."""
    source = X86_UNIT.read_text(encoding="utf-8")

    assert '[[ -n "$${KRT_WEB_CERTFILE:-}" ]]' in source
    assert '[[ -n "$${KRT_WEB_KEYFILE:-}" ]]' in source
    assert 'args+=(web_certfile:="$KRT_WEB_CERTFILE")' in source
    assert 'args+=(web_keyfile:="$KRT_WEB_KEYFILE")' in source
    assert '"$${args[@]}"' in source
