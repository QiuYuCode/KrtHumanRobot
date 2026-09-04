"""Contracts for dual-device systemd startup readiness."""

import os
from pathlib import Path
import subprocess

import pytest


WORKSPACE = Path(__file__).parents[3]
X86_UNIT = WORKSPACE / "deploy" / "systemd" / "krt-x86.service"
JETSON_UNIT = WORKSPACE / "deploy" / "systemd" / "krt-jetson.service"
READINESS_SCRIPT = WORKSPACE / "deploy" / "systemd" / "krt-wait-ready.sh"
RVIZ_ENV_SCRIPT = WORKSPACE / "deploy" / "systemd" / "krt-rviz-env.sh"
X86_ENV_EXAMPLE = WORKSPACE / "deploy" / "env" / "x86.env.example"


def test_x86_unit_omits_empty_optional_tls_arguments():
    """An HTTP deployment must not pass malformed empty ROS launch arguments."""
    source = X86_UNIT.read_text(encoding="utf-8")

    assert '[[ -n "$${KRT_WEB_CERTFILE:-}" ]]' in source
    assert '[[ -n "$${KRT_WEB_KEYFILE:-}" ]]' in source
    assert 'args+=(web_certfile:="$KRT_WEB_CERTFILE")' in source
    assert 'args+=(web_keyfile:="$KRT_WEB_KEYFILE")' in source
    assert '"$${args[@]}"' in source


def test_x86_unit_does_not_resolve_rviz_before_the_web_launch():
    source = X86_UNIT.read_text(encoding="utf-8")

    assert 'source "$KRT_WORKSPACE/deploy/systemd/krt-rviz-env.sh"' not in source


@pytest.mark.parametrize("stale_socket", [False, True])
def test_rviz_environment_falls_back_to_active_x11_session(tmp_path, stale_socket):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    (fake_bin / "loginctl").write_text(
        "#!/bin/bash\n"
        "if [[ $1 == list-sessions ]]; then printf '%s\n' '42 create -'; exit; fi\n"
        "case $4 in Type) printf '%s\n' x11;; Remote) printf '%s\n' no;; "
        "State) printf '%s\n' active;; Display) printf '%s\n' '';; esac\n",
        encoding="utf-8",
    )
    (fake_bin / "loginctl").chmod(0o755)
    (fake_bin / "xdpyinfo").write_text(
        "#!/bin/bash\n"
        "[[ ${DISPLAY:-} == :1 ]]\n",
        encoding="utf-8",
    )
    (fake_bin / "xdpyinfo").chmod(0o755)
    socket_dir = tmp_path / ".X11-unix"
    socket_dir.mkdir()
    if stale_socket:
        (socket_dir / "X0").symlink_to("/tmp/.X11-unix/X1")
    (socket_dir / "X1").symlink_to("/tmp/.X11-unix/X1")
    authority = tmp_path / "Xauthority"
    authority.write_bytes(b"auth")
    result = subprocess.run(
        ["bash", str(RVIZ_ENV_SCRIPT)],
        capture_output=True,
        check=False,
        env=os.environ | {
            "PATH": f"{fake_bin}:{os.environ['PATH']}",
            "USER": "create",
            "HOME": str(tmp_path),
            "UID": "1000",
            "KRT_RVIZ_DISPLAY": ":0",
            "KRT_RVIZ_XAUTHORITY": str(authority),
            "KRT_X11_SOCKET_DIR": str(socket_dir),
            "DISPLAY": "",
            "XAUTHORITY": "",
        },
        cwd=tmp_path,
        text=True,
    )
    assert result.returncode == 0
    assert "DISPLAY=:1" in result.stdout


def test_x86_readiness_accepts_the_configured_dds_bind_address(tmp_path):
    """A ready x86 starts only after its Cyclone DDS address is present."""
    fake_ip = tmp_path / "ip"
    fake_ip.write_text(
        "#!/bin/bash\n"
        "printf '%s\\n' '2: eno1    inet 10.168.1.100/24 brd 10.168.1.255'\n",
        encoding="utf-8",
    )
    fake_ip.chmod(0o755)

    environment = os.environ | {
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "KRT_STARTUP_TIMEOUT_S": "0",
        "KRT_WAIT_INTERVAL_S": "0",
        "KRT_DDS_BIND_ADDRESS": "10.168.1.100",
        "KRT_REQUIRE_GLOBAL_NETWORK": "no",
        "KRT_CAN_REQUIRED_CHANNELS": "",
    }
    result = subprocess.run(
        ["/bin/bash", str(READINESS_SCRIPT), "x86"],
        capture_output=True,
        check=False,
        env=environment,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "startup mode=x86 timeout_s=0" in result.stdout
    assert "observed_ipv4=10.168.1.100" in result.stdout
    assert "DDS bind address, global network, and required CAN buses ready" in result.stdout


def test_x86_readiness_rejects_a_missing_dds_bind_address(tmp_path):
    """Starting ROS before its configured DDS address exists reintroduces the boot race."""
    fake_ip = tmp_path / "ip"
    fake_ip.write_text(
        "#!/bin/bash\n"
        "printf '%s\\n' '1: lo    inet 127.0.0.1/8 scope host lo'\n",
        encoding="utf-8",
    )
    fake_ip.chmod(0o755)

    environment = os.environ | {
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "KRT_STARTUP_TIMEOUT_S": "0",
        "KRT_WAIT_INTERVAL_S": "0",
        "KRT_DDS_BIND_ADDRESS": "10.168.1.100",
        "KRT_REQUIRE_GLOBAL_NETWORK": "no",
        "KRT_CAN_REQUIRED_CHANNELS": "",
    }
    result = subprocess.run(
        ["/bin/bash", str(READINESS_SCRIPT), "x86"],
        capture_output=True,
        check=False,
        env=environment,
        text=True,
    )

    assert result.returncode == 1
    assert "timed out after 0s" in result.stdout


def test_x86_readiness_rejects_network_without_global_connectivity(tmp_path):
    """An assigned IP is insufficient while NetworkManager is still CONNECTED_SITE."""
    fake_ip = tmp_path / "ip"
    fake_ip.write_text(
        "#!/bin/bash\n"
        "printf '%s\\n' '2: eno1    inet 10.168.1.100/24 brd 10.168.1.255'\n",
        encoding="utf-8",
    )
    fake_ip.chmod(0o755)
    fake_nmcli = tmp_path / "nmcli"
    fake_nmcli.write_text("#!/bin/bash\nprintf '%s\\n' 'connecting'\n", encoding="utf-8")
    fake_nmcli.chmod(0o755)

    environment = os.environ | {
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "KRT_STARTUP_TIMEOUT_S": "0",
        "KRT_WAIT_INTERVAL_S": "0",
        "KRT_DDS_BIND_ADDRESS": "10.168.1.100",
        "KRT_REQUIRE_GLOBAL_NETWORK": "yes",
    }
    result = subprocess.run(
        ["/bin/bash", str(READINESS_SCRIPT), "x86"],
        capture_output=True,
        check=False,
        env=environment,
        text=True,
    )

    assert result.returncode == 1
    assert "global_network=connecting" in result.stdout


def test_x86_readiness_rejects_a_can_bus_without_received_frames(tmp_path):
    """An Error-Passive arm bus must not be treated as ready merely because it exists."""
    fake_ip = tmp_path / "ip"
    fake_ip.write_text(
        "#!/bin/bash\n"
        "if [[ \"$*\" == *\"-o -4 addr show\"* ]]; then\n"
        "  printf '%s\\n' '2: eno1    inet 10.168.1.100/24 brd 10.168.1.255'\n"
        "else\n"
        "  printf '%s\\n' 'can state ERROR-PASSIVE' '    RX:  bytes packets errors dropped' '       0 0 0 0'\n"
        "fi\n",
        encoding="utf-8",
    )
    fake_ip.chmod(0o755)

    environment = os.environ | {
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "KRT_STARTUP_TIMEOUT_S": "0",
        "KRT_WAIT_INTERVAL_S": "0",
        "KRT_DDS_BIND_ADDRESS": "10.168.1.100",
        "KRT_REQUIRE_GLOBAL_NETWORK": "no",
        "KRT_CAN_REQUIRED_CHANNELS": "can_right",
    }
    result = subprocess.run(
        ["/bin/bash", str(READINESS_SCRIPT), "x86"],
        capture_output=True,
        check=False,
        env=environment,
        text=True,
    )

    assert result.returncode == 1
    assert "can_right=state:ERROR-PASSIVE,rx_packets:0" in result.stdout


def test_x86_readiness_uses_the_cyclonedds_xml_address_when_not_overridden(tmp_path):
    """Existing deployments remain bootable before their private env file is updated."""
    fake_ip = tmp_path / "ip"
    fake_ip.write_text(
        "#!/bin/bash\n"
        "printf '%s\\n' '2: eno1    inet 10.168.1.100/24 brd 10.168.1.255'\n",
        encoding="utf-8",
    )
    fake_ip.chmod(0o755)
    cyclone_config = tmp_path / "x86.xml"
    cyclone_config.write_text(
        '<NetworkInterface address="10.168.1.100"/>\n', encoding="utf-8"
    )

    environment = os.environ | {
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "KRT_STARTUP_TIMEOUT_S": "0",
        "KRT_WAIT_INTERVAL_S": "0",
        "KRT_DDS_BIND_ADDRESS": "",
        "CYCLONEDDS_URI": f"file://{cyclone_config}",
        "KRT_REQUIRE_GLOBAL_NETWORK": "no",
        "KRT_CAN_REQUIRED_CHANNELS": "",
    }
    result = subprocess.run(
        ["/bin/bash", str(READINESS_SCRIPT), "x86"],
        capture_output=True,
        check=False,
        env=environment,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "DDS bind address, global network, and required CAN buses ready" in result.stdout


def test_jetson_readiness_requires_camera_aliases_and_realsense(tmp_path):
    """A ready Jetson requires both hand aliases and an enumerable D435."""
    left_camera = tmp_path / "camera_left"
    right_camera = tmp_path / "camera_right"
    left_camera.touch()
    right_camera.touch()
    realsense_probe = tmp_path / "rs-enumerate-devices"
    realsense_probe.write_text(
        "#!/bin/bash\nprintf '%s\\n' 'Device Name: Intel RealSense D435'\n",
        encoding="utf-8",
    )
    realsense_probe.chmod(0o755)

    environment = os.environ | {
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "KRT_STARTUP_TIMEOUT_S": "0",
        "KRT_WAIT_INTERVAL_S": "0",
        "KRT_CAMERA_LEFT_PATH": str(left_camera),
        "KRT_CAMERA_RIGHT_PATH": str(right_camera),
    }
    result = subprocess.run(
        ["/bin/bash", str(READINESS_SCRIPT), "jetson"],
        capture_output=True,
        check=False,
        env=environment,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "startup mode=jetson timeout_s=0" in result.stdout
    assert f"left_camera={left_camera} readable=yes" in result.stdout
    assert f"right_camera={right_camera} readable=yes" in result.stdout
    assert "realsense_d435=ready" in result.stdout
    assert "Jetson cameras ready" in result.stdout


def test_user_services_run_readiness_checks_before_ros_and_keep_retrying():
    """A slow network or USB bus must delay ROS instead of exhausting retries."""
    x86_source = X86_UNIT.read_text(encoding="utf-8")
    jetson_source = JETSON_UNIT.read_text(encoding="utf-8")

    assert 'ExecStartPre=/bin/bash -lc \'exec /bin/bash "$KRT_WORKSPACE/deploy/systemd/krt-wait-ready.sh" x86\'' in x86_source
    assert 'ExecStartPre=/bin/bash -lc \'exec /bin/bash "$KRT_WORKSPACE/deploy/systemd/krt-wait-ready.sh" jetson\'' in jetson_source
    assert "StartLimitIntervalSec=0" in x86_source
    assert "StartLimitIntervalSec=0" in jetson_source


def test_x86_deployment_requires_global_network_without_blocking_on_one_arm():
    """Cloud voice must wait for global networking, but one arm must not block it."""
    source = X86_ENV_EXAMPLE.read_text(encoding="utf-8")

    assert "KRT_REQUIRE_GLOBAL_NETWORK=yes" in source
    assert "KRT_CAN_REQUIRED_CHANNELS=" in source
