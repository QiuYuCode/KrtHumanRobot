"""Contract tests for fixed-peer CycloneDDS deployment configs."""

from pathlib import Path
import xml.etree.ElementTree as ET


WORKSPACE = Path(__file__).parents[3]
CONFIG_DIR = WORKSPACE / "deploy" / "cyclonedds"
NAMESPACE = {"c": "https://cdds.io/config"}


def _assert_config(name: str, local_address: str, remote_address: str) -> None:
    root = ET.parse(CONFIG_DIR / name).getroot()

    interface = root.find(
        "c:Domain/c:General/c:Interfaces/c:NetworkInterface", NAMESPACE
    )
    assert interface is not None
    assert interface.attrib["address"] == local_address
    assert root.find(
        "c:Domain/c:General/c:NetworkInterfaceAddress", NAMESPACE
    ) is None

    peers = {
        peer.attrib["address"]
        for peer in root.findall(
            "c:Domain/c:Discovery/c:Peers/c:Peer", NAMESPACE
        )
    }
    assert peers == {"localhost", remote_address}
    assert root.findtext(
        "c:Domain/c:Discovery/c:ParticipantIndex", namespaces=NAMESPACE
    ) == "auto"
    assert root.findtext(
        "c:Domain/c:Discovery/c:MaxAutoParticipantIndex", namespaces=NAMESPACE
    ) == "20"


def test_x86_cyclonedds_config_supports_local_and_remote_discovery():
    """The x86 config discovers local processes and the fixed Jetson peer."""
    _assert_config("x86.xml", "10.168.1.100", "10.168.1.101")


def test_jetson_cyclonedds_config_supports_local_and_remote_discovery():
    """The Jetson config discovers local processes and the fixed x86 peer."""
    _assert_config("jetson.xml", "10.168.1.101", "10.168.1.100")
