"""Runtime dependency contracts for the core robot package."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_XML = Path(__file__).parents[1] / "package.xml"


def test_core_declares_loguru_runtime_dependency():
    """The package declares the logger imported by core behavior modules."""
    root = ET.parse(PACKAGE_XML).getroot()
    dependencies = {
        element.text for element in root.findall("exec_depend")
    } | {element.text for element in root.findall("depend")}

    assert "python3-loguru" in dependencies
