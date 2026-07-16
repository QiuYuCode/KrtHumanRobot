from pathlib import Path
from xml.etree import ElementTree


project = "KrtHumanRobot"
author = "KrtHumanRobot Contributors"

package_xml = Path(__file__).resolve().parents[1] / "src/krt_human_robot/package.xml"
release = ElementTree.parse(package_xml).findtext("version") or "0.0.0"
version = release

extensions = ["sphinx.ext.graphviz", "sphinx_rtd_theme"]
language = "zh_CN"
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

html_theme = "sphinx_rtd_theme"
html_title = "KrtHumanRobot 文档"
html_theme_options = {
    "collapse_navigation": False,
    "sticky_navigation": True,
    "navigation_depth": 4,
    "includehidden": True,
    "titles_only": False,
}

graphviz_output_format = "svg"
numfig = True
