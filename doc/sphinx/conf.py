#!/usr/bin/env python3

from pathlib import Path
import re


CONF_DIR = Path(__file__).absolute().parent
"""conf.py directory."""

with open(CONF_DIR / ".." / "nrfx.doxyfile") as f:
    VERSION = re.search(r'PROJECT_NUMBER\s+=\s+"(.*)"', f.read()).group(1)

# General configuration --------------------------------------------------------

project = "nrfx"
copyright = "2021, Nordic Semiconductor ASA"
author = "Nordic Semiconductor"
version = VERSION

extensions = ["breathe", "m2r2"]
source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown"
}
master_doc = "index"
exclude_patterns = ["theme"]

# Options for HTML output ------------------------------------------------------

html_theme = "sphinx_ncs_theme"
html_static_path = [str(CONF_DIR / "_static")]
html_last_updated_fmt = "%b %d, %Y"
html_show_sphinx = False

# Options for Breathe ----------------------------------------------------------

breathe_projects = {"nrfx": str(CONF_DIR / ".." / "xml")}
breathe_default_project = "nrfx"
breathe_domain_by_extension = {"h": "c", "c": "c"}
breathe_separate_member_pages = True

c_id_attributes = [
    "NRF_STATIC_INLINE",
    "NRFX_STATIC_INLINE",
    "__STATIC_INLINE",
]
cpp_id_attributes = c_id_attributes


def setup(app):
    app.add_css_file("css/nrfx.css")
