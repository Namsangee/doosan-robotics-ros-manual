# -- Project information -----------------------------------------------------
project = 'Doosan Robotics ROS2 Manual'
author = 'ms'
copyright = '2025, ms'
version = '1.0'
release = '1.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.todo',
    'sphinx.ext.viewcode',
    'sphinx_multiversion',     # 멀티버전
    'sphinx.ext.githubpages',  # .nojekyll 등 (있어도 되고 없어도 됨)
]

templates_path = ['_templates']
exclude_patterns = ['_build', '_site', 'Thumbs.db', '.DS_Store']

rst_prolog = """
.. |br| raw:: html

   <br />
"""

import os
import re
import subprocess

def _build_smv_branch_whitelist():
    """
    Include ALL remote branches from origin (CI safe).
    """
    try:
        out = subprocess.check_output(
            ["git", "branch", "-r", "--format", "%(refname:short)"],
            text=True,
        )
    except Exception:
        return r"^(humble|jazzy)$"

    branches = []
    for line in out.splitlines():
        name = line.strip()
        if not name.startswith("origin/"):
            continue
        name = name.replace("origin/", "")
        if name in ("HEAD",):
            continue
        branches.append(name)

    if not branches:
        return r"^(humble|jazzy)$"

    escaped = [re.escape(b) for b in branches]
    return r"^(" + "|".join(escaped) + r")$"


templates_path = ['_templates']
exclude_patterns = ['_build', '_site', 'Thumbs.db', '.DS_Store']

smv_remote_whitelist = r'^origin$'

smv_branch_whitelist = _build_smv_branch_whitelist()

smv_tag_whitelist = r'^$'

smv_latest_version = 'jazzy'

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_css_files = ['manual.css']
html_title = 'ROS2 Manual Guide v1.0'
html_logo = 'tutorials/images/etc/Doosan_logo.png'

html_sidebars = {
    "**": [
        "globaltoc.html",
        "relations.html",
        "searchbox.html",
        "versions.html",
    ],
}

html_theme_options = {
    "collapse_navigation": False,
    "sticky_navigation": True,
    "navigation_depth": 4,
    "titles_only": False,
}