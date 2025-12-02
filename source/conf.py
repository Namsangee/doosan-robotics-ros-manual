project = 'Doosan Robotics ROS2 Manual'
author = 'ms'
copyright = '2025, ms'
version = '1.0'
release = '1.0'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.todo',
    'sphinx.ext.viewcode',
    'sphinx_multiversion',
    'sphinx.ext.githubpages',
]

templates_path = ['_templates']
exclude_patterns = ['_build', '_site', 'Thumbs.db', '.DS_Store']

rst_prolog = """
.. |br| raw:: html

   <br />
"""

import re
import subprocess

# -------------------------------------------------
# ✅ SMV 브랜치 자동 감지 + 최신 자동 지정 (단일 로직)
# -------------------------------------------------
def setup_smv_from_origin():
    try:
        out = subprocess.check_output(
            ["git", "branch", "-r", "--format", "%(refname:short)"],
            text=True,
        )
    except Exception:
        branches = ["jazzy", "humble"]
    else:
        branches = []
        for line in out.splitlines():
            name = line.strip()
            if not name.startswith("origin/"):
                continue
            name = name.replace("origin/", "")
            if name == "HEAD":
                continue
            branches.append(name)

        if not branches:
            branches = ["jazzy", "humble"]

    # ✅ 알파벳 역순 정렬 (Z → A)
    branches = sorted(set(branches), reverse=True)

    # ✅ whitelist 정규식 생성
    escaped = [re.escape(b) for b in branches]
    whitelist = r"^(" + "|".join(escaped) + r")$"

    # ✅ 최신 버전 = 역순 첫 번째
    latest = branches[0]

    return whitelist, latest


# ✅ SMV 설정 적용
smv_branch_whitelist, smv_latest_version = setup_smv_from_origin()
smv_remote_whitelist = r'^origin$'
smv_tag_whitelist = r'^$'


# -------------------------------------------------
# ✅ 브랜치별 프로젝트 제목 자동 변경
# -------------------------------------------------
def smv_rewrite_configs(app, config):
    if app.config.smv_current_version:
        app.config.project = f'Doosan ROS2 Manual ({app.config.smv_current_version})'


# -------------------------------------------------
# ✅ 브랜치별 GitHub 링크 자동 변경
# -------------------------------------------------
def github_link_rewrite_branch(app, pagename, templatename, context, doctree):
    if app.config.smv_current_version:
        context['github_version'] = app.config.smv_current_version


# -------------------------------------------------
# ✅ Sphinx hook 등록
# -------------------------------------------------
def setup(app):
    app.connect('config-inited', smv_rewrite_configs)
    app.connect('html-page-context', github_link_rewrite_branch)
    app.add_config_value('smv_eol_versions', [], 'html')


# -------------------------------------------------
# ✅ HTML 테마 설정
# -------------------------------------------------
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
