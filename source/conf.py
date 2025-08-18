# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Doosan Robotics ROS2 Manual'
copyright = '2025, ms'
author = 'ms'
version = '1.0'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',     # 자동 문서화: docstring 기반
    'sphinx.ext.napoleon',    # 구글/넘피 스타일 docstring 파싱
    'sphinx.ext.todo',        # TODO 지원 (.. todo::)
    'sphinx.ext.viewcode'     # 소스코드 보기 링크
]


templates_path = ['_templates']
exclude_patterns = []

rst_prolog = """
.. |br| raw:: html

   <br />
"""

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# Add custom CSS
html_css_files = [
    'manual.css',
]

# 문서 제목 변경
html_title = 'ROS2 Manual Guide v1.0'
html_logo = 'tutorials/images/etc/Doosan_logo.png' # logo
# html_favicon = '_static/favicon.ico'

html_sidebars = {
    '**': [
        'globaltoc.html',
        'relations.html',
        'searchbox.html'
    ]
}