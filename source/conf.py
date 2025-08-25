

project = 'Doosan Robotics ROS2 Manual'
copyright = '2025, ms'
author = 'ms'
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

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

html_css_files = ['manual.css']

html_title = 'ROS2 Manual Guide v1.0'
html_logo = '_static/doosan_logo.png'

html_sidebars = {
    "**": [
        "localtoc.html",
        "relations.html",
        "searchbox.html", 
        "versions.html",  
    ],
}

html_theme_options = {
    'collapse_navigation': False,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'titles_only': False,
}

html_context = {
    'display_lower_left': True,
}