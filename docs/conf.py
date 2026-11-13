#
# Sphinx 文档构建器的配置文件。
#
# 本文件只包含最常用选项中的一部分。完整列表见文档：
# https://www.sphinx-doc.org/en/master/config

# -- 路径设置 --------------------------------------------------------------

# 如果要用 autodoc 文档化的扩展（或模块）位于另一个目录，
# 在此把这些目录加入 sys.path。如果目录相对于文档根目录，
# 请使用 os.path.abspath 将其变为绝对路径，如下所示。
#
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.abspath('../'))


# -- 项目信息 -----------------------------------------------------

project = 'PythonRobotics'
copyright = '2018-now, Atsushi Sakai'
author = 'Atsushi Sakai'

# 简短的 X.Y 版本
version = ''
# 完整版本，包括 alpha/beta/rc 标记
release = ''


# -- 通用配置 ---------------------------------------------------

# 如果文档需要最低 Sphinx 版本，在此写明。
#
# needs_sphinx = '1.0'

# 在此添加 Sphinx 扩展模块名，以字符串形式。它们可以是
# Sphinx 自带的扩展（名为 'sphinx.ext.*'），也可以是你自定义的
# 扩展。
extensions = [
    'matplotlib.sphinxext.plot_directive',
    'sphinx.ext.autodoc',
    'sphinx.ext.mathjax',
    'sphinx.ext.linkcode',
    'sphinx.ext.napoleon',
    'sphinx.ext.imgconverter',
    'IPython.sphinxext.ipython_console_highlighting',
    'sphinx_copybutton',
    'sphinx_rtd_dark_mode',
]

# 在此添加包含模板的路径，相对于本目录。
templates_path = ['_templates']

# 源文件名的后缀。
# 可以把多个后缀指定为字符串列表：
#
# source_suffix = ['.rst', '.md']
source_suffix = '_main.rst'

# 主 toctree 文档。
master_doc = 'index'

# Sphinx 自动生成内容所使用的语言。支持的语言列表见文档。
#
# 如果通过 gettext 目录做内容翻译，也会用到此项。
# 在这些情况下，通常从命令行设置 "language"。
language = "zh_CN"

# 相对于源目录的匹配模式列表，用于在查找源文件时忽略
# 文件和目录。
# 该模式也会影响 html_static_path 和 html_extra_path。
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# 要使用的 Pygments（语法高亮）风格名称。
pygments_style = 'sphinx'


# -- HTML 输出选项 -------------------------------------------------

# HTML 和 HTML Help 页面使用的主题。内置主题列表见文档。
#
# 针对 Read the Docs 的修复
on_rtd = os.environ.get('READTHEDOCS') == 'True'
if on_rtd:
    html_theme = 'default'
else:
    html_theme = 'sphinx_rtd_light_them'

# 主题选项依主题而定，用于进一步自定义主题的外观。
# 各主题可用选项列表见文档。
#
html_logo = '../icon.png'
html_theme_options = {
    'display_version': False,
}

# 在 Read the Docs 主题中，将 “view page source” 替换为 “edit on github”
#  * https://github.com/readthedocs/sphinx_rtd_theme/issues/529
html_context = {
    'display_github': True,
    'github_user': 'AtsushiSakai',
    'github_repo': 'PythonRobotics',
    'github_version': 'master',
    "conf_py_path": "/docs/",
    "source_suffix": source_suffix,
}

# 在此添加包含自定义静态文件（例如样式表）的路径，
# 相对于本目录。它们会在内置静态文件之后被复制，
# 因此名为 "default.css" 的文件会覆盖内置的 "default.css"。
html_static_path = ['_static']

html_css_files = ['custom.css']

# 自定义侧边栏模板，必须是把文档名映射到模板名的字典。
#
# 默认侧边栏（用于不匹配任何模式的文档）由主题自身定义。
# 内置主题默认使用这些模板：``['localtoc.html', 'relations.html', 'sourcelink.html',
# 'searchbox.html']``。
#
# html_sidebars = {}


# -- HTMLHelp 输出选项 ---------------------------------------------

# HTML help 构建器的输出文件基本名。
htmlhelp_basename = 'PythonRoboticsdoc'


# -- LaTeX 输出选项 ------------------------------------------------

latex_elements = {
    # 纸张大小（'letterpaper' 或 'a4paper'）。
    #
    # 'papersize': 'letterpaper',

    # 字号（'10pt'、'11pt' 或 '12pt'）。
    #
    # 'pointsize': '10pt',

    # LaTeX 导言区的附加内容。
    #
    # 'preamble': '',

    # LaTeX 图（浮动体）对齐方式
    #
    # 'figure_align': 'htbp',
}

# 将文档树分组为 LaTeX 文件。元组列表
# （源起始文件，目标名，标题，
#  作者，documentclass [howto、manual 或自定义类]）。
latex_documents = [
    (master_doc, 'PythonRobotics.tex', 'PythonRobotics Documentation',
     'Atsushi Sakai', 'manual'),
]


# -- 手册页输出选项 ------------------------------------------

# 每个手册页一项。元组列表
# （源起始文件，名称，描述，作者，手册章节）。
man_pages = [
    (master_doc, 'pythonrobotics', 'PythonRobotics Documentation',
     [author], 1)
]


# -- Texinfo 输出选项 ----------------------------------------------

# 将文档树分组为 Texinfo 文件。元组列表
# （源起始文件，目标名，标题，作者，
#  目录菜单项，描述，类别）
texinfo_documents = [
    (master_doc, 'PythonRobotics', 'PythonRobotics Documentation',
     author, 'PythonRobotics', 'One line description of project.',
     'Miscellaneous'),
]


# -- linkcode 设置 -------------------------------------------------

import inspect
import os
import sys
import functools

GITHUB_REPO = "https://github.com/AtsushiSakai/PythonRobotics"
GITHUB_BRANCH = "master"


def linkcode_resolve(domain, info):
    if domain != "py":
        return None

    modname = info["module"]
    fullname = info["fullname"]

    try:
        module = __import__(modname, fromlist=[fullname])
        obj = functools.reduce(getattr, fullname.split("."), module)
    except (ImportError, AttributeError):
        return None

    try:
        srcfile = inspect.getsourcefile(obj)
        srcfile = get_relative_path_from_parent(srcfile, "PythonRobotics")
        lineno = inspect.getsourcelines(obj)[1]
    except Exception:
        return None

    return f"{GITHUB_REPO}/blob/{GITHUB_BRANCH}/{srcfile}#L{lineno}"


def get_relative_path_from_parent(file_path: str, parent_dir: str):
    path = Path(file_path).resolve()

    try:
        parent_path = next(p for p in path.parents if p.name == parent_dir)
        return str(path.relative_to(parent_path))
    except StopIteration:
        raise ValueError(f"Parent directory '{parent_dir}' not found in {file_path}")
