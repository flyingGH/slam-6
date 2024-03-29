# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys
sys.path.insert(0, os.path.abspath('../'))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'slam-practice'
copyright = '2023, applejxd'
author = 'applejxd'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    # docstring 使用
    "sphinx.ext.autodoc",
    # ソースコードへの link を追加
    # ビルドが止まることがあるため注意 (その場合は一旦コメントアウトする)
    "sphinx.ext.viewcode",
    # 各種図生成
    "sphinx.ext.graphviz",
    # 継承関係図生成
    "sphinx.ext.inheritance_diagram",
    # Markdown 対応
    "sphinx_mdinclude"
]

# 型ヒントを有効
autodoc_typehints = 'description'
# __init__()も出力
autoclass_content = 'both'
autodoc_default_options = {
    # プライベートメソッドも出力
    'private-members': True,
    # 継承を表示
    'show-inheritance': True
}

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

language = 'ja'

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# # LuaLaTeXを使用
# latex_engine = 'lualatex'
# # LaTeXドキュメントクラスとしてltjsbookを使用
# latex_docclass = {'manual': 'ltjsarticle'}

latex_preamble = R'''
\usepackage{xltxtra}
\setmainfont{TakaoMincho}
\setsansfont{TakaoGothic}
\setmonofont{TakaoGothic}
\XeTeXlinebreaklocale "ja"
'''
latex_elements = {
    'extraclassoptions': 'openany',
}
