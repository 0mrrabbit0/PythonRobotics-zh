## Python Robotics 文档

本目录包含 Python Robotics 项目的文档。


### 构建文档

#### 安装 Sphinx 与主题

```
pip install sphinx sphinx-autobuild sphinx-rtd-theme sphinx_rtd_dark_mode sphinx_copybutton sphinx_rtd_dark_mode
```

#### 构建文档

在 `docs/` 目录中：
```
make html
```

如果希望在文件每次更改时重新构建：

```
sphinx-autobuild . _build/html
```

#### 查看生成的文档

打开 docs/_build/ 下的 index.html 文件
