.. _`What is PythonRobotics?`:

什么是 PythonRobotics？
------------------------

这是一个开源软件（OSS）项目：PythonRobotics，即机器人算法的 Python 代码合集。
这些代码在 `MIT 许可证 <MIT license_>`_ 下开发，并托管于 `GitHub`_。

.. _GitHub: https://github.com/AtsushiSakai/PythonRobotics
.. _`MIT license`: https://github.com/AtsushiSakai/PythonRobotics/blob/master/LICENSE

本项目有以下三条主要理念：

理念 1. 易于理解每种算法的基本思想。
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

目标是让机器人学的初学者理解每种算法背后的基本思想。

本项目采用 `Python`_ 编程语言来实现这一目标。
Python 是一种易于阅读和编写的高级编程语言。
如果代码不易阅读，就很难实现让初学者理解算法这一目标。
Python 拥有出色的矩阵运算、数学与科学计算以及可视化库，
这些运算无需重新实现，因此代码更易读。
具备这些核心 Python 包后，使用者可以把注意力放在算法本身，
而不是具体实现上。

PythonRobotics 不仅提供代码，还提供直观的动画，
随时间展示每种算法的过程与行为。
下面是一个动画 GIF 示例，展示自动驾驶车辆在高速公路场景中的
路径规划过程。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/FrenetOptimalTrajectory/high_speed_and_velocity_keeping_frenet_path.gif

该动画是使用 `Matplotlib`_ 库制作的 GIF 文件。
所有动画 GIF 文件存放在 `PythonRoboticsGifs`_ 仓库中，
所有动画视频也上传到了这个 `YouTube 频道 <YouTube channel_>`_。

PythonRobotics 还提供一本讲解每种算法基本思想的教材。
借助 PythonRobotics 的示例代码与动画，
本教材让你可以用尽可能少的数学公式和文字说明，
学习机器人学中使用的基础算法。
与代码一样，本文档的内容在 PythonRobotics 的
`GitHub`_ 仓库中管理，并使用基于 Python 的文档构建工具 `Sphinx`_
转换为基于 HTML 的在线文档。
关于如何阅读本文档进行学习，请参阅“:ref:`如何阅读本教材 <How to read this textbook>`”一节。


.. _`Python`: https://www.python.org/
.. _`PythonRoboticsGifs`: https://github.com/AtsushiSakai/PythonRoboticsGifs
.. _`YouTube channel`: https://youtube.com/playlist?list=PL12URV8HFpCozuz0SDxke6b2ae5UZvIwa&si=AH2fNPPYufPtK20S


理念 2. 选取广泛使用且实用的算法。
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

第二条理念是：所实现的算法必须实用，
并且在学术界与工业界都被广泛使用。
我们相信，学习这些算法会在许多应用中有用。
例如，用于定位的卡尔曼滤波与粒子滤波，
用于建图的栅格建图，
用于路径规划的基于动态规划的方法与基于采样的方法，
以及用于路径跟踪的基于最优控制的方法。
这些算法都在本项目的教材中实现并加以讲解。

理念 3. 最少依赖。
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

PythonRobotics 的每个示例代码都用 Python3 编写，并且只依赖
若干标准模块，以便于阅读、安装与使用。


.. _`Requirements`:

依赖要求
============

要运行每个示例代码，PythonRobotics 只依赖以下五个库
（包括 Python 本身）。

-  `Python 3.12.x`_\ （Python 运行时）
-  `NumPy`_\ （矩阵运算）
-  `SciPy`_\ （科学计算）
-  `cvxpy`_\ （凸优化）
-  `Matplotlib`_\ （可视化）

如果只需要运行代码，上述五个库就足够了。
不过，进行代码开发或为教材编写文档时，
还需要以下额外的库。

-  `pytest`_\ （单元测试）
-  `pytest-xdist`_\ （并行单元测试）
-  `mypy`_\ （类型检查）
-  `Sphinx`_\ （文档生成）
-  `ruff`_\ （代码风格检查）

.. _`Python 3.12.x`: https://www.python.org/
.. _`NumPy`: https://numpy.org/
.. _`SciPy`: https://scipy.org/
.. _`Matplotlib`: https://matplotlib.org/
.. _`cvxpy`: https://www.cvxpy.org/
.. _`pytest`: https://docs.pytest.org/en/latest/
.. _`pytest-xdist`: https://github.com/pytest-dev/pytest-xdist
.. _`mypy`: https://mypy-lang.org/
.. _`Sphinx`: https://www.sphinx-doc.org/en/master/index.html
.. _`ruff`: https://github.com/astral-sh/ruff

关于如何安装上述库，请参阅
“:ref:`如何运行示例代码 <How to run sample codes>`”一节。

本项目的音频概览
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
关于本项目的音频概览，请参阅这段 `YouTube 视频 <YouTube video_>`_。

.. _`YouTube video`: https://www.youtube.com/watch?v=uMeRnNoJAfU

Arxiv 论文
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

我们于 2018 年在 Arxiv 上发表了关于本项目的论文。

有关本项目的更多细节见这篇论文：

- `PythonRobotics: a Python code collection of robotics algorithms`_\ （`BibTeX`_\）

.. _`PythonRobotics: a Python code collection of robotics algorithms`: https://arxiv.org/abs/1808.10703
.. _`BibTeX`: https://github.com/AtsushiSakai/PythonRoboticsPaper/blob/master/python_robotics.bib
