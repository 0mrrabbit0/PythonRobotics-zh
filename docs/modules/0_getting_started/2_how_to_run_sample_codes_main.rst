.. _`How to run sample codes`:

如何运行示例代码
-------------------------

本章将说明运行 PythonRobotics 中各个示例代码的环境配置过程，
并介绍每个目录的内容。

配置并运行示例代码的步骤
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. 安装 `Python 3.12.x`_

请注意，更早的 Python3 版本也许可以运行，但我们建议使用
指定版本，因为示例代码只在该版本上测试过。

2. 如果希望使用 conda 管理软件包，请安装
Anaconda 或 Miniconda，以便使用 `conda`_ 命令。

3. 克隆本仓库并进入目录。

.. code-block::

    >$ git clone https://github.com/AtsushiSakai/PythonRobotics.git

    >$ cd PythonRobotics


4. 安装所需的库。

我们在 requirements 目录下准备了用于 `conda`_ 和 `pip`_ 的
依赖管理文件。使用这些文件可以方便地安装所需的库。

使用 conda：

.. code-block::

    >$ conda env create -f requirements/environment.yml

使用 pip：

.. code-block::

    >$ pip install -r requirements/requirements.txt


5. 在每个目录中执行 Python 脚本。

例如，要运行 `localization` 目录中扩展卡尔曼滤波（EKF）的示例代码，
执行以下命令：

.. code-block::

    >$ cd localization/extended_kalman_filter

	>$ python extended_kalman_filter.py

然后可以看到基于扩展卡尔曼滤波（EKF）算法的定位动画：

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/extended_kalman_filter/animation.gif

各目录内容的更多细节请参阅 `目录结构 <Directory structure_>`_ 一节。

6. 如果喜欢本仓库，请为它加一颗 star 😃。

.. _`Python 3.12.x`: https://www.python.org/
.. _`conda`: https://docs.conda.io/projects/conda/en/stable/user-guide/install/index.html
.. _`pip`: https://pip.pypa.io/en/stable/
.. _`the requirements directory`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/requirements

.. _`Directory structure`:

目录结构
~~~~~~~~~~~~~~~~~~~~

PythonRobotics 的顶层目录结构如下：

示例代码目录：

- `AerialNavigation`_：无人机与火箭着陆的空中导航算法示例代码。
- `ArmNavigation`_：机械臂导航算法的示例代码。
- `Localization`_：定位算法的示例代码。
- `Bipedal`_：足式机器人双足行走算法的示例代码。
- `Control`_：机器人系统控制算法的示例代码。
- `Mapping`_：建图或障碍物形状识别算法的示例代码。
- `PathPlanning`_：路径规划算法的示例代码。
- `PathTracking`_：类车机器人路径跟踪算法的示例代码。
- `SLAM`_：同步定位与建图（SLAM）算法的示例代码。

其他目录：

- `docs`_：本目录包含 PythonRobotics 的文档。
- `requirements`_：本目录包含依赖管理文件。
- `tests`_：本目录包含单元测试文件。
- `utils`_：本目录包含若干示例代码共用的工具函数。
- `.github`_：本目录包含 GitHub Actions 配置文件。
- `.circleci`_：本目录包含 CircleCI 配置文件。

本文档的结构与上述示例代码目录的结构相同。
更多细节请参阅 :ref:`如何阅读本教材 <How to read this textbook>` 一节。


.. _`AerialNavigation`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/AerialNavigation
.. _`ArmNavigation`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/ArmNavigation
.. _`Localization`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/Localization
.. _`Bipedal`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/Bipedal
.. _`Control`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/Control
.. _`Mapping`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/Mapping
.. _`PathPlanning`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning
.. _`PathTracking`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking
.. _`SLAM`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/SLAM
.. _`docs`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/docs
.. _`requirements`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/requirements
.. _`tests`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/tests
.. _`utils`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/utils
.. _`.github`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/.github
.. _`.circleci`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/.circleci
