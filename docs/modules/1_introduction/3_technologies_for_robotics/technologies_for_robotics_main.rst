机器人技术
-------------------------

机器人学领域需要广泛的技术，例如机械工程、
电气工程、计算机科学和人工智能（AI）。
本项目 `PythonRobotics` 只聚焦于计算机科学和人工智能。

机器人技术分为以下 3 类：

#. `自主导航 <Autonomous Navigation_>`_
#. `操作 <Manipulation_>`_
#. `特定机器人类型的技术 <Robot type specific technologies_>`_

.. _`Autonomous Navigation`:

自主导航
^^^^^^^^^^^^^^^^^^^^^^^^
自主导航是一种能力：机器人能够在没有操作员外部控制的情况下，长时间运动到目标。

要实现自主导航，机器人需要具备以下技术：
- 它需要知道自己在哪里（定位）
- 哪里是安全的（建图）
- 哪里是安全的，以及机器人在地图中的位置（同步定位与建图（SLAM））
- 往哪里走以及如何走（路径规划）
- 如何控制其运动（路径跟踪）。

如果缺少其中任何一项技术，自主系统就无法正确工作。

近年来，自主导航技术在许多领域受到巨大关注。
例如，自动驾驶汽车、无人机，以及室内外环境中的自主移动机器人。

在本项目中，我们为自主导航提供许多算法、示例代码
和文档。

#. :ref:`定位 <Localization>`
#. :ref:`建图 <Mapping>`
#. :ref:`同步定位与建图（SLAM） <SLAM>`
#. :ref:`路径规划 <Path planning>`
#. :ref:`路径跟踪 <Path tracking>`



.. _`Manipulation`:

操作
^^^^^^^^^^^^^^^^^^^^^^^^

#. :ref:`机械臂导航 <Arm Navigation>`

.. _`Robot type specific technologies`:

特定机器人类型的技术
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. :ref:`空中导航 <Aerial Navigation>`
#. :ref:`双足 <Bipedal>`
#. :ref:`倒立摆 <Inverted Pendulum>`


补充信息
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. :ref:`工具 <utils>`
#. :ref:`附录 <Appendix>`

