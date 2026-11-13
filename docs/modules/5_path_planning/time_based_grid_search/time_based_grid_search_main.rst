基于时间的栅格搜索
----------------------

时空 A\*
~~~~~~~~~~~~~~~~~~~~~~

这是 A\* 算法的扩展，支持在动态障碍物周围进行规划。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/TimeBasedPathPlanning/SpaceTimeAStar/path_animation.gif

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/TimeBasedPathPlanning/SpaceTimeAStar/path_animation2.gif

与原始 A\* 相比，该算法的关键区别是代价和启发式现在基于时间，而不是基于距离。
使用基于时间的代价和启发式，可以保证找到的路径在到达目标点的时间上是最优的。

代价是到达给定节点所用的时间，启发式是从该节点到达目标点可能需要的最短时间，且不考虑任何障碍物。
对于机器人每个时间步移动 1 个栅格、并可随意停止和前进的简单情形，时间上的启发式等价于距离上的启发式。

在 `此 PR <https://github.com/AtsushiSakai/PythonRobotics/pull/1183>`__ 中增加的一项优化，是给算法加入已扩展集合。已经在该集合中的节点不会再被扩展。由于不会扩展重复节点，找到路径所需的节点扩展次数大大减少，算法占用的内存也会降低。

优化前::

    Found path to goal after 204490 expansions
    Planning took: 1.72464 seconds
    Memory usage (RSS): 68.19 MB


优化后::

    Found path to goal after 2348 expansions
    Planning took: 0.01550 seconds
    Memory usage (RSS): 64.85 MB

以上是起点为 (1, 11)、处于结构化障碍物布置（上面两个 gif 中的第二个）时的结果。

代码链接
^^^^^^^^^^^^^
.. autoclass:: PathPlanning.TimeBasedPathPlanning.SpaceTimeAStar.SpaceTimeAStar


安全区间路径规划
~~~~~~~~~~~~~~~~~~~~~~~~~~~

安全区间路径规划算法在这篇论文中有所描述：

`SIPP: Safe Interval Path Planning for Dynamic Environments <https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf>`__

它比时空 A\* 更快，因为它预先计算每个栅格未被占用的时间区间。这样就可以避免在同一区间内生成节点，从而减少生成的后继节点数量。

**与时空 A\* 的比较：**

布置 1，起点为 (1, 18)

安全区间规划器::

    Found path to goal after 322 expansions
    Planning took: 0.00730 seconds

时空 A\*::

    Found path to goal after 2717154 expansions
    Planning took: 20.51330 seconds

**安全区间路径规划器的基准测试：**

250 个随机障碍物::

    Found path to goal after 764 expansions
    Planning took: 0.60596 seconds

.. image:: https://raw.githubusercontent.com/AtsushiSakai/PythonRoboticsGifs/refs/heads/master/PathPlanning/TimeBasedPathPlanning/SafeIntervalPathPlanner/path_animation.gif

布置 1，起点为 (1, 18)::

    Found path to goal after 322 expansions
    Planning took: 0.00730 seconds

.. image:: https://raw.githubusercontent.com/AtsushiSakai/PythonRoboticsGifs/refs/heads/master/PathPlanning/TimeBasedPathPlanning/SafeIntervalPathPlanner/path_animation2.gif

代码链接
^^^^^^^^^^^^^
.. autoclass:: PathPlanning.TimeBasedPathPlanning.SafeInterval.SafeIntervalPathPlanner

多智能体路径规划
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

基于优先级的规划
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

规划器先生成规划顺序，再按该顺序为各机器人生成规划。每条已规划路径都会在栅格中预留，此后的所有规划都必须避开该路径。机器人按从起点到目标点的距离从大到小依次规划。

这是一种次优算法，因为路径一旦找到就不再重规划。它不保证为任何一个机器人找到最短路径，也不保证最终的路径集合在时间或移动量上最少。

静态障碍物：

.. image:: https://raw.githubusercontent.com/AtsushiSakai/PythonRoboticsGifs/refs/heads/master/PathPlanning/TimeBasedPathPlanning/PriorityBasedPlanner/priority_planner2.gif

动态障碍物：

.. image:: https://raw.githubusercontent.com/AtsushiSakai/PythonRoboticsGifs/refs/heads/master/PathPlanning/TimeBasedPathPlanning/PriorityBasedPlanner/priority_planner.gif


参考文献
~~~~~~~~~~~

-  `Cooperative Pathfinding <https://www.davidsilver.uk/wp-content/uploads/2020/03/coop-path-AIWisdom.pdf>`__
-  `SIPP: Safe Interval Path Planning for Dynamic Environments <https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf>`__
- `Prioritized Planning Algorithms for Trajectory Coordination of Multiple Mobile Robots <https://pure.tudelft.nl/ws/portalfiles/portal/67074672/07138650.pdf>`__
