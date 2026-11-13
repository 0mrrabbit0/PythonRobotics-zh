基于栅格的搜索
-----------------

广度优先搜索
~~~~~~~~~~~~~~~~~~~~

这是用广度优先搜索算法在二维栅格上做路径规划。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/BreadthFirstSearch/animation.gif

动画中，青色点是已搜索节点。

代码链接
+++++++++++++

.. autofunction:: PathPlanning.BreadthFirstSearch.breadth_first_search.BreadthFirstSearchPlanner


深度优先搜索
~~~~~~~~~~~~~~~~~~~~

这是用深度优先搜索算法在二维栅格上做路径规划。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DepthFirstSearch/animation.gif

动画中，青色点是已搜索节点。

代码链接
+++++++++++++

.. autofunction:: PathPlanning.DepthFirstSearch.depth_first_search.DepthFirstSearchPlanner


.. _dijkstra:

Dijkstra 算法
~~~~~~~~~~~~~~~~~~

这是用 Dijkstra 算法在二维栅格上做最短路径规划。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/Dijkstra/animation.gif

动画中，青色点是已搜索节点。

代码链接
+++++++++++++

.. autofunction:: PathPlanning.Dijkstra.dijkstra.DijkstraPlanner


.. _a*-algorithm:

A\* 算法
~~~~~~~~~~~~~

这是用 A\* 算法在二维栅格上做最短路径规划。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/AStar/animation.gif

动画中，青色点是已搜索节点。

启发式函数是二维欧氏距离。

代码链接
+++++++++++++

.. autofunction:: PathPlanning.AStar.a_star.AStarPlanner


双向 A\* 算法
~~~~~~~~~~~~~~~~~~~~~~~~~~~

这是用双向 A\* 算法在二维栅格上做最短路径规划。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/BidirectionalAStar/animation.gif

动画中，青色点是已搜索节点。

代码链接
+++++++++++++

.. autofunction:: PathPlanning.BidirectionalAStar.bidirectional_a_star.BidirectionalAStarPlanner


.. _Theta*-algorithm:

Theta\* 算法
~~~~~~~~~~~~~~~~~

这是用 Theta\* 算法在二维栅格上做最短路径规划。

它对 A\* 算法做了优化，用以生成任意角度路径。A\* 总是把当前节点指定为后继节点的父节点；Theta\* 则检查从更早的节点（通常是当前节点的父节点）到后继节点是否存在视线（未被阻挡的路径），若可见就直接把它指定为父节点。这样用直线代替折线路径段，从而降低代价。

视线检查要验证两个节点之间的直线是否被障碍物挡住。在栅格上，这表示找出直线经过的全部离散单元格，并判断它们是否为空。Bresenham 直线算法常用于此。它从一端点出发，沿一个轴逐步前进，同时根据斜率确定另一轴上的位置。由于只做整数加减，它在 Theta\* 的栅格可见性检查中既高效又精确。

因此，Theta\* 得到的路径比 A\* 更短、更平滑，适合在连续环境中运行的地面或空中机器人。更平滑的运动可以带来更高的加速度和更短的行驶时间。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ThetaStar/animation.gif

动画中，每个青色箭头表示一个指向其父节点的节点。

参考文献
++++++++++++

-  `Theta*: Any-Angle Path Planning on Grids <https://cdn.aaai.org/AAAI/2007/AAAI07-187.pdf>`__
-  `Bresenham's line algorithm <https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm>`__

代码链接
+++++++++++++

.. autofunction:: PathPlanning.ThetaStar.theta_star.ThetaStarPlanner


.. _D*-algorithm:

D\* 算法
~~~~~~~~~~~~~

这是用 D\* 算法在二维栅格上做最短路径规划。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DStar/animation.gif

动画展示机器人用 D\* 搜索算法寻找路径并绕开障碍物。

代码链接
+++++++++++++

.. autoclass:: PathPlanning.DStar.dstar.Dstar


参考文献
++++++++++++

-  `D* search Wikipedia <https://en.wikipedia.org/wiki/D*>`__

D\* Lite 算法
~~~~~~~~~~~~~~~~~~

这是用 D\* Lite 算法在二维栅格上做路径规划与重规划。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DStarLite/animation.gif

代码链接
+++++++++++++

.. autoclass:: PathPlanning.DStarLite.d_star_lite.DStarLite

参考文献
++++++++++++

- `Improved Fast Replanning for Robot Navigation in Unknown Terrain <http://www.cs.cmu.edu/~maxim/files/dlite_icra02.pdf>`_


人工势场算法
~~~~~~~~~~~~~~~~~~~~~~~~~

这是人工势场算法的二维栅格路径规划。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/PotentialFieldPlanning/animation.gif

动画中，蓝色热力图表示每个栅格上的势场值。

代码链接
+++++++++++++

.. autofunction:: PathPlanning.PotentialFieldPlanning.potential_field_planning.potential_field_planning


参考文献
++++++++++++

-  `Robotic Motion Planning:Potential
   Functions <https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf>`__
