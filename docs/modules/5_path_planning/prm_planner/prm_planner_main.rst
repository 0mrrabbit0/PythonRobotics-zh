.. _probabilistic-road-map-(prm)-planning:

概率路线图（PRM）规划
-------------------------------------

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ProbabilisticRoadMap/animation.gif

该 PRM 规划器使用 Dijkstra 方法做图搜索。

动画中，蓝点是采样点，

青色叉号是用 Dijkstra 方法搜索过的点，

红线是 PRM 的最终路径。

代码链接
~~~~~~~~~~~~~~~

.. autofunction:: PathPlanning.ProbabilisticRoadMap.probabilistic_road_map.prm_planning


参考文献
~~~~~~~~~~~

-  `Probabilistic roadmap -
   Wikipedia <https://en.wikipedia.org/wiki/Probabilistic_roadmap>`__
