Voronoi 路线图路径规划
-------------------------

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/VoronoiRoadMap/animation.gif

该 Voronoi 路线图规划器使用 Dijkstra 方法做图搜索。

动画中，蓝点是 Voronoi 点，

青色叉号是用 Dijkstra 方法搜索过的点，

红线是 Voronoi 路线图的最终路径。

代码链接
~~~~~~~~~~~~~~~
.. autoclass:: PathPlanning.VoronoiRoadMap.voronoi_road_map.VoronoiRoadMapPlanner


参考文献
~~~~~~~~~~~~

-  `Robotic Motion Planning <https://www.cs.cmu.edu/~motionplanning/lecture/Chap5-RoadMap-Methods_howie.pdf>`__
