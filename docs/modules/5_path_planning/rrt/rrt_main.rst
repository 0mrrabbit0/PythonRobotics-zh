.. _rapidly-exploring-random-trees-(rrt):

快速扩展随机树（RRT）
------------------------------------

基本 RRT
~~~~~~~~~

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/RRT/animation.gif

这是使用快速扩展随机树（RRT）的简单路径规划代码。

黑圆是障碍物，绿线是搜索树，红叉是起点和目标点。

代码链接
^^^^^^^^^^

.. autoclass:: PathPlanning.RRT.rrt.RRT


.. include:: rrt_star.rst


带 Dubins 路径的 RRT
~~~~~~~~~~~~~~~~~~~~

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/RRTDubins/animation.gif

面向汽车机器人的路径规划，使用 RRT 与 Dubins 路径规划器。

代码链接
^^^^^^^^^^

.. autoclass:: PathPlanning.RRTDubins.rrt_dubins.RRTDubins


.. _rrt*-with-dubins-path:

带 Dubins 路径的 RRT\*
~~~~~~~~~~~~~~~~~~~~~~

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/RRTStarDubins/animation.gif

面向汽车机器人的路径规划，使用 RRT\* 与 Dubins 路径规划器。

代码链接
^^^^^^^^^^

.. autoclass:: PathPlanning.RRTStarDubins.rrt_star_dubins.RRTStarDubins


.. _rrt*-with-reeds-sheep-path:

结合 Reeds-Shepp 路径的 RRT\*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/RRTStarReedsShepp/animation.gif

面向汽车机器人的路径规划，使用 RRT\* 与 Reeds-Shepp 路径规划器。

代码链接
^^^^^^^^^^

.. autoclass:: PathPlanning.RRTStarReedsShepp.rrt_star_reeds_shepp.RRTStarReedsShepp


.. _informed-rrt*:

Informed RRT\*
~~~~~~~~~~~~~~

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/InformedRRTStar/animation.gif

这是使用 Informed RRT\* 的路径规划代码。

青色椭圆是 Informed RRT\* 的启发式采样区域。

代码链接
^^^^^^^^^^

.. autoclass:: PathPlanning.InformedRRTStar.informed_rrt_star.InformedRRTStar


参考文献
^^^^^^^^^^

-  `Informed RRT\*: Optimal Sampling-based Path Planning Focused via
   Direct Sampling of an Admissible Ellipsoidal
   Heuristic <https://arxiv.org/pdf/1404.2334>`__

.. _batch-informed-rrt*:

Batch Informed RRT\*
~~~~~~~~~~~~~~~~~~~~

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/BatchInformedRRTStar/animation.gif

这是使用 Batch Informed RRT\* 的路径规划代码。

代码链接
^^^^^^^^^^

.. autoclass:: PathPlanning.BatchInformedRRTStar.batch_informed_rrt_star.BITStar


参考文献
^^^^^^^^^^^

-  `Batch Informed Trees (BIT*): Sampling-based Optimal Planning via the
   Heuristically Guided Search of Implicit Random Geometric
   Graphs <https://arxiv.org/abs/1405.5848>`__


.. _closed-loop-rrt*:

闭环 RRT\*
~~~~~~~~~~~~~~~~~

基于车辆模型、使用闭环 RRT\* 的路径规划。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ClosedLoopRRTStar/animation.gif

本代码用纯追踪（pure pursuit）算法做转向控制，

用 PID 做速度控制。

代码链接
^^^^^^^^^^

.. autoclass:: PathPlanning.ClosedLoopRRTStar.closed_loop_rrt_star_car.ClosedLoopRRTStar


参考文献
^^^^^^^^^^^^

-  `Motion Planning in Complex Environments using Closed-loop
   Prediction <https://acl.mit.edu/papers/KuwataGNC08.pdf>`__

-  `Real-time Motion Planning with Applications to Autonomous Urban
   Driving <https://acl.mit.edu/papers/KuwataTCST09.pdf>`__

-  `[1601.06326] Sampling-based Algorithms for Optimal Motion Planning
   Using Closed-loop Prediction <https://arxiv.org/abs/1601.06326>`__


.. _lqr-rrt*:

LQR-RRT\*
~~~~~~~~~

这是 LQR-RRT\* 路径规划仿真。

局部规划器使用双积分器运动模型上的 LQR。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/LQRRRTStar/animation.gif

代码链接
^^^^^^^^^^

.. autoclass:: PathPlanning.LQRRRTStar.lqr_rrt_star.LQRRRTStar


参考文献
~~~~~~~~~~~~~

-  `LQR-RRT\*: Optimal Sampling-Based Motion Planning with Automatically
   Derived Extension
   Heuristics <https://lis.csail.mit.edu/pubs/perez-icra12.pdf>`__

-  `MahanFathi/LQR-RRTstar: LQR-RRT\* method is used for random motion planning of a simple pendulum in its phase plot <https://github.com/MahanFathi/LQR-RRTstar>`__
