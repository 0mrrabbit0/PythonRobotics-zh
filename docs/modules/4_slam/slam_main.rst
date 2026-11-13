.. _`SLAM`:

同步定位与建图（SLAM）
======================

同步定位与建图（SLAM）示例。

同步定位与建图（SLAM）是同时估计机器人位姿与环境地图的能力。SLAM 问题难以求解，因为定位需要地图，而建图又需要定位。因此，SLAM 常被比作“先有鸡还是先有蛋”的问题。常用的 SLAM 求解方法包括扩展卡尔曼滤波（EKF）、粒子滤波以及 FastSLAM 算法[31]。图 4 给出使用扩展卡尔曼滤波（EKF）的 SLAM 仿真结果，以及使用 FastSLAM 2.0 的结果[31]。

.. toctree::
   :maxdepth: 2
   :caption: 目录

   iterative_closest_point_matching/iterative_closest_point_matching
   ekf_slam/ekf_slam
   FastSLAM1/FastSLAM1
   FastSLAM2/FastSLAM2
   graph_slam/graph_slam
