.. _`Mapping`:

建图
======
建图是机器人借助激光雷达和相机等外部传感器理解周围环境的能力。机器人必须识别障碍物的位置与形状，才能避开它们。在建图中，栅格建图与机器学习算法被广泛使用[31][18]。图3展示了使用二维射线投射进行栅格建图，以及使用 k-均值算法进行二维目标聚类的建图仿真结果。

.. toctree::
   :maxdepth: 2
   :caption: 目录

   gaussian_grid_map/gaussian_grid_map
   ndt_map/ndt_map
   ray_casting_grid_map/ray_casting_grid_map
   lidar_to_grid_map_tutorial/lidar_to_grid_map_tutorial
   point_cloud_sampling/point_cloud_sampling
   k_means_object_clustering/k_means_object_clustering
   circle_fitting/circle_fitting
   rectangle_fitting/rectangle_fitting
   normal_vector_estimation/normal_vector_estimation
   distance_map/distance_map
