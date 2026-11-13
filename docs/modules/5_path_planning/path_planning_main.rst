.. _`Path Planning`:

路径规划
=============

路径规划是机器人搜索通往目标点的可行且高效路径的能力。路径必须满足由机器人运动模型和障碍物位置决定的约束，并优化到达目标点的时间、与障碍物的距离等目标函数。在路径规划中，基于动态规划的方法和基于采样的方法都得到广泛使用[22]。图 5 给出了人工势场路径规划与 LQR-RRT\* 路径规划的仿真结果[27]。

.. toctree::
   :maxdepth: 2
   :caption: 目录

   dynamic_window_approach/dynamic_window_approach
   bugplanner/bugplanner
   grid_base_search/grid_base_search
   time_based_grid_search/time_based_grid_search
   model_predictive_trajectory_generator/model_predictive_trajectory_generator
   state_lattice_planner/state_lattice_planner
   prm_planner/prm_planner
   visibility_road_map_planner/visibility_road_map_planner
   vrm_planner/vrm_planner
   rrt/rrt
   particleSwarmOptimization/particleSwarmOptimization
   cubic_spline/cubic_spline
   bspline_path/bspline_path
   catmull_rom_spline/catmull_rom_spline
   clothoid_path/clothoid_path
   eta3_spline/eta3_spline
   bezier_path/bezier_path
   quintic_polynomials_planner/quintic_polynomials_planner
   dubins_path/dubins_path
   reeds_shepp_path/reeds_shepp_path
   lqr_path/lqr_path
   hybridastar/hybridastar
   frenet_frame_path/frenet_frame_path
   coverage_path/coverage_path
   elastic_bands/elastic_bands
