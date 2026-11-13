.. _`Path Tracking`:

路径跟踪
========

路径跟踪是机器人跟随路径规划器生成的参考路径，并同时稳定机器人的能力。路径跟踪控制器可能需要考虑建模误差以及其他形式的不确定性。在路径跟踪中，反馈控制技术与基于优化的控制技术被广泛使用[22]。图 6 给出了后轮反馈转向控制与 PID 速度控制，以及迭代线性模型预测路径跟踪控制的仿真[27]。

.. toctree::
   :maxdepth: 2
   :caption: 目录

   pure_pursuit_tracking/pure_pursuit_tracking
   stanley_control/stanley_control
   rear_wheel_feedback_control/rear_wheel_feedback_control
   lqr_steering_control/lqr_steering_control
   lqr_speed_and_steering_control/lqr_speed_and_steering_control
   model_predictive_speed_and_steering_control/model_predictive_speed_and_steering_control
   cgmres_nmpc/cgmres_nmpc
   move_to_a_pose/move_to_a_pose
