状态格规划
----------------------

这段脚本用状态格规划做路径规划。

代码使用模型预测轨迹生成器来求解边界值问题。


均匀极坐标采样
~~~~~~~~~~~~~~~~~~~~~~

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/StateLatticePlanner/UniformPolarSampling.gif

代码链接
^^^^^^^^^^^^^

.. autofunction:: PathPlanning.StateLatticePlanner.state_lattice_planner.calc_uniform_polar_states


有偏极坐标采样
~~~~~~~~~~~~~~~~~~~~~

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/StateLatticePlanner/BiasedPolarSampling.gif

代码链接
^^^^^^^^^^^^^
.. autofunction:: PathPlanning.StateLatticePlanner.state_lattice_planner.calc_biased_polar_states


车道采样
~~~~~~~~~~~~~

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/StateLatticePlanner/LaneSampling.gif

代码链接
^^^^^^^^^^^^^
.. autofunction:: PathPlanning.StateLatticePlanner.state_lattice_planner.calc_lane_states


参考文献
~~~~~~~~~~~~~~~

-  `Optimal rough terrain trajectory generation for wheeled mobile
   robots <https://journals.sagepub.com/doi/pdf/10.1177/0278364906075328>`__

-  `State Space Sampling of Feasible Motions for High-Performance Mobile
   Robot Navigation in Complex
   Environments <https://www.frc.ri.cmu.edu/~alonzo/pubs/papers/JFR_08_SS_Sampling.pdf>`__
