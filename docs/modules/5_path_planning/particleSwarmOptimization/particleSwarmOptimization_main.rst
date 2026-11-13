.. _particle_swarm_optimization:

粒子群优化（PSO）路径规划
------------------------------------------

这是用粒子群优化（PSO）算法做的二维路径规划仿真。

PSO 是受鸟群或鱼群社会行为启发的元启发式优化算法。在路径规划中，粒子表示潜在解，它们在搜索空间中寻找从起点到目标点的无碰撞路径。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ParticleSwarmOptimization/animation.gif

算法概述
++++++++++++++++++

PSO 算法维护一群粒子，它们按照简单的数学规则在搜索空间中运动：

1. **初始化**：粒子在起点附近随机分布
2. **评估**：根据到目标点的距离和障碍物惩罚计算每个粒子的适应度
3. **更新**：粒子根据下列量调整速度：
   - 个体最优位置（认知分量）
   - 全局最优位置（社会分量）
   - 当前速度（惯性分量）
4. **移动**：粒子移动到新位置并检查碰撞
5. **收敛**：重复该过程，直到达到最大迭代次数或到达目标点

数学基础
+++++++++++++++++++++++

PSO 的核心速度更新方程为：

.. math::

   v_{i}(t+1) = w \cdot v_{i}(t) + c_1 \cdot r_1 \cdot (p_{i} - x_{i}(t)) + c_2 \cdot r_2 \cdot (g - x_{i}(t))

其中：
- :math:`v_{i}(t)` = 粒子 i 在时刻 t 的速度
- :math:`x_{i}(t)` = 粒子 i 在时刻 t 的位置
- :math:`w` = 惯性权重（控制探索与利用）
- :math:`c_1` = 认知系数（对个体最优的吸引）
- :math:`c_2` = 社会系数（对全局最优的吸引）
- :math:`r_1, r_2` = [0, 1] 内的随机数
- :math:`p_{i}` = 粒子 i 的个体最优位置
- :math:`g` = 全局最优位置

位置更新：

.. math::

   x_{i}(t+1) = x_{i}(t) + v_{i}(t+1)

适应度函数
++++++++++++++++

适应度函数把到目标的距离与障碍物惩罚结合起来：

.. math::

   f(x) = ||x - x_{goal}|| + \sum_{j} P_{obs}(x, O_j)

其中：
- :math:`||x - x_{goal}||` = 到目标点的欧氏距离
- :math:`P_{obs}(x, O_j)` = 障碍物 j 的惩罚
- :math:`O_j` = 具有位置和半径的障碍物 j

障碍物惩罚函数定义为：

.. math::

   P_{obs}(x, O_j) = \begin{cases}
   1000 & \text{if } ||x - O_j|| < r_j \text{ (inside obstacle)} \\
   \frac{50}{||x - O_j|| - r_j + 0.1} & \text{if } r_j \leq ||x - O_j|| < r_j + R_{influence} \text{ (near obstacle)} \\
   0 & \text{if } ||x - O_j|| \geq r_j + R_{influence} \text{ (safe distance)}
   \end{cases}

其中：
- :math:`r_j` = 障碍物 j 的半径
- :math:`R_{influence}` = 影响半径（通常为 5 个单位）

碰撞检测
+++++++++++++++++++

用直线与圆的相交来检测粒子路径和圆形障碍物之间的碰撞：

.. math::

   ||P_0 + t \cdot \vec{d} - C|| = r

其中：
- :math:`P_0` = 路径段的起点
- :math:`\vec{d}` = 路径的方向向量
- :math:`C` = 障碍物中心
- :math:`r` = 障碍物半径
- :math:`t \in [0,1]` = 沿线段的参数

算法参数
++++++++++++++++++++

影响性能的关键参数：

- **粒子数量** (n_particles)：粒子越多，探索越好，但越慢
- **最大迭代次数** (max_iter)：迭代越多，收敛越好，但越慢
- **惯性权重** (w)：大表示探索，小表示利用
- **认知系数** (c1)：对个体最优的吸引
- **社会系数** (c2)：对全局最优的吸引

典型取值：
- n_particles：20–50
- max_iter：100–300
- w：0.9 → 0.4（线性递减）
- c1、c2：1.5–2.0

优点
++++++++++

- **全局优化**：可以跳出局部极小
- **不需要导数**：适用于不可微的适应度地形
- **并行探索**：多个粒子同时搜索
- **实现简单**：参数少，逻辑直接
- **灵活**：易于适配不同的环境和约束

缺点
+++++++++++++

- **随机性**：各次运行的结果可能不同
- **对参数敏感**：性能在很大程度上取决于参数调节
- **不保证最优**：属于元启发式方法，没有收敛性证明
- **计算代价**：需要大量适应度评估
- **容易停滞**：会过早收敛；若探索不足，整个群体可能陷在某个局部极小里

代码链接
+++++++++

.. autofunction:: PathPlanning.ParticleSwarmOptimization.particle_swarm_optimization.main

使用示例
+++++++++++++

.. code-block:: python

   import matplotlib.pyplot as plt
   from PathPlanning.ParticleSwarmOptimization.particle_swarm_optimization import main
   
   # Run PSO path planning with visualization
   main()

参考文献
++++++++++

- `Particle swarm optimization - Wikipedia <https://en.wikipedia.org/wiki/Particle_swarm_optimization>`__
- Kennedy, J.; Eberhart, R. (1995). "Particle Swarm Optimization". Proceedings of IEEE International Conference on Neural Networks. IV. pp. 1942–1948.
- Shi, Y.; Eberhart, R. (1998). "A Modified Particle Swarm Optimizer". IEEE International Conference on Evolutionary Computation.
- `A Gentle Introduction to Particle Swarm Optimization <https://machinelearningmastery.com/a-gentle-introduction-to-particle-swarm-optimization/>`__
- Clerc, M.; Kennedy, J. (2002). "The particle swarm - explosion, stability, and convergence in a multidimensional complex space". IEEE Transactions on Evolutionary Computation. 6 (1): 58–73.