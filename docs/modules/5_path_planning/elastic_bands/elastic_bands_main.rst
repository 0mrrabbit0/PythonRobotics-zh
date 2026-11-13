弹性带
-------------

这是使用弹性带的路径规划。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ElasticBands/animation.gif

代码链接
+++++++++++++

.. autoclass:: PathPlanning.ElasticBands.elastic_bands.ElasticBands


核心概念
~~~~~~~~~~~~
- **弹性带**：由全局规划器初始化、可动态变形的无碰撞路径。
- **目标**：

  * 缩短并平滑路径。
  * 尽量增大与障碍物的间隙。
  * 保持全局路径的连通性。

气泡表示
~~~~~~~~~~~~~~~~~~~~~~~~
- **定义**：构型 :math:`b` 周围的局部自由空间区域：

  .. math::
     B(b) = \{ q: \|q - b\| < \rho(b) \},
  
  其中 :math:`\rho(b)` 是气泡半径。


基于力的变形
~~~~~~~~~~~~~~~~~~~~~~~
弹性带在人工力作用下变形：

内部收缩力
++++++++++++++++++++++++++
- **作用**：减小路径的松弛和长度。
- **公式**：对于节点 :math:`b_i`：

  .. math::
     f_c(b_i) = k_c \left( \frac{b_{i-1} - b_i}{\|b_{i-1} - b_i\|} + \frac{b_{i+1} - b_i}{\|b_{i+1} - b_i\|} \right)

  其中 :math:`k_c` 是收缩增益。

外部排斥力
+++++++++++++++++++++++++
- **作用**：把路径推离障碍物。
- **公式**：对于节点 :math:`b_i`：

  .. math::
     f_r(b_i) = \begin{cases} 
     k_r (\rho_0 - \rho(b_i)) \nabla \rho(b_i) & \text{if } \rho(b_i) < \rho_0, \\
     0 & \text{otherwise}.
     \end{cases}

  其中 :math:`k_r` 是排斥增益，:math:`\rho_0` 是施加排斥力的最大距离，:math:`\nabla \rho(b_i)` 用有限差分近似：

  .. math::
     \frac{\partial \rho}{\partial x} \approx \frac{\rho(b_i + h) - \rho(b_i - h)}{2h}.

动态路径维护
~~~~~~~~~~~~~~~~~~~~~~~~~~
1. **节点更新**：
   
   .. math::
      b_i^{\text{new}} = b_i^{\text{old}} + \alpha (f_c + f_r),

   其中 :math:`\alpha` 是步长参数，通常与 :math:`\rho(b_i^{\text{old}})` 成正比。

2. **重叠约束**：
- 若相邻节点相距过远，则插入新节点
- 若相邻节点相距过近，则删除冗余节点

参考文献
+++++++++++++

-  `Elastic Bands: Connecting Path Planning and Control <http://www8.cs.umu.se/research/ifor/dl/Control/elastic%20bands.pdf>`__
