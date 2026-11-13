.. _clothoid-path-planning:

clothoid 路径规划
--------------------------

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ClothoidPath/animation1.gif
.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ClothoidPath/animation2.gif
.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ClothoidPath/animation3.gif

这是 clothoid（回旋曲线）路径规划的示例代码。

它可以在两个二维位姿 (x, y, yaw) 之间用一条 clothoid 路径插值，其曲率线性连续。
换言之，这是用单段 clothoid 做的 G1 Hermite 插值。

该路径规划算法如下：

步骤 1：求解 g 函数
~~~~~~~~~~~~~~~~~~~~~~~

用非线性优化求解器求解 g(A) 函数。

.. math::

    g(A):=Y(2A, \delta-A, \phi_{s})

其中

* :math:`\delta`：起点位姿与目标点位姿的朝向差。
* :math:`\phi_{s}`：起点位姿的朝向。
* :math:`Y`：:math:`Y(a, b, c)=\int_{0}^{1} \sin \left(\frac{a}{2} \tau^{2}+b \tau+c\right) d \tau`


步骤 2：计算路径参数
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

可以用 :math:`A` 计算下列路径参数。

:math:`L`：路径长度

.. math::

        L=\frac{R}{X\left(2 A, \delta-A, \phi_{s}\right)}

其中

* :math:`R`：起点位姿与目标点位姿之间的距离
* :math:`X`：:math:`X(a, b, c)=\int_{0}^{1} \cos \left(\frac{a}{2} \tau^{2}+b \tau+c\right) d \tau`


- :math:`\kappa`：曲率

.. math::

        \kappa=(\delta-A) / L


- :math:`\kappa'`：曲率变化率

.. math::

        \kappa^{\prime}=2 A / L^{2}


步骤 3：用 Fresnel 积分构造路径
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

最终的 clothoid 路径可由路径参数和 Fresnel 积分算出。

.. math::
        \begin{aligned}
        &x(s)=x_{0}+\int_{0}^{s} \cos \left(\frac{1}{2} \kappa^{\prime} \tau^{2}+\kappa \tau+\vartheta_{0}\right) \mathrm{d} \tau \\
        &y(s)=y_{0}+\int_{0}^{s} \sin \left(\frac{1}{2} \kappa^{\prime} \tau^{2}+\kappa \tau+\vartheta_{0}\right) \mathrm{d} \tau
        \end{aligned}

代码链接
~~~~~~~~~~~~~

.. autofunction:: PathPlanning.ClothoidPath.clothoid_path_planner.generate_clothoid_path


参考文献
~~~~~~~~~~

-  `Fast and accurate G1 fitting of clothoid curves <https://www.researchgate.net/publication/237062806>`__
