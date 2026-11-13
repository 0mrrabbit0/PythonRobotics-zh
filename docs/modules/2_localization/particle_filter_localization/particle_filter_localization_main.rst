粒子滤波定位
------------

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/particle_filter/animation.gif

这是使用粒子滤波（PF）的传感器融合定位。

蓝线是真实轨迹，黑线是航位推算轨迹，

红线是粒子滤波（PF）估计轨迹。

假设机器人能够测量到路标（RFID）的距离。

这些测量用于粒子滤波（PF）定位。

代码链接
~~~~~~~~

.. autofunction:: Localization.particle_filter.particle_filter.pf_localization


如何由粒子计算协方差矩阵
~~~~~~~~~~~~~~~~~~~~~~~~

由粒子信息得到的协方差矩阵 :math:`\Xi` 按下列公式计算：

.. math:: \Xi_{j,k}=\frac{1}{1-\sum^N_{i=1}(w^i)^2}\sum^N_{i=1}w^i(x^i_j-\mu_j)(x^i_k-\mu_k)

- :math:`\Xi_{j,k}` 是第 :math:`i` 行、第 :math:`k` 列的协方差矩阵元素。

- :math:`w^i` 是第 :math:`i` 个粒子的权重。

- :math:`x^i_j` 是第 :math:`j` 个状态，属于第 :math:`i` 个粒子。

- :math:`\mu_j` 是粒子第 :math:`j` 个状态的均值。

参考文献
~~~~~~~~

- `_PROBABILISTIC ROBOTICS: <http://www.probabilistic-robotics.org>`_
- `Improving the particle filter in high dimensions using conjugate artificial process noise <https://arxiv.org/pdf/1801.07000>`_
