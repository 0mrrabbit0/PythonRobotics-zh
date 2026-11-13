使用矩形拟合的物体形状识别
--------------------------

这是使用矩形拟合进行的物体形状识别。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/rectangle_fitting/animation.gif

本示例代码基于这篇论文中的算法：

- `Efficient L\-Shape Fitting for Vehicle Detection Using Laser Scanners \- The Robotics Institute Carnegie Mellon University <https://www.ri.cmu.edu/publications/efficient-l-shape-fitting-for-vehicle-detection-using-laser-scanners/>`_

该算法包含如下 2 个步骤。

步骤 1：自适应距离分割
~~~~~~~~~~~~~~~~~~~~~~

第一步，把全部距离数据点分割为若干聚类。

计算每个距离数据与最近的距离数据之间的距离；若该距离低于某一阈值，则判定它们属于同一聚类。该距离阈值按到传感器的距离成比例确定。这利用了距离传感器的一般模型：随着到传感器的距离增加，数据分布往往更稀疏。

阈值距离按如下公式计算：

.. math:: r_{th} = R_0 + R_d * r_{origin}

其中

- :math:`r_{th}`：阈值距离
- :math:`R_0, R_d`：常数参数
- :math:`r_{origin}`：某个距离数据到传感器的距离。

步骤 2：矩形搜索
~~~~~~~~~~~~~~~~

第二步，对上一步算出的每个聚类进行矩形拟合。在该矩形拟合中，每个聚类的距离数据按一定的角度间隔旋转，并用下面三个评价函数之一加以评估，然后选取角度参数最优的一个作为矩形形状。

1. 矩形面积最小化准则
=====================

该评价函数计算包含全部点的最小矩形的面积，这个面积由所有距离数据点在 x-y 轴上的最大值与最小值之差得到。这样就可以沿尽可能包住该最小矩形形状的方向拟合矩形。

2. 接近度准则
=============

该评价函数把矩形右侧上下顶点与距离数据中每个点之间的距离用作评价值。若矩形边上存在点，该评价值会减小。

3. 方差准则
===========

该评价函数使用矩形的边（水平边与垂直边）与每个点之间的平方距离。计算平方误差等同于计算方差。该方差越小，越说明这些点与矩形贴合。

代码链接
~~~~~~~~

.. autoclass:: Mapping.rectangle_fitting.rectangle_fitting.LShapeFitting
	:members:

参考文献
~~~~~~~~

- `Efficient L\-Shape Fitting for Vehicle Detection Using Laser Scanners \- The Robotics Institute Carnegie Mellon University <https://www.ri.cmu.edu/publications/efficient-l-shape-fitting-for-vehicle-detection-using-laser-scanners/>`_
