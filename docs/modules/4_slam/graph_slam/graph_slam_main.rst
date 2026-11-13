基于图的 SLAM
----------------

这是一个基于图的同步定位与建图（SLAM）示例。

蓝线是真值。

黑线是航位推算。

红线是基于图的 SLAM 估计的轨迹。

黑色星号是用来生成图边的路标。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/SLAM/GraphBasedSLAM/animation.gif

代码链接
~~~~~~~~~~~~

.. autofunction:: SLAM.GraphBasedSLAM.graph_based_slam.graph_based_slam


.. include:: graphSLAM_doc.rst
.. include:: graphSLAM_formulation.rst
.. include:: graphSLAM_SE2_example.rst

参考文献
~~~~~~~~~~~

- `A Tutorial on Graph-Based SLAM <http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf>`_
