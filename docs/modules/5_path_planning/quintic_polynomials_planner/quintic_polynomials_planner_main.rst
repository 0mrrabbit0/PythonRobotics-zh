
五次多项式规划
----------------------------

用五次多项式做运动规划。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/QuinticPolynomialsPlanner/animation.gif

它可以根据五次多项式计算二维路径以及速度、加速度曲线。

代码链接
~~~~~~~~~~~~~~~

.. autofunction:: PathPlanning.QuinticPolynomialsPlanner.quintic_polynomials_planner.quintic_polynomials_planner



一维机器人运动的五次多项式
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

假设一维机器人运动 :math:`x(t)` 在时刻 :math:`t` 被写成关于时间的五次多项式，如下：

.. math:: x(t) = a_0+a_1t+a_2t^2+a_3t^3+a_4t^4+a_5t^5
    :label: quintic_eq1

:math:`a_0, a_1. a_2, a_3, a_4, a_5` 是五次多项式的参数。

假定端点状态（起点和终点）作为边界条件是已知的。

起点的位置、速度和加速度分别为 :math:`x_s, v_s, a_s`。

终点的位置、速度和加速度分别为 :math:`x_e, v_e, a_e`。

因此，当时间为 0 时，

.. math:: x(0) = a_0 = x_s
    :label: quintic_eq2

然后把方程 :eq:`quintic_eq1` 对 t 求导，

.. math:: x'(t) = a_1+2a_2t+3a_3t^2+4a_4t^3+5a_5t^4
    :label: quintic_eq3

因此，当时间为 0 时，

.. math:: x'(0) = a_1 = v_s
    :label: quintic_eq4

再把方程 :eq:`quintic_eq3` 对 t 求导，

.. math:: x''(t) = 2a_2+6a_3t+12a_4t^2
    :label: quintic_eq5

因此，当时间为 0 时，

.. math:: x''(0) = 2a_2 = a_s
    :label: quintic_eq6

于是可以由式 :eq:`quintic_eq2`、:eq:`quintic_eq4`、:eq:`quintic_eq6` 以及边界条件算出 :math:`a_0, a_1, a_2`。

式 :eq:`quintic_eq1` 中的 :math:`a_3, a_4, a_5` 仍然未知。

设一次机动的结束时间为 :math:`T`，可以由式 :eq:`quintic_eq1`、:eq:`quintic_eq3`、:eq:`quintic_eq5` 得到下列方程：

.. math:: x(T)=a_0+a_1T+a_2T^2+a_3T^3+a_4T^4+a_5T^5=x_e
    :label: quintic_eq7

.. math:: x'(T)=a_1+2a_2T+3a_3T^2+4a_4T^3+5a_5T^4=v_e
    :label: quintic_eq8

.. math:: x''(T)=2a_2+6a_3T+12a_4T^2+20a_5T^3=a_e
    :label: quintic_eq9

由式 :eq:`quintic_eq7`、:eq:`quintic_eq8`、:eq:`quintic_eq9`，可以通过求解线性方程组 :math:`Ax=b` 算出 :math:`a_3, a_4, a_5`：

.. math:: \begin{bmatrix} T^3 & T^4 & T^5 \\ 3T^2 & 4T^3 & 5T^4 \\ 6T & 12T^2 & 20T^3 \end{bmatrix}\begin{bmatrix} a_3\\ a_4\\ a_5\end{bmatrix}=\begin{bmatrix} x_e-x_s-v_sT-0.5a_sT^2\\ v_e-v_s-a_sT\\ a_e-a_s\end{bmatrix}

至此可以得到全部未知参数。

二维机器人运动（x-y）的五次多项式
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

若沿 x 轴和 y 轴各使用一个五次多项式，就可以在 x-y 平面内规划二维机器人运动。

.. math:: x(t) = a_0+a_1t+a_2t^2+a_3t^3+a_4t^4+a_5t^5
    :label: quintic_eq10

.. math:: y(t) = b_0+b_1t+b_2t^2+b_3t^3+b_4t^4+b_5t^5
    :label: quintic_eq11

假定端点状态（起点和终点）作为边界条件是已知的。

起点的位置、朝向、速度和加速度分别为 :math:`x_s, y_s, \theta_s, v_s, a_s`。

终点的位置、朝向、速度和加速度分别为 :math:`x_e, y_e. \theta_e, v_e, a_e`。

各个速度和加速度边界条件可以由各自的朝向算出。

:math:`v_{xs}=v_scos(\theta_s), v_{ys}=v_ssin(\theta_s)`

:math:`v_{xe}=v_ecos(\theta_e), v_{ye}=v_esin(\theta_e)`

参考文献
~~~~~~~~~~~

-  `Local Path Planning And Motion Control For Agv In
   Positioning <https://ieeexplore.ieee.org/document/637936/>`__


