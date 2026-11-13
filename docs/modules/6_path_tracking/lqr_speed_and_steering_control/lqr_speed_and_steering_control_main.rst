.. _linearquadratic-regulator-(lqr)-speed-and-steering-control:

线性二次调节器（LQR）速度与转向控制
-----------------------------------------------------------

使用线性二次调节器（LQR）速度与转向控制的路径跟踪仿真。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/lqr_speed_steer_control/animation.gif


代码链接
~~~~~~~~~~~~~~~

.. autofunction:: PathTracking.lqr_speed_steer_control.lqr_speed_steer_control.lqr_speed_steering_control


概述
~~~~~~~~

`lqr_speed_steer_control.py` 中实现的线性二次调节器（LQR）速度与转向控制模型提供一项仿真，使自动驾驶车辆跟踪：

1. 期望速度：根据当前状态与期望速度的反馈调节加速度。

2. 期望轨迹：根据当前状态与期望轨迹的反馈调节转向角。

上述跟踪仅使用一个 LQT 控制器。

车辆运动模型
~~~~~~~~~~~~~~~~~~~~~

下图为本仿真所用车辆的几何模型：

.. image:: lqr_steering_control_model.jpg
   :width: 600px

`e`、:math:`{\theta}` 与 :math:`\upsilon` 分别表示相对于期望轨迹和期望速度的横向误差、航向角误差与速度误差。
:math:`\dot{e}` 与 :math:`\dot{\theta}` 表示这些误差的变化率。

:math:`e_t`、:math:`\theta_t` 与 :math:`\upsilon` 分别是 `e`、:math:`\theta` 与 :math:`\upsilon` 在时刻 `t` 的更新值，可由下列运动学方程计算：

.. math:: e_t = e_{t-1} + \dot{e}_{t-1} dt

.. math:: \theta_t = \theta_{t-1} + \dot{\theta}_{t-1} dt

.. math:: \upsilon_t = \upsilon_{t-1} + a_{t-1} dt

其中 `dt` 为时间差，:math:`a_t` 为时刻 `t` 的加速度。

`e` 的变化率可计算为：

.. math:: \dot{e}_t = V \sin(\theta_{t-1})

其中 `V` 为当前车辆速度。

若 :math:`\theta` 很小，

.. math:: \theta \approx 0

则 :math:`\sin(\theta)` 可近似为 :math:`\theta`：

.. math:: \sin(\theta) = \theta

因此，`e` 的变化率可近似为：

.. math:: \dot{e}_t = V \theta_{t-1}

:math:`\theta` 的变化率可计算为：

.. math:: \dot{\theta}_t = \frac{V}{L} \tan(\delta)

其中 `L` 为车辆轴距，:math:`\delta` 为转向角。

若 :math:`\delta` 很小，

.. math:: \delta \approx 0

则 :math:`\tan(\delta)` 可近似为 :math:`\delta`：

.. math:: \tan(\delta) = \delta

因此，:math:`\theta` 的变化率可近似为：

.. math:: \dot{\theta}_t = \frac{V}{L} \delta

上述方程可用于在每个时间步更新车辆状态。

控制模型
~~~~~~~~~~~~~~

为把车辆动力学的状态空间表示写成线性模型，状态向量 `x` 与控制输入向量 `u` 定义如下：

.. math:: x_t = [e_t, \dot{e}_t, \theta_t, \dot{\theta}_t, \upsilon_t]^T

.. math:: u_t = [\delta_t, a_t]^T

线性状态转移方程可表示为：

.. math:: x_{t+1} = A x_t + B u_t

其中：

:math:`\begin{equation*} A = \begin{bmatrix} 1 & dt & 0 & 0 & 0\\ 0 & 0 & v & 0 & 0\\ 0 & 0 & 1 & dt & 0\\ 0 & 0 & 0 & 0 & 0\\ 0 & 0 & 0 & 0 & 1\\\end{bmatrix} \end{equation*}`

:math:`\begin{equation*} B = \begin{bmatrix} 0 & 0\\ 0 & 0\\ 0 & 0\\ \frac{v}{L} & 0\\ 0 & dt \\ \end{bmatrix} \end{equation*}`

线性二次调节器（LQR）控制器
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

线性二次调节器（LQR）控制器用于计算使二次代价函数最小的最优控制输入 `u`：

:math:`J = \sum_{t=0}^{N} (x_t^T Q x_t + u_t^T R u_t)`

其中 `Q` 与 `R` 分别是状态与控制输入的加权矩阵。

对于线性模型：

:math:`x_{t+1} = A x_t + B u_t`

最优控制输入 `u` 可计算为：

:math:`u_t = -K x_t`

其中 `K` 是求解 Riccati 方程得到的反馈增益矩阵。

仿真结果
~~~~~~~~~~~~~~~~~~~


.. image:: x-y.png
   :width: 600px

.. image:: yaw.png
   :width: 600px

.. image:: speed.png
   :width: 600px



参考文献
~~~~~~~~~~~

-  `Towards fully autonomous driving: Systems and algorithms <https://ieeexplore.ieee.org/document/5940562/>`__
