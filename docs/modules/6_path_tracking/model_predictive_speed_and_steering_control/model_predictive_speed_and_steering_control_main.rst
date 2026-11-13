
模型预测速度与转向控制
-------------------------------------------

.. figure:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/model_predictive_speed_and_steer_control/animation.gif?raw=true
   :alt: 模型预测速度与转向控制

这是使用模型预测控制（MPC）的路径跟踪仿真。

MPC 控制器基于线性化模型控制车辆速度与转向。

本代码使用 cvxpy 作为优化建模工具。

-  `Welcome to CVXPY 1.0 — CVXPY 1.0.6
   documentation <http://www.cvxpy.org/>`__

代码链接
~~~~~~~~~~~~~~~

.. autofunction:: PathTracking.model_predictive_speed_and_steer_control.model_predictive_speed_and_steer_control.iterative_linear_mpc_control


模型预测控制（MPC）建模
~~~~~~~~~~~~~~~~~~~~~~~~~~

状态向量为：

.. math::  z = [x, y, v,\phi]

x：x 位置，y：y 位置，v：速度，φ：航向角

输入向量为：

.. math::  u = [a, \delta]

a：加速度，δ：转向角

MPC 控制器为路径跟踪最小化如下代价函数：

.. math:: min\ Q_f(z_{T,ref}-z_{T})^2+Q\Sigma({z_{t,ref}-z_{t}})^2+R\Sigma{u_t}^2+R_d\Sigma({u_{t+1}-u_{t}})^2

z_ref 来自目标路径与速度。

约束为：

-  线性化车辆模型

.. math:: z_{t+1}=Az_t+Bu+C

-  最大转向速率

.. math:: |u_{t+1}-u_{t}|<du_{max}

-  最大转向角

.. math:: |u_{t}|<u_{max}

-  初始状态

.. math:: z_0 = z_{0,ob}

-  最大与最小速度

.. math:: v_{min} < v_t < v_{max}

-  最大与最小输入

.. math:: u_{min} < u_t < u_{max}

实现位于

`PythonRobotics/model_predictive_speed_and_steer_control.py at
f51a73f47cb922a12659f8ce2d544c347a2a8156 ·
AtsushiSakai/PythonRobotics <https://github.com/AtsushiSakai/PythonRobotics/blob/f51a73f47cb922a12659f8ce2d544c347a2a8156/PathTracking/model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py#L247-L301>`__

车辆模型线性化
~~~~~~~~~~~~~~~~~~~~~~~~~~~

车辆模型为

.. math::  \dot{x} = vcos(\phi)

.. math::  \dot{y} = vsin((\phi)

.. math::  \dot{v} = a

.. math::  \dot{\phi} = \frac{vtan(\delta)}{L}

常微分方程为

.. math::  \dot{z} =\frac{\partial }{\partial z} z = f(z, u) = A'z+B'u

其中

:math:`\begin{equation*} A' = \begin{bmatrix} \frac{\partial }{\partial x}vcos(\phi) & \frac{\partial }{\partial y}vcos(\phi) & \frac{\partial }{\partial v}vcos(\phi) & \frac{\partial }{\partial \phi}vcos(\phi)\\ \frac{\partial }{\partial x}vsin(\phi) & \frac{\partial }{\partial y}vsin(\phi) & \frac{\partial }{\partial v}vsin(\phi) & \frac{\partial }{\partial \phi}vsin(\phi)\\ \frac{\partial }{\partial x}a& \frac{\partial }{\partial y}a& \frac{\partial }{\partial v}a& \frac{\partial }{\partial \phi}a\\ \frac{\partial }{\partial x}\frac{vtan(\delta)}{L}& \frac{\partial }{\partial y}\frac{vtan(\delta)}{L}& \frac{\partial }{\partial v}\frac{vtan(\delta)}{L}& \frac{\partial }{\partial \phi}\frac{vtan(\delta)}{L}\\ \end{bmatrix} \\ 　= \begin{bmatrix} 0 & 0 & cos(\bar{\phi}) & -\bar{v}sin(\bar{\phi})\\ 0 & 0 & sin(\bar{\phi}) & \bar{v}cos(\bar{\phi}) \\ 0 & 0 & 0 & 0 \\ 0 & 0 &\frac{tan(\bar{\delta})}{L} & 0 \\ \end{bmatrix} \end{equation*}`

:math:`\begin{equation*} B' = \begin{bmatrix} \frac{\partial }{\partial a}vcos(\phi) & \frac{\partial }{\partial \delta}vcos(\phi)\\ \frac{\partial }{\partial a}vsin(\phi) & \frac{\partial }{\partial \delta}vsin(\phi)\\ \frac{\partial }{\partial a}a & \frac{\partial }{\partial \delta}a\\ \frac{\partial }{\partial a}\frac{vtan(\delta)}{L} & \frac{\partial }{\partial \delta}\frac{vtan(\delta)}{L}\\ \end{bmatrix} \\ 　= \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ 1 & 0 \\ 0 & \frac{\bar{v}}{Lcos^2(\bar{\delta})} \\ \end{bmatrix} \end{equation*}`

以采样时间 dt 做前向欧拉离散化，可得到离散时间模型。

.. math:: z_{k+1}=z_k+f(z_k,u_k)dt

在 zbar 与 ubar 附近作一阶泰勒展开

.. math:: z_{k+1}=z_k+(f(\bar{z},\bar{u})+A'z_k+B'u_k-A'\bar{z}-B'\bar{u})dt

.. math:: z_{k+1}=(I + dtA')z_k+(dtB')u_k + (f(\bar{z},\bar{u})-A'\bar{z}-B'\bar{u})dt

因此，

.. math:: z_{k+1}=Az_k+Bu_k +C

其中，

:math:`\begin{equation*} A = (I + dtA')\\ = \begin{bmatrix} 1 & 0 & cos(\bar{\phi})dt & -\bar{v}sin(\bar{\phi})dt\\ 0 & 1 & sin(\bar{\phi})dt & \bar{v}cos(\bar{\phi})dt \\ 0 & 0 & 1 & 0 \\ 0 & 0 &\frac{tan(\bar{\delta})}{L}dt & 1 \\ \end{bmatrix} \end{equation*}`

:math:`\begin{equation*} B = dtB'\\ = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ dt & 0 \\ 0 & \frac{\bar{v}}{Lcos^2(\bar{\delta})}dt \\ \end{bmatrix} \end{equation*}`

:math:`\begin{equation*} C = (f(\bar{z},\bar{u})-A'\bar{z}-B'\bar{u})dt\\ = dt( \begin{bmatrix} \bar{v}cos(\bar{\phi})\\ \bar{v}sin(\bar{\phi}) \\ \bar{a}\\ \frac{\bar{v}tan(\bar{\delta})}{L}\\ \end{bmatrix} - \begin{bmatrix} \bar{v}cos(\bar{\phi})-\bar{v}sin(\bar{\phi})\bar{\phi}\\ \bar{v}sin(\bar{\phi})+\bar{v}cos(\bar{\phi})\bar{\phi}\\ 0\\ \frac{\bar{v}tan(\bar{\delta})}{L}\\ \end{bmatrix} - \begin{bmatrix} 0\\ 0 \\ \bar{a}\\ \frac{\bar{v}\bar{\delta}}{Lcos^2(\bar{\delta})}\\ \end{bmatrix} )\\ = \begin{bmatrix} \bar{v}sin(\bar{\phi})\bar{\phi}dt\\ -\bar{v}cos(\bar{\phi})\bar{\phi}dt\\ 0\\ -\frac{\bar{v}\bar{\delta}}{Lcos^2(\bar{\delta})}dt\\ \end{bmatrix} \end{equation*}`

该方程实现于

`PythonRobotics/model_predictive_speed_and_steer_control.py at
eb6d1cbe6fc90c7be9210bf153b3a04f177cc138 ·
AtsushiSakai/PythonRobotics <https://github.com/AtsushiSakai/PythonRobotics/blob/eb6d1cbe6fc90c7be9210bf153b3a04f177cc138/PathTracking/model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py#L80-L102>`__

参考文献
~~~~~~~~~

-  `Vehicle Dynamics and Control \| Rajesh Rajamani \|
   Springer <http://www.springer.com/us/book/9781461414322>`__

-  `MPC Book - MPC Lab @
   UC-Berkeley <https://sites.google.com/berkeley.edu/mpc-lab/mpc-course-material>`__
