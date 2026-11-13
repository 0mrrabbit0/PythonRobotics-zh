集合卡尔曼滤波（EnKF）定位
--------------------------

.. figure:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/ensamble_kalman_filter/animation.gif

这是使用集合卡尔曼滤波（EnKF）的传感器融合定位。

蓝线是真实轨迹，黑线是航位推算轨迹，
红线是集合卡尔曼滤波（EnKF）估计轨迹。

红色椭圆是集合卡尔曼滤波（EnKF）估计的协方差椭圆。

假设机器人能够测量到路标（RFID）的距离和方位角。

这些测量用于集合卡尔曼滤波（EnKF）定位。

代码链接
^^^^^^^^

.. autofunction:: Localization.ensemble_kalman_filter.ensemble_kalman_filter.enkf_localization


集合卡尔曼滤波算法
~~~~~~~~~~~~~~~~~~

集合卡尔曼滤波（EnKF）是卡尔曼滤波的一种蒙特卡洛方法，它用一组状态估计（集合）表示概率分布。与以解析方式传播均值和协方差的传统卡尔曼滤波不同，集合卡尔曼滤波（EnKF）使用一组样本（集合成员）来估计状态及其不确定性。

集合卡尔曼滤波（EnKF）算法包含两个主要步骤：

=== 预测 ===

对每个集合成员 :math:`i = 1, ..., N`：

1. 向控制输入加入随机噪声：

   :math:`\mathbf{u}^i = \mathbf{u} + \epsilon_u`，其中 :math:`\epsilon_u \sim \mathcal{N}(0, \mathbf{R})`

2. 预测状态：

   :math:`\mathbf{x}^i_{pred} = f(\mathbf{x}^i_t, \mathbf{u}^i)`

3. 预测观测：

   :math:`\mathbf{z}^i_{pred} = h(\mathbf{x}^i_{pred}) + \epsilon_z`，其中 :math:`\epsilon_z \sim \mathcal{N}(0, \mathbf{Q})`

=== 更新 ===

1. 计算状态的集合均值：

   :math:`\bar{\mathbf{x}} = \frac{1}{N}\sum_{i=1}^N \mathbf{x}^i_{pred}`

2. 计算观测的集合均值：

   :math:`\bar{\mathbf{z}} = \frac{1}{N}\sum_{i=1}^N \mathbf{z}^i_{pred}`

3. 计算状态偏离：

   :math:`\mathbf{X}' = \mathbf{x}^i_{pred} - \bar{\mathbf{x}}`

4. 计算观测偏离：

   :math:`\mathbf{Z}' = \mathbf{z}^i_{pred} - \bar{\mathbf{z}}`

5. 计算协方差矩阵：

   :math:`\mathbf{U} = \frac{1}{N-1}\mathbf{X}'\mathbf{Z}'^T`

   :math:`\mathbf{V} = \frac{1}{N-1}\mathbf{Z}'\mathbf{Z}'^T`

6. 计算卡尔曼增益：

   :math:`\mathbf{K} = \mathbf{U}\mathbf{V}^{-1}`

7. 更新每个集合成员：

   :math:`\mathbf{x}^i_{t+1} = \mathbf{x}^i_{pred} + \mathbf{K}(\mathbf{z}_{obs} - \mathbf{z}^i_{pred})`

8. 计算最终状态估计：

   :math:`\mathbf{x}_{est} = \frac{1}{N}\sum_{i=1}^N \mathbf{x}^i_{t+1}`

9. 计算协方差估计：

   :math:`\mathbf{P}_{est} = \frac{1}{N}\sum_{i=1}^N (\mathbf{x}^i_{t+1} - \mathbf{x}_{est})(\mathbf{x}^i_{t+1} - \mathbf{x}_{est})^T`


滤波器设计
~~~~~~~~~~

在本仿真中，机器人在时刻 :math:`t` 具有包含 4 个状态的状态向量：

.. math:: \mathbf{x}_t = [x_t, y_t, \phi_t, v_t]^T

其中：

- :math:`x, y` 是二维位置坐标
- :math:`\phi` 是朝向（航向角）
- :math:`v` 是速度

滤波器使用由 :math:`N=20` 个粒子组成的集合来表示状态分布。

输入向量
^^^^^^^^

机器人具有速度传感器和陀螺仪，因此每个时间步的控制输入向量为：

.. math:: \mathbf{u}_t = [v_t, \omega_t]^T

其中：

- :math:`v_t` 是线速度
- :math:`\omega_t` 是角速度（航向角速度）

输入向量包含由协方差矩阵 :math:`\mathbf{R}` 建模的传感器噪声：

.. math:: \mathbf{R} = \begin{bmatrix} \sigma_v^2 & 0 \\ 0 & \sigma_\omega^2 \end{bmatrix}


观测向量
^^^^^^^^

机器人可以通过距离和方位角测量来观测路标（RFID 标签）：

.. math:: \mathbf{z}_t = [d_t, \theta_t, x_{lm}, y_{lm}]

其中：

- :math:`d_t` 是到路标的距离
- :math:`\theta_t` 是到路标的方位角
- :math:`x_{lm}, y_{lm}` 是已知的路标位置

观测噪声由协方差矩阵 :math:`\mathbf{Q}` 建模：

.. math:: \mathbf{Q} = \begin{bmatrix} \sigma_d^2 & 0 \\ 0 & \sigma_\theta^2 \end{bmatrix}


运动模型
~~~~~~~~

机器人运动模型为：

.. math::  \dot{x} = v \cos(\phi)

.. math::  \dot{y} = v \sin(\phi)

.. math::  \dot{\phi} = \omega

.. math::  \dot{v} = 0

以时间步 :math:`\Delta t` 离散化后，运动模型变为：

.. math:: \mathbf{x}_{t+1} = f(\mathbf{x}_t, \mathbf{u}_t) = \mathbf{F}\mathbf{x}_t + \mathbf{B}\mathbf{u}_t

其中：

:math:`\begin{equation*} \mathbf{F} = \begin{bmatrix} 1 & 0 & 0 & 0\\ 0 & 1 & 0 & 0\\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 \\ \end{bmatrix} \end{equation*}`

:math:`\begin{equation*} \mathbf{B} = \begin{bmatrix} \cos(\phi) \Delta t & 0\\ \sin(\phi) \Delta t & 0\\ 0 & \Delta t\\ 1 & 0\\ \end{bmatrix} \end{equation*}`

运动函数展开为：

:math:`\begin{equation*} \begin{bmatrix} x_{t+1} \\ y_{t+1} \\ \phi_{t+1} \\ v_{t+1} \end{bmatrix} = \begin{bmatrix} x_t + v_t\cos(\phi_t)\Delta t \\ y_t + v_t\sin(\phi_t)\Delta t \\ \phi_t + \omega_t \Delta t \\ v_t \end{bmatrix} \end{equation*}`


观测模型
~~~~~~~~

对位于 :math:`(x_{lm}, y_{lm})` 的每个路标，观测模型计算其在全局坐标系中的期望路标位置：

.. math:: 
   \begin{bmatrix} 
   x_{lm,obs} \\ 
   y_{lm,obs} 
   \end{bmatrix} = 
   \begin{bmatrix} 
   x + d \cos(\phi + \theta) \\ 
   y + d \sin(\phi + \theta) 
   \end{bmatrix}

其中 :math:`d` 与 :math:`\theta` 是观测到的到该路标的距离和方位角。

观测函数把机器人状态投影为期望的路标位置，然后在更新步骤中与实际路标位置进行比较。


集合卡尔曼滤波（EnKF）的优点
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- **无需雅可比矩阵**：与扩展卡尔曼滤波（EKF）不同，集合卡尔曼滤波（EnKF）无需计算雅可比矩阵，因此更容易用于非线性系统
- **可处理非高斯分布**：集合表示能够捕捉状态分布的非高斯特征
- **计算高效**：对于高维系统，集合卡尔曼滤波（EnKF）可以比维护完整协方差矩阵更高效
- **易于并行**：每个集合成员可以独立传播


参考文献
~~~~~~~~

- `Ensemble Kalman filtering <https://rmets.onlinelibrary.wiley.com/doi/10.1256/qj.05.135>`_
- `PROBABILISTIC ROBOTICS <http://www.probabilistic-robotics.org/>`_
