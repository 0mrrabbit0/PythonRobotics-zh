无迹卡尔曼滤波（UKF）定位
-------------------------

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/unscented_kalman_filter/animation.gif

这是使用无迹卡尔曼滤波（UKF）的传感器融合定位。

蓝线是真实轨迹，黑线是航位推算轨迹，
绿点是 GPS 观测，红线是无迹卡尔曼滤波（UKF）估计轨迹。

红色椭圆是无迹卡尔曼滤波（UKF）估计的协方差椭圆。

代码链接
~~~~~~~~

.. autofunction:: Localization.unscented_kalman_filter.unscented_kalman_filter.ukf_estimation


无迹卡尔曼滤波（UKF）算法
~~~~~~~~~~~~~~~~~~~~~~~~~

无迹卡尔曼滤波（UKF）是一种非线性状态估计方法，它利用无迹变换处理非线性。与使用雅可比矩阵对非线性函数进行线性化的扩展卡尔曼滤波（EKF）不同，无迹卡尔曼滤波（UKF）使用称为西格玛点的确定性采样方法，来捕捉状态分布的均值和协方差。

无迹卡尔曼滤波（UKF）算法包含两个主要步骤：

=== 预测 ===

1. 在当前状态估计周围生成西格玛点：

   :math:`\chi_0 = \mathbf{x}_{t}`

   :math:`\chi_i = \mathbf{x}_{t} + \gamma\sqrt{\mathbf{P}_t}_i`，:math:`i = 1, ..., n`

   :math:`\chi_i = \mathbf{x}_{t} - \gamma\sqrt{\mathbf{P}_t}_{i-n}`，:math:`i = n+1, ..., 2n`

   其中 :math:`\gamma = \sqrt{n + \lambda}`，且 :math:`\lambda = \alpha^2(n + \kappa) - n`

2. 将西格玛点通过运动模型传播：

   :math:`\chi^-_i = f(\chi_i, \mathbf{u}_t)`

3. 计算预测状态均值：

   :math:`\mathbf{x}^-_{t+1} = \sum_{i=0}^{2n} w^m_i \chi^-_i`

4. 计算预测状态协方差：

   :math:`\mathbf{P}^-_{t+1} = \sum_{i=0}^{2n} w^c_i (\chi^-_i - \mathbf{x}^-_{t+1})(\chi^-_i - \mathbf{x}^-_{t+1})^T + \mathbf{Q}`

=== 更新 ===

1. 在预测状态周围生成西格玛点：

   :math:`\chi_i = \text{generate\_sigma\_points}(\mathbf{x}^-_{t+1}, \mathbf{P}^-_{t+1})`

2. 将西格玛点通过观测模型传播：

   :math:`\mathcal{Z}_i = h(\chi_i)`

3. 计算预测观测均值：

   :math:`\mathbf{z}^-_{t+1} = \sum_{i=0}^{2n} w^m_i \mathcal{Z}_i`

4. 计算新息协方差：

   :math:`\mathbf{S}_t = \sum_{i=0}^{2n} w^c_i (\mathcal{Z}_i - \mathbf{z}^-_{t+1})(\mathcal{Z}_i - \mathbf{z}^-_{t+1})^T + \mathbf{R}`

5. 计算互相关矩阵：

   :math:`\mathbf{P}_{xz} = \sum_{i=0}^{2n} w^c_i (\chi_i - \mathbf{x}^-_{t+1})(\mathcal{Z}_i - \mathbf{z}^-_{t+1})^T`

6. 计算卡尔曼增益：

   :math:`\mathbf{K} = \mathbf{P}_{xz}\mathbf{S}_t^{-1}`

7. 更新状态估计：

   :math:`\mathbf{x}_{t+1} = \mathbf{x}^-_{t+1} + \mathbf{K}(\mathbf{z}_t - \mathbf{z}^-_{t+1})`

8. 更新协方差估计：

   :math:`\mathbf{P}_{t+1} = \mathbf{P}^-_{t+1} - \mathbf{K}\mathbf{S}_t\mathbf{K}^T`


西格玛点与权重
~~~~~~~~~~~~~~

无迹卡尔曼滤波（UKF）使用确定性西格玛点表示状态分布。用于组合西格玛点的权重按如下方式计算：

**均值权重：**

:math:`w^m_0 = \frac{\lambda}{n + \lambda}`

:math:`w^m_i = \frac{1}{2(n + \lambda)}`，:math:`i = 1, ..., 2n`

**协方差权重：**

:math:`w^c_0 = \frac{\lambda}{n + \lambda} + (1 - \alpha^2 + \beta)`

:math:`w^c_i = \frac{1}{2(n + \lambda)}`，:math:`i = 1, ..., 2n`

其中：

- :math:`\alpha` 控制西格玛点在均值周围的散布（通常 :math:`0.001 \leq \alpha \leq 1`）
- :math:`\beta` 纳入分布的先验知识（对高斯分布，:math:`\beta = 2` 为最优）
- :math:`\kappa` 是次级缩放参数（通常 :math:`\kappa = 0`）
- :math:`n` 是状态向量的维数


滤波器设计
~~~~~~~~~~

在本仿真中，机器人在时刻 :math:`t` 具有包含 4 个状态的状态向量：

.. math:: \mathbf{x}_t = [x_t, y_t, \phi_t, v_t]^T

其中：

- :math:`x, y` 是二维位置坐标
- :math:`\phi` 是朝向（航向角）
- :math:`v` 是速度

在代码中，"xEst" 表示状态向量。

协方差矩阵为：

- :math:`\mathbf{P}_t` 是状态协方差矩阵
- :math:`\mathbf{Q}` 是过程噪声协方差矩阵
- :math:`\mathbf{R}` 是观测噪声协方差矩阵

**过程噪声协方差** :math:`\mathbf{Q}`：

.. math:: 
   \mathbf{Q} = \begin{bmatrix} 
   0.1^2 & 0 & 0 & 0 \\
   0 & 0.1^2 & 0 & 0 \\
   0 & 0 & (0.017)^2 & 0 \\
   0 & 0 & 0 & 1.0^2
   \end{bmatrix}

**观测噪声协方差** :math:`\mathbf{R}`：

.. math:: 
   \mathbf{R} = \begin{bmatrix} 
   1.0^2 & 0 \\
   0 & 1.0^2
   \end{bmatrix}


输入向量
^^^^^^^^

机器人具有速度传感器和陀螺仪，因此每个时间步的控制输入向量为：

.. math:: \mathbf{u}_t = [v_t, \omega_t]^T

其中：

- :math:`v_t` 是线速度 [m/s]
- :math:`\omega_t` 是角速度（航向角速度）[rad/s]

输入向量包含传感器噪声。


观测向量
^^^^^^^^

机器人具有可测量 x-y 位置的 GNSS（GPS）传感器：

.. math:: \mathbf{z}_t = [x_t, y_t]^T

观测包含协方差为 :math:`\mathbf{R}` 的 GPS 噪声。


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

:math:`\Delta t = 0.1` [s] 是时间间隔。


观测模型
~~~~~~~~

机器人可以从 GPS 获取 x-y 位置信息。

GPS 观测模型为：

.. math:: \mathbf{z}_{t} = h(\mathbf{x}_t) = \mathbf{H} \mathbf{x}_t

其中：

:math:`\begin{equation*} \mathbf{H} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ \end{bmatrix} \end{equation*}`

观测函数表明：

:math:`\begin{equation*} \begin{bmatrix} x_{obs} \\ y_{obs} \end{bmatrix} = h(\mathbf{x}) = \begin{bmatrix} x \\ y \end{bmatrix} \end{equation*}`


无迹卡尔曼滤波（UKF）参数
~~~~~~~~~~~~~~~~~~~~~~~~~

无迹卡尔曼滤波（UKF）使用三个调节参数：

- **ALPHA = 0.001**：控制西格玛点在均值周围的散布。取值越小，西格玛点越靠近均值。
- **BETA = 2**：纳入关于分布的先验知识。对于高斯分布，最优值为 2。
- **KAPPA = 0**：次级缩放参数。通常取 0，或取 3-n，其中 n 为状态维数。

这些参数影响 :math:`\lambda = \alpha^2(n + \kappa) - n` 的计算，而该值决定西格玛点的散布和权重。


无迹卡尔曼滤波（UKF）相对于扩展卡尔曼滤波（EKF）的优点
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

无迹卡尔曼滤波（UKF）相对于扩展卡尔曼滤波（EKF）有以下优点：

- **无需雅可比矩阵**：无迹卡尔曼滤波（UKF）无需计算雅可比矩阵；对复杂非线性系统而言，雅可比矩阵既容易出错，计算代价也高
- **精度更高**：无迹卡尔曼滤波（UKF）对均值和协方差的捕捉达到二阶（对高斯分布为三阶），而扩展卡尔曼滤波（EKF）只达到一阶精度
- **更好地处理非线性**：无迹变换能更好地近似非线性变换之后的概率分布
- **更易实现**：对于强非线性系统，无迹卡尔曼滤波（UKF）通常更容易实现，因为它不需要解析导数
- **数值稳定性**：对于强非线性系统，无迹卡尔曼滤波（UKF）在数值上可以比扩展卡尔曼滤波（EKF）更稳定


参考文献
~~~~~~~~

- `Discriminatively Trained Unscented Kalman Filter for Mobile Robot Localization <https://www.researchgate.net/publication/267963417_Discriminatively_Trained_Unscented_Kalman_Filter_for_Mobile_Robot_Localization>`_
- `The Unscented Kalman Filter for Nonlinear Estimation <https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf>`_
- `PROBABILISTIC ROBOTICS <http://www.probabilistic-robotics.org/>`_
