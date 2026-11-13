带偏差估计的 GPS/惯性测量单元（IMU）融合定位
============================================

本示例使用扩展卡尔曼滤波（EKF），在局部米制坐标系中，把机体坐标系下的加速度计和陀螺仪测量与 GPS 位置结合起来。惯性测量单元（IMU）预测以 20 Hz 运行，GPS 校正以 1 Hz 运行。仿真沿 8 字形轨迹比较带偏差估计的融合、不带偏差估计的融合，以及仅使用惯性测量单元（IMU）的航位推算，其中包括从 20 秒到 30 秒的 GPS 中断。

.. image:: https://raw.githubusercontent.com/AtsushiSakai/PythonRoboticsGifs/5abf69412ae056a2f30ba562a1c04600b6062fe4/Localization/gps_imu_fusion/animation.gif
   :alt: GPS 与惯性测量单元（IMU）融合定位，含位置协方差椭圆，以及相对真值带有三倍标准差不确定度区间的偏差估计

下方三个面板以蓝色显示扩展卡尔曼滤波（EKF）估计的加速度计 x/y 偏差（m/s²）和陀螺仪偏差（deg/s），黑色虚线为真值。真实偏差分别为 0.04 m/s²、-0.03 m/s² 和 0.4 deg/s。灰色阴影在误差图和偏差图中标出 GPS 中断。

基于协方差的不确定度
--------------------

路径图在每个扩展卡尔曼滤波（EKF）估计处绘制以该估计为中心的 3σ 位置椭圆，并标出当前估计和真值。若 :math:`\lambda_i` 是位置协方差 :math:`P_{xy}` 的特征值，则半轴长度为 :math:`3\sqrt{\lambda_i}`，特征向量给出它们的方向。
其中包含 x/y 互协方差。椭圆边界满足
:math:`(\mathbf{p}-\hat{\mathbf{p}})^T P_{xy}^{-1}
(\mathbf{p}-\hat{\mathbf{p}})=9`。

位置误差图中的蓝色虚线跟踪带偏差估计的扩展卡尔曼滤波（EKF）的椭圆半长轴 :math:`3\sqrt{\lambda_{\max}(P_{xy})}`。
它是椭圆的外接半径，不是欧氏位置误差的标准差。仅凭误差低于这条线，并不能保证真实位置落在椭圆内部，方向同样重要。

每个偏差面板对 :math:`\hat b_i \pm 3\sqrt{P_{ii}}` 加阴影，使用估计状态协方差中对应的对角元素。陀螺仪的估计值和标准差都从 rad/s 转换为 deg/s。估计值位于其区间的中心，真值提供独立对照。
这些图直接使用滤波器的协方差，因此显示不确定度随 GPS 校正而收缩，并在中断期间增大。

对于默认的种子 0 仿真，在每个采样上，真值都位于带偏差估计的扩展卡尔曼滤波（EKF）的位置椭圆之内，也位于全部三个偏差区间之内。不带偏差估计的扩展卡尔曼滤波（EKF）在运行的部分时段，位置误差落在其椭圆之外。这些是本次合成仿真的结果，并不保证对其他数据具有同样的覆盖率。3σ 椭圆约包围理想二维高斯分布的 98.9%，而标量 ±3σ 区间约包围 99.7%；参见 `Matplotlib 置信椭圆说明
<https://matplotlib.org/stable/gallery/statistics/confidence_ellipse.html>`_。

假设
------

运动限制在水平面内。惯性测量单元（IMU）与机体坐标系对齐，其加速度已经过重力补偿。GPS 位置以米表示，而不是纬度和经度。各传感器在时间上对齐，并且安装在同一位置。初始航向和速度近似已知。横滚、俯仰、高度、磁力计和气压计都没有建模。

本示例估计恒定或缓慢变化的加速度计偏差和陀螺仪偏差。GPS 只观测位置，因此航向和偏差的估计依赖于运动以及初始不确定度；它们无法全部从静止时的位置测量中恢复。其中不包含真实传感器数据、离群点剔除，也不包含针对具体硬件的标定。

状态与惯性预测
--------------

八维状态包含世界坐标系位置、世界坐标系速度、航向、两个机体坐标系加速度计偏差，以及一个航向角速度偏差：

.. math::

   \mathbf{x} = [p_x, p_y, v_x, v_y, \psi, b_{ax}, b_{ay}, b_\omega]^T.

设 :math:`\mathbf{a}_m` 和 :math:`\omega_m` 分别为测得的加速度和航向角速度。借助机体坐标系到世界坐标系的旋转

.. math::

   R(\psi)=\begin{bmatrix}\cos\psi&-\sin\psi\\\sin\psi&\cos\psi\end{bmatrix},
   \qquad \mathbf{a}_w=R(\psi)(\mathbf{a}_m-\mathbf{b}_a),

离散预测在一个采样周期内把世界坐标系加速度视为常量：

.. math::

   \mathbf{p}^{-} &= \mathbf{p}+\mathbf{v}\Delta t+\tfrac12\mathbf{a}_w\Delta t^2,\\
   \mathbf{v}^{-} &= \mathbf{v}+\mathbf{a}_w\Delta t,\\
   \psi^{-} &= \operatorname{wrap}(\psi+(\omega_m-b_\omega)\Delta t),\\
   \mathbf{b}^{-} &= \mathbf{b}.

航向被卷绕到 :math:`[-\pi,\pi)`。定义
:math:`\mathbf{j}=[-a_{wy},a_{wx}]^T`。按分块顺序
:math:`(\mathbf{p},\mathbf{v},\psi,\mathbf{b}_a,b_\omega)`，雅可比矩阵为

.. math::

   F=\begin{bmatrix}
   I&\Delta t I&\tfrac12\Delta t^2\mathbf{j}&-\tfrac12\Delta t^2R&0\\
   0&I&\Delta t\mathbf{j}&-\Delta t R&0\\
   0&0&1&0&-\Delta t\\
   0&0&0&I&0\\
   0&0&0&0&1
   \end{bmatrix},\qquad
   G=\begin{bmatrix}
   \tfrac12\Delta t^2R&0\\ \Delta t R&0\\ 0&\Delta t\\ 0&0\\ 0&0
   \end{bmatrix}.

协方差按如下方式传播

.. math::

   P^{-}=FPF^T+GQ_{\mathrm{imu}}G^T+Q_b.

:math:`Q_{\mathrm{imu}}` 是一次离散惯性测量单元（IMU）采样的协方差。
对于按每平方根秒给出的偏差随机游走标准差，
:math:`Q_b` 的偏差分块为
:math:`\operatorname{diag}(\sigma_{bax}^2,\sigma_{bay}^2,\sigma_{b\omega}^2)\Delta t`；
其余元素为零。仿真加入相互独立的高斯传感器噪声和固定偏差，而滤波器允许这些偏差缓慢变化。

GPS 校正与中断
--------------

GPS 提供 :math:`\mathbf{z}=[p_x,p_y]^T`，测量矩阵为
:math:`H=[I_2\;0_{2\times6}]`，协方差为
:math:`R_{\mathrm{gps}}=\sigma_{\mathrm{gps}}^2 I_2`。

.. math::

   S &= HP^{-}H^T+R_{\mathrm{gps}},\\
   K &= P^{-}H^T S^{-1},\\
   \mathbf{x}^{+} &= \mathbf{x}^{-}+K(\mathbf{z}-H\mathbf{x}^{-}),\\
   P^{+} &= (I-KH)P^{-}(I-KH)^T+KR_{\mathrm{gps}}K^T.

增益通过求解线性方程组得到，Joseph 协方差更新有助于保持数值上的对称性和半正定性。在相邻两次 GPS 定位之间以及整个中断期间，只进行惯性测量单元（IMU）预测。不确定度在中断期间可能增大，并在 GPS 校正恢复后减小。

不含偏差估计的对比
------------------

基线扩展卡尔曼滤波（EKF）只估计 :math:`[p_x,p_y,v_x,v_y,\psi]^T`。它假设加速度计偏差和陀螺仪偏差为零，因此使用未经偏差补偿的原始惯性测量单元（IMU）测量。其预测使用 :math:`F` 的前五行和前五列，以及 :math:`G` 的前五行，并且不含偏差随机游走项。GPS 仍然通过互协方差校正位置、速度和航向。因此它不同于仅使用惯性测量单元（IMU）的航位推算，后者从不接收 GPS 校正。

三种方法使用完全相同的惯性测量单元（IMU）采样，包括相同的非零传感器偏差和噪声。两种扩展卡尔曼滤波（EKF）接收相同的 GPS 定位和中断时段，并以相同的位姿、速度及相应协方差为初值。只有八维状态的扩展卡尔曼滤波（EKF）估计并补偿这些偏差。红色虚线表示不带偏差估计的五维状态扩展卡尔曼滤波（EKF）。

解析参考轨迹与滤波器的离散积分器无关。固定的随机种子使本示例可以复现，误差图给出每种方法的欧氏位置误差。这些合成结果并不是对真实传感器精度的结论。

代码
------

.. autofunction:: Localization.gps_imu_fusion.gps_imu_fusion.predict

.. autofunction:: Localization.gps_imu_fusion.gps_imu_fusion.update_gps

.. autofunction:: Localization.gps_imu_fusion.gps_imu_fusion.simulate

参考文献
--------

- `Oliver J. Woodman, An introduction to inertial navigation, University of Cambridge, 2007 <https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf>`_
- :doc:`../extended_kalman_filter_localization_files/extended_kalman_filter_localization`
