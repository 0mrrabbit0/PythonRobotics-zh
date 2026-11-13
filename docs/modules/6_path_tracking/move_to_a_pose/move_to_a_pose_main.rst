移动到位姿控制
----------------------

本节给出 PathFinderController 的逻辑，它把车辆从起始位姿 (x, y, theta) 驱动到目标位姿。下方给出移动到位姿控制的仿真。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Control/move_to_pose/animation.gif

代码链接
~~~~~~~~~~~~~~~

.. autofunction:: PathTracking.move_to_pose.move_to_pose.move_to_pose


非完整系统的位置控制
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

本节说明带约束系统（非完整系统）的位置控制器逻辑。

1 自由度（DOF）系统的位置控制相当直接：只需计算位置误差，再乘以比例增益，即可得到控制指令。系统的执行器接收该指令，并把系统驱动到目标位置。该控制器可以很容易地扩展到更高维（例如，二维位置控制使用 Kp_x 与 Kp_y 增益）。在这些系统中，控制指令的数量等于自由度的数量（完整系统）。

要描述二维平面上车辆的位形，需要三个自由度（即 x、y 与 theta）。但驱动车辆只需要两个控制指令（theta_engine 与 theta_steering_wheel）。这一差别来自 x 与 y 这两个自由度之间的约束。delta_x 与 delta_y 之间的关系由 theta_steering_wheel 决定。

注意，车辆通常是非完整系统；若路面打滑，车辆会变成完整系统，从而需要三个独立指令才能控制。

PathFinderController 类
~~~~~~~~~~~~~~~~~~~~~~~~~~

构造函数
^^^^^^^^^^^

.. code-block:: ipython3

   PathFinderController(Kp_rho, Kp_alpha, Kp_beta)

构造 PathFinderController 的实例，用于在二维平面上导航三自由度轮式机器人。

参数：

- | **Kp_rho** ：使机器人沿直线移向目标的线速度增益
- | **Kp_alpha** ：使机器人朝向目标旋转的角速度增益
- | **Kp_beta** ：用于平滑并入目标角度的偏置角速度增益（即帮助机器人航向角与目标角度保持平行。）


成员函数
^^^^^^^^^^^^^^^^^^

.. code-block:: ipython3

   calc_control_command(x_diff, y_diff, theta, theta_goal)

返回线速度与角速度的控制指令，以及到目标的距离。

参数：

- | **x_diff** ：目标相对于机器人当前位置在 x 方向上的位置
- | **y_diff** ：目标相对于机器人当前位置在 y 方向上的位置
- | **theta** ：机器人相对于 x 轴的当前航向角
- | **theta_goal** ：机器人相对于 x 轴的目标角度

返回值：

- | **rho** ：机器人与目标位置之间的距离
- | **v** ：指令线速度
- | **w** ：指令角速度

算法原理
~~~~~~~~~~~~~~~~~~~~~~~~~~~
机器人与目标位置之间的距离 :math:`\rho` 按下式计算

.. math::
 \rho = \sqrt{(x_{robot} - x_{target})^2 + (y_{robot} - y_{target})^2}.

距离 :math:`\rho` 用于确定机器人速度。其思路是：机器人越接近目标，速度越低。

.. math::
 v = K_P{_\rho} \times \rho\qquad
 :label: move_to_a_pose_eq1

注意，在具体应用中，需要把速度增益 :math:`K_P{_\rho}` 调节到合适的数值。

为使机器人转向，并将其航向角 :math:`\theta` 对准目标位置（而不是目标朝向）:math:`\rho \vec{u}`，需要计算角度差 :math:`\alpha`。 

.. math::
 \alpha = (\arctan2(y_{diff}, x_{diff}) - \theta + \pi) mod (2\pi) - \pi

项 :math:`mod(2\pi)` 用于把角度映射到 :math:`[-\pi, \pi)` 区间。

最后，为校正机器人的朝向，需要计算机器人的朝向误差 :math:`\beta`。

.. math::
 \beta = (\theta_{goal} - \theta - \alpha + \pi) mod (2\pi) - \pi

注意，为了在机器人接近目标时抵消 :math:`\alpha` 的影响，式中包含项

:math:`-\alpha`。

最终的角速度指令为

.. math::
 \omega = K_P{_\alpha} \alpha - K_P{_\beta} \beta\qquad
 :label: move_to_a_pose_eq2
 
线速度与角速度（方程 :eq:`move_to_a_pose_eq1` 与 :eq:`move_to_a_pose_eq2`）是该算法的输出。

移动到位姿机器人（类）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

本程序（move_to_pose_robot.py）提供 Robot 类，用于定义规格不同的机器人。
使用该类可以同时仿真不同的机器人，并比较参数设置的效果。

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Control/move_to_pos_robot_class/animation.gif

注意：Robot 类基于 'move_to_pose.py' 中的 PathFinderController 类。

Robot 类
^^^^^^^^^^^^

构造函数
^^^^^^^^^^^^

.. code-block:: ipython3

    Robot(name, color, max_linear_speed, max_angular_speed, path_finder_controller)

构造在二维平面上导航的三自由度轮式 Robot 实例。

参数：

- | **name** ：（字符串）机器人的名称
- | **color** ：（字符串）机器人的颜色
- | **max_linear_speed** ：（浮点数）机器人能够达到的最大线速度
- | **max_angular_speed** ：（浮点数）机器人绕其竖直轴旋转的最大角速度
- | **path_finder_controller** ：（PathFinderController）可配置的控制器，用于寻找路径并计算指令线速度与角速度。

成员函数
^^^^^^^^^^^^^^^^^^^

.. code-block:: ipython3

    set_start_target_poses(pose_start, pose_target)

设置机器人的起始位置与目标位置。

参数：

- | **pose_start** ：（Pose）机器人的起始位置（参见 Pose 类）
- | **pose_target** ：（Pose）机器人的目标位置（参见 Pose 类）

.. code-block:: ipython3

    move(dt)

将机器人移动一个时间步。

参数：

- | **dt** ：<浮点数> 时间增量



参考文献
~~~~~~~~~~~~
- PathFinderController 类
-  `P. I. Corke, "Robotics, Vision and Control" \| SpringerLink
   p102 <https://link.springer.com/book/10.1007/978-3-642-20144-8>`__
