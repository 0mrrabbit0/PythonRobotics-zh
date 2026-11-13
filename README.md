> **中文学习版说明（非官方译本）**
>
> 本仓库是 [Atsushi Sakai](https://github.com/AtsushiSakai) 的开源项目 [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) 的**非官方中文翻译学习版**，供中文读者学习机器人算法。算法代码、数值结果、应用程序接口、标识符与文件布局与上游一致；注释、文档字符串和文档已译为中文。
>
> - 原作者与版权：Atsushi Sakai 及其他贡献者。许可证为 MIT，全文见 [`LICENSE`](LICENSE)。版权归原作者所有。
> - 原仓库：https://github.com/AtsushiSakai/PythonRobotics （本译本基于上游 `master` 提交 [`ef64608ceb0012f2a0b3d532fddc80243e807cfa`](https://github.com/AtsushiSakai/PythonRobotics/commit/ef64608ceb0012f2a0b3d532fddc80243e807cfa)，2026-11-10）
> - 翻译问题（错译、漏译、术语）请在本仓库提交 Issue：https://github.com/0mrrabbit0/PythonRobotics-zh/issues
> - 算法错误、数值问题与上游缺陷请报告到原仓库：https://github.com/AtsushiSakai/PythonRobotics/issues

<img src="https://github.com/AtsushiSakai/PythonRobotics/raw/master/icon.png?raw=true" align="right" width="300" alt="页眉图片"/>

# PythonRobotics
![GitHub_Action_Linux_CI](https://github.com/AtsushiSakai/PythonRobotics/workflows/Linux_CI/badge.svg)
![GitHub_Action_MacOS_CI](https://github.com/AtsushiSakai/PythonRobotics/workflows/MacOS_CI/badge.svg)
![GitHub_Action_Windows_CI](https://github.com/AtsushiSakai/PythonRobotics/workflows/Windows_CI/badge.svg)
[![Build status](https://ci.appveyor.com/api/projects/status/sb279kxuv1be391g?svg=true)](https://ci.appveyor.com/project/AtsushiSakai/pythonrobotics)

机器人算法的 Python 示例代码与[教材](https://atsushisakai.github.io/PythonRobotics/index.html)。


# <a id="table-of-contents"></a>目录
   * [这是什么？](#what-is-this)
   * [运行环境](#requirements)
   * [文档](#documentation)
   * [如何使用](#how-to-use)
   * [定位](#localization)
      * [扩展卡尔曼滤波定位](#extended-kalman-filter-localization)
      * [带偏差估计的 GPS/IMU 融合定位](#gpsimu-fusion-localization-with-bias-estimation)
      * [粒子滤波定位](#particle-filter-localization)
      * [直方图滤波定位](#histogram-filter-localization)
   * [建图](#mapping)
      * [高斯栅格地图](#gaussian-grid-map)
      * [射线投射栅格地图](#ray-casting-grid-map)
      * [激光雷达转栅格地图](#lidar-to-grid-map)
      * [k-均值目标聚类](#k-means-object-clustering)
      * [矩形拟合](#rectangle-fitting)
   * [SLAM](#slam)
      * [迭代最近点（ICP）匹配](#iterative-closest-point-icp-matching)
      * [FastSLAM 1.0](#fastslam-10)
   * [路径规划](#path-planning)
      * [动态窗口法](#dynamic-window-approach)
      * [基于栅格的搜索](#grid-based-search)
         * [Dijkstra 算法](#dijkstra-algorithm)
         * [A\* 算法](#a-algorithm)
         * [D\* 算法](#d-algorithm)
         * [D\* Lite 算法](#d-lite-algorithm)
         * [势场法](#potential-field-algorithm)
         * [基于栅格的覆盖路径规划](#grid-based-coverage-path-planning)
         * [粒子群优化（PSO）](#particle-swarm-optimization-pso)
      * [状态格规划](#state-lattice-planning)
         * [有偏极坐标采样](#biased-polar-sampling)
         * [车道采样](#lane-sampling)
      * [概率路线图（PRM）规划](#probabilistic-road-map-prm-planning)
      * [快速扩展随机树（RRT）](#rapidly-exploring-random-trees-rrt)
         * [RRT\*](#rrt)
         * [结合 Reeds-Shepp 路径的 RRT\*](#rrt-with-reeds-shepp-path)
         * [LQR-RRT\*](#lqr-rrt)
      * [五次多项式规划](#quintic-polynomials-planning)
      * [Reeds-Shepp 规划](#reeds-shepp-planning)
      * [基于 LQR 的路径规划](#lqr-based-path-planning)
      * [Frenet 坐标系中的最优轨迹](#optimal-trajectory-in-a-frenet-frame)
   * [路径跟踪](#path-tracking)
      * [驶向目标位姿](#move-to-a-pose-control)
      * [Stanley 控制](#stanley-control)
      * [后轮反馈控制](#rear-wheel-feedback-control)
      * [线性二次调节器（LQR）速度与转向控制](#linearquadratic-regulator-lqr-speed-and-steering-control)
      * [模型预测速度与转向控制](#model-predictive-speed-and-steering-control)
      * [基于 C-GMRES 的非线性模型预测控制](#nonlinear-model-predictive-control-with-c-gmres)
   * [机械臂导航](#arm-navigation)
      * [N 关节机械臂到点控制](#n-joint-arm-to-point-control)
      * [带避障的机械臂导航](#arm-navigation-with-obstacle-avoidance)
   * [空中导航](#aerial-navigation)
      * [无人机三维轨迹跟踪](#drone-3d-trajectory-following)
      * [火箭动力着陆](#rocket-powered-landing)
   * [双足](#bipedal)
      * [基于倒立摆的双足规划器](#bipedal-planner-with-inverted-pendulum)
   * [许可证](#license)
   * [使用案例](#use-case)
   * [贡献](#contribution)
   * [引用](#citing)
   * [支持本项目](#support)
   * [赞助者](#sponsors)
      * [JetBrains](#JetBrains)
      * [1Password](#1password)
   * [作者](#authors)

# <a id="what-is-this"></a>PythonRobotics 是什么？

PythonRobotics 是一套机器人算法的 Python 代码合集，以及一本[教材](https://atsushisakai.github.io/PythonRobotics/index.html)。

特点：

1. 便于阅读，能够看清每种算法的基本思想。

2. 选取了广泛使用、注重实用的算法。

3. 依赖尽量少。

更多介绍见文档

- [入门 — PythonRobotics 文档](https://atsushisakai.github.io/PythonRobotics/modules/0_getting_started/1_what_is_python_robotics.html)

或这段视频：

- [PythonRobotics 项目语音概述](https://www.youtube.com/watch?v=uMeRnNoJAfU)

或这篇论文：

- [\[1808\.10703\] PythonRobotics: a Python code collection of robotics algorithms](https://arxiv.org/abs/1808.10703)（[BibTeX](https://github.com/AtsushiSakai/PythonRoboticsPaper/blob/master/python_robotics.bib)）


# <a id="requirements"></a>运行代码的环境

运行各个示例需要：

- [Python 3.13.x](https://www.python.org/)

- [NumPy](https://numpy.org/)

- [SciPy](https://scipy.org/)

- [Matplotlib](https://matplotlib.org/)

- [cvxpy](https://www.cvxpy.org/)

开发还需要：

- [pytest](https://pytest.org/)（单元测试）

- [pytest-xdist](https://pypi.org/project/pytest-xdist/)（并行单元测试）

- [mypy](https://mypy-lang.org/)（类型检查）

- [sphinx](https://www.sphinx-doc.org/)（生成文档）

- [pycodestyle](https://pypi.org/project/pycodestyle/)（代码风格检查）

# <a id="documentation"></a>文档（教材）

本 README 只展示项目中的一部分示例。

若希望查看其余示例，或了解每种算法的数学背景，

可以在线阅读完整文档（教材）：[Welcome to PythonRobotics’s documentation\! — PythonRobotics documentation](https://atsushisakai.github.io/PythonRobotics/index.html)

本仓库 `docs/` 中的教材正文已译为中文，可用 Sphinx 在本地生成。全部动画 GIF 存放在：[AtsushiSakai/PythonRoboticsGifs: Animation gifs of PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs)

# <a id="how-to-use"></a>如何使用

1. 克隆本中文学习版仓库。

   ```terminal
   git clone https://github.com/0mrrabbit0/PythonRobotics-zh.git
   ```

   上游英文原仓库（算法问题请报到这里）：https://github.com/AtsushiSakai/PythonRobotics


2. 安装依赖库。

- 使用 conda：

  ```terminal
  conda env create -f requirements/environment.yml
  ```

- 使用 pip：

  ```terminal
  pip install -r requirements/requirements.txt
  ```


3. 在各个目录中运行 Python 脚本。

4. 若觉得有用，请给原仓库点一颗 star :smiley:。

# <a id="localization"></a>定位

## <a id="extended-kalman-filter-localization"></a>扩展卡尔曼滤波定位

<img src="https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/extended_kalman_filter/animation.gif" width="640" alt="EKF 示意图">

参考

- [文档](https://atsushisakai.github.io/PythonRobotics/modules/2_localization/extended_kalman_filter_localization_files/extended_kalman_filter_localization.html)

## <a id="gpsimu-fusion-localization-with-bias-estimation"></a>带偏差估计的 GPS/IMU 融合定位

![带偏差估计的 GPS/IMU 融合定位](https://raw.githubusercontent.com/AtsushiSakai/PythonRoboticsGifs/5abf69412ae056a2f30ba562a1c04600b6062fe4/Localization/gps_imu_fusion/animation.gif)

扩展卡尔曼滤波把机体坐标系下的加速度计、陀螺仪读数与频率更低的 GPS 位置融合在一起，估计传感器偏差，并在 GPS 短时中断期间继续做惯性预测。动画把该方法与不做偏差估计的 EKF、以及使用同一组传感器测量的纯 IMU 航位推算作对比。图中还画出加速度计与陀螺仪的偏差估计及其真值，并给出由协方差得到的 3σ 位置椭圆和偏差不确定度区间。

- [文档](https://atsushisakai.github.io/PythonRobotics/modules/2_localization/gps_imu_fusion/gps_imu_fusion.html)
- [示例代码](Localization/gps_imu_fusion/gps_imu_fusion.py)

## <a id="particle-filter-localization"></a>粒子滤波定位

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/particle_filter/animation.gif)

这是使用粒子滤波（PF）的传感器融合定位。

蓝线是真实轨迹，黑线是航位推算轨迹，

红线是粒子滤波估计轨迹。

假设机器人能够测量到各个路标（RFID）的距离。

这些测量用于粒子滤波定位。

参考

- [PROBABILISTIC ROBOTICS](http://www.probabilistic-robotics.org/)


## <a id="histogram-filter-localization"></a>直方图滤波定位

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/histogram_filter/animation.gif)

这是直方图滤波的二维定位示例。

红叉是真实位置，黑点是 RFID 位置。

蓝色栅格表示直方图滤波给出的位置概率。

本仿真中 x、y 未知，航向角已知。

滤波器综合速度输入和 RFID 距离观测来完成定位。

不需要给定初始位置。

参考

- [PROBABILISTIC ROBOTICS](http://www.probabilistic-robotics.org/)

# <a id="mapping"></a>建图

## <a id="gaussian-grid-map"></a>高斯栅格地图

这是二维高斯栅格建图示例。

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/gaussian_grid_map/animation.gif)

## <a id="ray-casting-grid-map"></a>射线投射栅格地图

这是二维射线投射栅格建图示例。

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/raycasting_grid_map/animation.gif)

## <a id="lidar-to-grid-map"></a>激光雷达转栅格地图

本示例说明如何把二维测距结果转换成栅格地图。

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/lidar_to_grid_map/animation.gif)

## <a id="k-means-object-clustering"></a>k-均值目标聚类

这是用 k-均值算法做的二维目标聚类。

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/kmeans_clustering/animation.gif)

## <a id="rectangle-fitting"></a>矩形拟合

这是用于车辆检测的二维矩形拟合。

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/rectangle_fitting/animation.gif)


# <a id="slam"></a>SLAM

同步定位与建图（SLAM）示例。

## <a id="iterative-closest-point-icp-matching"></a>迭代最近点（ICP）匹配

这是用奇异值分解做的二维 ICP 匹配示例。

它可以计算点集与点集之间的旋转矩阵和平移向量。

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/SLAM/iterative_closest_point/animation.gif)

参考

- [Introduction to Mobile Robotics: Iterative Closest Point Algorithm](https://cs.gmu.edu/~kosecka/cs685/cs685-icp.pdf)


## <a id="fastslam-10"></a>FastSLAM 1.0

这是使用 FastSLAM 1.0 的基于特征的 SLAM 示例。

蓝线是真值，黑线是航位推算，红线是 FastSLAM 估计轨迹。

红点是 FastSLAM 的粒子。

黑点是路标，蓝色叉号是 FastSLAM 估计的路标位置。


![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/SLAM/FastSLAM1/animation.gif)


参考

- [PROBABILISTIC ROBOTICS](http://www.probabilistic-robotics.org/)

- [SLAM simulations by Tim Bailey](http://www-personal.acfr.usyd.edu.au/tbailey/software/slam_simulations.htm)


# <a id="path-planning"></a>路径规划

## <a id="dynamic-window-approach"></a>动态窗口法

这是动态窗口法的二维导航示例。

- [The Dynamic Window Approach to Collision Avoidance](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf)

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DynamicWindowApproach/animation.gif)


## <a id="grid-based-search"></a>基于栅格的搜索

### <a id="dijkstra-algorithm"></a>Dijkstra 算法

这是用 Dijkstra 算法在二维栅格上做最短路径规划。

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/Dijkstra/animation.gif)

动画中，青色点是已搜索节点。

### <a id="a-algorithm"></a>A\* 算法

这是用 A\* 算法在二维栅格上做最短路径规划。

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/AStar/animation.gif)

动画中，青色点是已搜索节点。

启发式函数是二维欧氏距离。

### <a id="d-algorithm"></a>D\* 算法

这是用 D\* 算法在二维栅格上做最短路径规划。

![figure at master · nirnayroy/intelligentrobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DStar/animation.gif)

动画展示机器人用 D\* 搜索算法寻找路径并绕开障碍物。

参考

- [D* Algorithm Wikipedia](https://en.wikipedia.org/wiki/D*)

### <a id="d-lite-algorithm"></a>D\* Lite 算法

该算法在两点之间寻找最短路径，并在发现新障碍物时重新规划。这里的实现面向二维栅格。

![D* Lite](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DStarLite/animation.gif)

动画展示机器人寻找路径，并在障碍物被发现时改道，所用算法是 D\* Lite。

参考文献：

- [D* Lite](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf)
- [Improved Fast Replanning for Robot Navigation in Unknown Terrain](http://www.cs.cmu.edu/~maxim/files/dlite_icra02.pdf)

### <a id="potential-field-algorithm"></a>势场法

这是势场法的二维栅格路径规划。

![PotentialField](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/PotentialFieldPlanning/animation.gif)

动画中，蓝色热力图表示每个栅格上的势场值。

参考

- [Robotic Motion Planning:Potential Functions](https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf)

### <a id="grid-based-coverage-path-planning"></a>基于栅格的覆盖路径规划

这是二维栅格覆盖路径规划仿真。

![PotentialField](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/GridBasedSweepCPP/animation.gif)

### <a id="particle-swarm-optimization-pso"></a>粒子群优化（PSO）

这是用粒子群优化算法做的二维路径规划仿真。

![PSO](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ParticleSwarmOptimization/animation.gif)

PSO 是受鸟群行为启发的元启发式优化算法。在路径规划中，粒子在搜索空间中寻找无碰撞路径并避开障碍物。

动画展示粒子（蓝点）从起点（绿色区域）向目标（红星）收敛到最优路径（黄线）。

参考

- [Particle swarm optimization - Wikipedia](https://en.wikipedia.org/wiki/Particle_swarm_optimization)

- [Kennedy, J.; Eberhart, R. (1995). "Particle Swarm Optimization"](https://ieeexplore.ieee.org/document/488968)



## <a id="state-lattice-planning"></a>状态格规划

这段脚本用状态格规划做路径规划。

代码使用模型预测轨迹生成器来求解边界值问题。

参考

- [Optimal rough terrain trajectory generation for wheeled mobile robots](https://journals.sagepub.com/doi/pdf/10.1177/0278364906075328)

- [State Space Sampling of Feasible Motions for High-Performance Mobile Robot Navigation in Complex Environments](https://www.cs.cmu.edu/~alonzo/pubs/papers/JFR_08_SS_Sampling.pdf)


### <a id="biased-polar-sampling"></a>有偏极坐标采样

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/StateLatticePlanner/BiasedPolarSampling.gif)


### <a id="lane-sampling"></a>车道采样

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/StateLatticePlanner/LaneSampling.gif)

## <a id="probabilistic-road-map-prm-planning"></a>概率路线图（PRM）规划

![PRM](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ProbabilisticRoadMap/animation.gif)

该 PRM 规划器使用 Dijkstra 方法做图搜索。

动画中，蓝点是采样点，

青色叉号是 Dijkstra 方法搜索过的点，

红线是 PRM 的最终路径。

参考

- [Probabilistic roadmap \- Wikipedia](https://en.wikipedia.org/wiki/Probabilistic_roadmap)



## <a id="rapidly-exploring-random-trees-rrt"></a>快速扩展随机树（RRT）

### <a id="rrt"></a>RRT\*

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/RRTstar/animation.gif)

这是 RRT\* 路径规划代码。

黑圆是障碍物，绿线是搜索树，红叉是起点和目标点。

参考

- [Incremental Sampling-based Algorithms for Optimal Motion Planning](https://arxiv.org/abs/1005.0416)

- [Sampling-based Algorithms for Optimal Motion Planning](https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=bddbc99f97173430aa49a0ada53ab5bade5902fa)

### <a id="rrt-with-reeds-shepp-path"></a>结合 Reeds-Shepp 路径的 RRT\*

![Robotics/animation.gif at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/RRTStarReedsShepp/animation.gif)

面向汽车机器人的路径规划，使用 RRT\* 与 Reeds-Shepp 路径规划器。

### <a id="lqr-rrt"></a>LQR-RRT\*

这是 LQR-RRT\* 路径规划仿真。

局部规划器使用双积分器运动模型上的 LQR。

![LQR_RRT](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/LQRRRTStar/animation.gif)

参考

- [LQR\-RRT\*: Optimal Sampling\-Based Motion Planning with Automatically Derived Extension Heuristics](https://lis.csail.mit.edu/pubs/perez-icra12.pdf)

- [MahanFathi/LQR\-RRTstar: LQR\-RRT\* method is used for random motion planning of a simple pendulum in its phase plot](https://github.com/MahanFathi/LQR-RRTstar)


## <a id="quintic-polynomials-planning"></a>五次多项式规划

用五次多项式做运动规划。

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/QuinticPolynomialsPlanner/animation.gif)

它可以根据五次多项式计算二维路径以及速度、加速度曲线。

参考

- [Local Path Planning And Motion Control For Agv In Positioning](https://ieeexplore.ieee.org/document/637936/)

## <a id="reeds-shepp-planning"></a>Reeds-Shepp 规划

Reeds-Shepp 路径规划示例。

![RSPlanning](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ReedsSheppPath/animation.gif?raw=true)

参考

- [15.3.2 Reeds\-Shepp Curves](https://lavalle.pl/planning/node822.html)

- [optimal paths for a car that goes both forwards and backwards](https://pdfs.semanticscholar.org/932e/c495b1d0018fd59dee12a0bf74434fac7af4.pdf)

- [ghliu/pyReedsShepp: Implementation of Reeds Shepp curve\.](https://github.com/ghliu/pyReedsShepp)


## <a id="lqr-based-path-planning"></a>基于 LQR 的路径规划

面向双积分器模型、基于 LQR 的路径规划示例。

![RSPlanning](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/LQRPlanner/animation.gif?raw=true)


## <a id="optimal-trajectory-in-a-frenet-frame"></a>Frenet 坐标系中的最优轨迹

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/FrenetOptimalTrajectory/animation.gif)

这是在 Frenet 坐标系中生成最优轨迹。

青色线是目标路线，黑色叉号是障碍物。

红线是预测路径。

参考

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame](https://www.youtube.com/watch?v=Cj6tAQe7UCY)


# <a id="path-tracking"></a>路径跟踪

## <a id="move-to-a-pose-control"></a>驶向目标位姿

这是驶向某一目标位姿的控制仿真。

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Control/move_to_pose/animation.gif)

参考

- [P. I. Corke, "Robotics, Vision and Control" \| SpringerLink p102](https://link.springer.com/book/10.1007/978-3-642-20144-8)


## <a id="stanley-control"></a>Stanley 控制

使用 Stanley 转向控制与 PID 速度控制的路径跟踪仿真。

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/stanley_controller/animation.gif)

参考

- [Stanley: The robot that won the DARPA grand challenge](http://robots.stanford.edu/papers/thrun.stanley05.pdf)

- [Automatic Steering Methods for Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)



## <a id="rear-wheel-feedback-control"></a>后轮反馈控制

使用后轮反馈转向控制与 PID 速度控制的路径跟踪仿真。

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/rear_wheel_feedback/animation.gif)

参考

- [A Survey of Motion Planning and Control Techniques for Self-driving Urban Vehicles](https://arxiv.org/abs/1604.07446)


## <a id="linearquadratic-regulator-lqr-speed-and-steering-control"></a>线性二次调节器（LQR）速度与转向控制

使用 LQR 同时控制速度和转向的路径跟踪仿真。

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/lqr_speed_steer_control/animation.gif)

参考

- [Towards fully autonomous driving: Systems and algorithms \- IEEE Conference Publication](https://ieeexplore.ieee.org/document/5940562/)


## <a id="model-predictive-speed-and-steering-control"></a>模型预测速度与转向控制

使用迭代线性化模型预测控制来调节速度和转向的路径跟踪仿真。

<img src="https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/model_predictive_speed_and_steer_control/animation.gif" width="640" alt="MPC 示意图">

参考

- [文档](https://atsushisakai.github.io/PythonRobotics/modules/6_path_tracking/model_predictive_speed_and_steering_control/model_predictive_speed_and_steering_control.html)

- [Real\-time Model Predictive Control \(MPC\), ACADO, Python \| Work\-is\-Playing](http://grauonline.de/wordpress/?page_id=3244)

## <a id="nonlinear-model-predictive-control-with-c-gmres"></a>基于 C-GMRES 的非线性模型预测控制

使用 C-GMRES 非线性模型预测控制做运动规划与路径跟踪的仿真。

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/cgmres_nmpc/animation.gif)

参考

- [文档](https://atsushisakai.github.io/PythonRobotics/modules/6_path_tracking/cgmres_nmpc/cgmres_nmpc.html)


# <a id="arm-navigation"></a>机械臂导航

## <a id="n-joint-arm-to-point-control"></a>N 关节机械臂到点控制

N 关节机械臂到点控制仿真。

这是一个交互式仿真。

可以在绘图区域单击左键，设置末端执行器的目标位置。

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/ArmNavigation/n_joint_arm_to_point_control/animation.gif)

本仿真中 N = 10，也可以自行修改。

## <a id="arm-navigation-with-obstacle-avoidance"></a>带避障的机械臂导航

带障碍物回避的机械臂导航仿真。

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/ArmNavigation/arm_obstacle_navigation/animation.gif)


# <a id="aerial-navigation"></a>空中导航

## <a id="drone-3d-trajectory-following"></a>无人机三维轨迹跟踪

这是四旋翼的三维轨迹跟踪仿真。

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/AerialNavigation/drone_3d_trajectory_following/animation.gif)

## <a id="rocket-powered-landing"></a>火箭动力着陆

这是火箭动力着陆的三维轨迹生成仿真。

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/AerialNavigation/rocket_powered_landing/animation.gif)

参考

- [文档](https://atsushisakai.github.io/PythonRobotics/modules/8_aerial_navigation/rocket_powered_landing/rocket_powered_landing.html)

# <a id="bipedal"></a>双足

## <a id="bipedal-planner-with-inverted-pendulum"></a>基于倒立摆的双足规划器

这是用倒立摆修正落脚点的双足规划器。

可以设置落脚点，规划器会自动加以调整。

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Bipedal/bipedal_planner/animation.gif)

# <a id="license"></a>许可证

MIT。版权归 Atsushi Sakai 及上游贡献者所有，许可全文见 [`LICENSE`](LICENSE)。本仓库为非官方中文学习译本。

# <a id="use-case"></a>使用案例

若本项目对你的机器人项目有帮助，请通过提交 Issue 告知原作者。

非常欢迎使用了 PythonRobotics 的机器人视频！！

用户评论与参考列表：[users_comments](users_comments.md)

# <a id="contribution"></a>贡献

欢迎任何贡献！！

请阅读这篇文档：[How To Contribute — PythonRobotics documentation](https://atsushisakai.github.io/PythonRobotics/modules/0_getting_started/3_how_to_contribute.html)

翻译问题请提交到本仓库；算法与代码缺陷请提交到上游：https://github.com/AtsushiSakai/PythonRobotics/issues

# <a id="citing"></a>引用

若在学术工作中使用本项目代码，建议引用[原论文](https://arxiv.org/abs/1808.10703)。

若在工业界使用本项目代码，也欢迎直接联系开发者。

# <a id="support"></a>支持本项目

若你或你的公司希望支持这个项目，可以考虑：

- [在 GitHub Sponsors 上赞助 @AtsushiSakai](https://github.com/sponsors/AtsushiSakai)

- [在 Patreon 上成为支持者或赞助者](https://www.patreon.com/myenigma)

- [通过 PayPal 一次性捐赠](https://www.paypal.com/paypalme/myenigmapay/)

若希望以其他方式支持，请提交 Issue 与作者联系。

## <a id="sponsors"></a>赞助者

### <a id="JetBrains"></a>[JetBrains](https://www.jetbrains.com/)

他们为本开源项目的开发提供了 IDE 免费许可。

### <a id="1password"></a>[1Password](https://github.com/1Password/for-open-source)

他们为本开源项目提供了 1Password 团队许可。


# <a id="authors"></a>作者

- 原项目作者：Atsushi Sakai 及其他贡献者。[Contributors to AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics/graphs/contributors)
- 本仓库是上述项目的非官方中文学习译本，不代表原作者。
