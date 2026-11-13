.. _Graph SLAM Formulation:

基于图的 SLAM 问题表述
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
作者 Jeff Irion

问题表述
^^^^^^^^^^^^^^^^^^^

设机器人在环境中的轨迹由一列 :math:`N` 个位姿表示：
:math:`\mathbf{p}_1, \mathbf{p}_2, \ldots, \mathbf{p}_N`。每个位姿位于某个流形上：:math:`\mathbf{p}_i \in \mathcal{M}`。基于图的 SLAM 中所用的简单流形包括一维、二维与三维空间，即 :math:`\mathbb{R}`、:math:`\mathbb{R}^2` 与 :math:`\mathbb{R}^3`。这些环境是*直线型*的，其中没有朝向这一概念。在 :math:`SE(2)` 问题设定中，机器人的位姿由其在 :math:`\mathbb{R}^2` 中的位置以及朝向 :math:`\theta` 组成。类似地，在 :math:`SE(3)` 中，机器人的位姿由其在 :math:`\mathbb{R}^3` 中的位置以及朝向组成，朝向可用欧拉角、四元数或 :math:`SO(3)` 旋转矩阵表示。

机器人探索环境时，采集一组共 :math:`M` 个测量 :math:`\mathcal{Z} = \{\mathbf{z}_j\}`。这类测量的例子包括里程计、GPS 与 IMU 数据。给定一组位姿 :math:`\mathbf{p}_1, \ldots, \mathbf{p}_N`，可以计算估计测量 :math:`\hat{\mathbf{z}}_j(\mathbf{p}_1, \ldots, \mathbf{p}_N)`。进而可以计算测量 :math:`j` 的*残差* :math:`\mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j)`。残差的公式取决于测量的类型。例如，设 :math:`\mathbf{z}_1` 为机器人从 :math:`\mathbf{p}_1` 运动到 :math:`\mathbf{p}_2` 时采集的里程计测量。期望测量与残差按如下方式计算

.. math::

   \begin{aligned}
       \hat{\mathbf{z}}_1(\mathbf{p}_1, \mathbf{p}_2) &= \mathbf{p}_2 \ominus \mathbf{p}_1 \\
       \mathbf{e}_1(\mathbf{z}_1, \hat{\mathbf{z}}_1) &= \mathbf{z}_1 \ominus \hat{\mathbf{z}}_1 = \mathbf{z}_1 \ominus (\mathbf{p}_2 \ominus \mathbf{p}_1),\end{aligned}

其中算子 :math:`\ominus` 表示逆位姿复合。把测量 :math:`\mathbf{z}_j` 建模为带有独立零均值高斯噪声的量，其协方差矩阵为 :math:`\Omega_j^{-1}`；称 :math:`\Omega_j` 为测量 :math:`j` 的*信息矩阵*。亦即，

.. math::
    p(\mathbf{z}_j \ | \ \mathbf{p}_1, \ldots, \mathbf{p}_N) = \eta_j \exp (-\mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j))^{\mathsf{T}}\Omega_j \mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j)
    :label: infomat

其中 :math:`\eta_j` 是归一化常数。

Graph SLAM 的目标，是在给定测量 :math:`\mathcal{Z} = \{\mathbf{z}_j\}` 的条件下，求出位姿的最大似然集合；也就是求

.. math:: \mathop{\mathrm{arg\,max}}_{\mathbf{p}_1, \ldots, \mathbf{p}_N} \ p(\mathbf{p}_1, \ldots, \mathbf{p}_N \ | \ \mathcal{Z})

利用贝叶斯公式，可以把这个概率写成

.. math::
    \begin{aligned}
       p(\mathbf{p}_1, \ldots, \mathbf{p}_N \ | \ \mathcal{Z}) &= \frac{p( \mathcal{Z} \ | \ \mathbf{p}_1, \ldots, \mathbf{p}_N) p(\mathbf{p}_1, \ldots, \mathbf{p}_N) }{ p(\mathcal{Z}) } \notag \\
       &\propto p( \mathcal{Z} \ | \ \mathbf{p}_1, \ldots, \mathbf{p}_N)
    \end{aligned}
    :label: bayes

这是因为 :math:`p(\mathcal{Z})` 是常数（尽管是未知常数），并且假定 :math:`p(\mathbf{p}_1, \ldots, \mathbf{p}_N)` 服从均匀分布。因此，可以用式 :eq:`infomat` 与式 :eq:`bayes` 把 Graph SLAM 的优化简化如下：

.. math::

   \begin{aligned}
       \mathop{\mathrm{arg\,max}}_{\mathbf{p}_1, \ldots, \mathbf{p}_N} \ p(\mathbf{p}_1, \ldots, \mathbf{p}_N \ | \ \mathcal{Z}) &= \mathop{\mathrm{arg\,max}}_{\mathbf{p}_1, \ldots, \mathbf{p}_N} \ p( \mathcal{Z} \ | \ \mathbf{p}_1, \ldots, \mathbf{p}_N) \\
       &= \mathop{\mathrm{arg\,max}}_{\mathbf{p}_1, \ldots, \mathbf{p}_N} \prod_{j=1}^M p(\mathbf{z}_j \ | \ \mathbf{p}_1, \ldots, \mathbf{p}_N) \\
       &= \mathop{\mathrm{arg\,max}}_{\mathbf{p}_1, \ldots, \mathbf{p}_N} \prod_{j=1}^M \exp \left( -(\mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j))^{\scriptstyle{\mathsf{T}}}\Omega_j \mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j) \right) \\
       &= \mathop{\mathrm{arg\,min}}_{\mathbf{p}_1, \ldots, \mathbf{p}_N} \sum_{j=1}^M (\mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j))^{\scriptstyle{\mathsf{T}}}\Omega_j \mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j).\end{aligned}

定义

.. math:: \chi^2 := \sum_{j=1}^M (\mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j))^{\scriptstyle{\mathsf{T}}}\Omega_j \mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j),

所要最小化的就是这个量。

维数与位姿表示
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

继续讨论之前，先说明该问题的维数。其中包括：

-  一组 :math:`N` 个位姿
   :math:`\mathbf{p}_1, \mathbf{p}_2, \ldots, \mathbf{p}_N`，每个位姿都位于流形 :math:`\mathcal{M}` 上

   -  每个位姿 :math:`\mathbf{p}_i` 表示为 :math:`\mathbb{R}^d` （的某个子集）中的向量。例如：

      -  :math:`SE(2)` 位姿通常表示为
         :math:`(x, y, \theta)`，因此 :math:`d = 3`。

      -  :math:`SE(3)` 位姿通常表示为
         :math:`(x, y, z, q_x, q_y, q_z, q_w)`，其中 :math:`(x, y, z)`
         是 :math:`\mathbb{R}^3` 中的一个点，
         :math:`(q_x, q_y, q_z, q_w)` 是一个*四元数*，因而
         :math:`d = 7`。关于 :math:`SE(3)`
         参数化与位姿变换的更多内容，见
         [blanco2010tutorial]_。

   -  还需要能够把每个位姿紧凑地表示为
      :math:`\mathbb{R}^c` （的某个子集）中的向量。

      -  :math:`SE(2)` 位姿有三个自由度，
         :math:`(x, y, \theta)` 这一表示仍然足够，并且
         :math:`c=3`。

      -  :math:`SE(3)` 位姿只有六个自由度，可以把它紧凑地表示为 :math:`(x, y, z, q_x, q_y, q_z)`，
         因此 :math:`c=6`。

   -  当其中一个或两个位姿采用紧凑表示时，用算子 :math:`\boxplus` 表示位姿复合。
      按上下文需要，输出可以是 :math:`\mathcal{M}` 中的一个位姿，也可以是
      :math:`\mathbb{R}^c` 中的一个向量。

-  一组 :math:`M` 个测量
   :math:`\mathcal{Z} = \{\mathbf{z}_1, \mathbf{z}_2, \ldots, \mathbf{z}_M\}`

   -  每个测量的维数可以各不相同，下文用
      :math:`\bullet` 表示一个“通配”变量。

   -  测量 :math:`\mathbf{z}_j \in \mathbb{R}^\bullet` 具有相应的信息矩阵
      :math:`\Omega_j \in \mathbb{R}^{\bullet \times \bullet}` 以及残差函数
      :math:`\mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j) = \mathbf{e}_j(\mathbf{z}_j, \mathbf{p}_1, \ldots, \mathbf{p}_N) \in \mathbb{R}^\bullet`。

   -  理论上，一条测量可以只约束 1 个位姿，也可以约束全部 :math:`N` 个位姿。实践中，每条测量通常只约束 1 个或 2 个位姿。

Graph SLAM 算法
^^^^^^^^^^^^^^^^^^^^

Graph SLAM 中的“图”，是指把该问题看成一张图。这张图有顶点集合 :math:`\mathcal{V}`，其中含 :math:`N` 个顶点，每个顶点 :math:`v_i` 关联一个位姿 :math:`\mathbf{p}_i`。同样，这张图有边集合 :math:`\mathcal{E}`，其中含 :math:`M` 条边，每条边 :math:`e_j` 关联一个测量 :math:`\mathbf{z}_j`。实践中，图中的边要么是一元的（即自环），要么是二元的。（注：:math:`e_j` 指与测量 :math:`\mathbf{z}_j` 相关联的图中的边，而 :math:`\mathbf{e}_j` 指与 :math:`\mathbf{z}_j` 相关联的残差函数。）关于 Graph SLAM 算法的更多内容，见 [grisetti2010tutorial]_。

目标是优化

.. math:: \chi^2 = \sum_{e_j \in \mathcal{E}} \mathbf{e}_j^{\scriptstyle{\mathsf{T}}}\Omega_j \mathbf{e}_j.

设 :math:`\mathbf{x}_i \in \mathbb{R}^c` 是位姿 :math:`\mathbf{p}_i \in \mathcal{M}` 的紧凑表示，并令

.. math:: \mathbf{x} := \begin{bmatrix} \mathbf{x}_1 \\ \mathbf{x}_2 \\ \vdots \\ \mathbf{x}_N \end{bmatrix} \in \mathbb{R}^{cN}

下面迭代求解这个优化问题。令

.. math:: \mathbf{x}^{k+1} := \mathbf{x}^k \boxplus \Delta \mathbf{x}^k = \begin{bmatrix} \mathbf{x}_1 \boxplus \Delta \mathbf{x}_1 \\ \mathbf{x}_2 \boxplus \Delta \mathbf{x}_2 \\ \vdots \\ \mathbf{x}_N \boxplus \Delta \mathbf{x}_2 \end{bmatrix}
    :label: update

第 :math:`k+1` 次迭代处的 :math:`\chi^2` 误差为

.. math:: \chi_{k+1}^2 = \sum_{e_j \in \mathcal{E}} \underbrace{\left[ \mathbf{e}_j(\mathbf{x}^{k+1}) \right]^{\scriptstyle{\mathsf{T}}}}_{1 \times \bullet} \underbrace{\Omega_j}_{\bullet \times \bullet} \underbrace{\mathbf{e}_j(\mathbf{x}^{k+1})}_{\bullet \times 1}.
    :label: chisq_at_kplusone

把残差线性化为：

.. math::
    \begin{aligned}
        \mathbf{e}_j(\mathbf{x}^{k+1}) &= \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k) \\
        &\approx \mathbf{e}_j(\mathbf{x}^{k}) + \frac{\partial}{\partial \Delta \mathbf{x}^k} \left[ \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k) \right] \Delta \mathbf{x}^k \\
        &= \mathbf{e}_j(\mathbf{x}^{k}) + \left( \left. \frac{\partial \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)} \right|_{\Delta \mathbf{x}^k = \mathbf{0}} \right) \frac{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial \Delta \mathbf{x}^k} \Delta \mathbf{x}^k.
    \end{aligned}
    :label: linearization

把 :eq:`linearization` 代入 :eq:`chisq_at_kplusone`，得到：

.. math::

   \begin{aligned}
       \chi_{k+1}^2 &\approx \ \ \ \ \ \sum_{e_j \in \mathcal{E}} \underbrace{[ \mathbf{e}_j(\mathbf{x}^k)]^{\scriptstyle{\mathsf{T}}}}_{1 \times \bullet} \underbrace{\Omega_j}_{\bullet \times \bullet} \underbrace{\mathbf{e}_j(\mathbf{x}^k)}_{\bullet \times 1} \notag \\
       &\hphantom{\approx} \ \ \ + \sum_{e_j \in \mathcal{E}} \underbrace{[ \mathbf{e}_j(\mathbf{x^k}) ]^{\scriptstyle{\mathsf{T}}}}_{1 \times \bullet} \underbrace{\Omega_j}_{\bullet \times \bullet} \underbrace{\left( \left. \frac{\partial \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)} \right|_{\Delta \mathbf{x}^k = \mathbf{0}} \right)}_{\bullet \times dN} \underbrace{\frac{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial \Delta \mathbf{x}^k}}_{dN \times cN} \underbrace{\Delta \mathbf{x}^k}_{cN \times 1} \notag \\
       &\hphantom{\approx} \ \ \ + \sum_{e_j \in \mathcal{E}} \underbrace{(\Delta \mathbf{x}^k)^{\scriptstyle{\mathsf{T}}}}_{1 \times cN} \underbrace{ \left( \frac{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial \Delta \mathbf{x}^k} \right)^{\scriptstyle{\mathsf{T}}}}_{cN \times dN} \underbrace{\left( \left. \frac{\partial \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)} \right|_{\Delta \mathbf{x}^k = \mathbf{0}} \right)^{\scriptstyle{\mathsf{T}}}}_{dN \times \bullet} \underbrace{\Omega_j}_{\bullet \times \bullet} \underbrace{\left( \left. \frac{\partial \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)} \right|_{\Delta \mathbf{x}^k = \mathbf{0}} \right)}_{\bullet \times dN} \underbrace{\frac{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial \Delta \mathbf{x}^k}}_{dN \times cN} \underbrace{\Delta \mathbf{x}^k}_{cN \times 1} \notag \\
       &= \chi_k^2 + 2 \mathbf{b}^{\scriptstyle{\mathsf{T}}}\Delta \mathbf{x}^k + (\Delta \mathbf{x}^k)^{\scriptstyle{\mathsf{T}}}H \Delta \mathbf{x}^k,  \notag\end{aligned}

其中

.. math::

   \begin{aligned}
       \mathbf{b}^{\scriptstyle{\mathsf{T}}}&= \sum_{e_j \in \mathcal{E}} \underbrace{[ \mathbf{e}_j(\mathbf{x^k}) ]^{\scriptstyle{\mathsf{T}}}}_{1 \times \bullet} \underbrace{\Omega_j}_{\bullet \times \bullet} \underbrace{\left( \left. \frac{\partial \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)} \right|_{\Delta \mathbf{x}^k = \mathbf{0}} \right)}_{\bullet \times dN} \underbrace{\frac{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial \Delta \mathbf{x}^k}}_{dN \times cN} \\
       H &= \sum_{e_j \in \mathcal{E}} \underbrace{ \left( \frac{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial \Delta \mathbf{x}^k} \right)^{\scriptstyle{\mathsf{T}}}}_{cN \times dN} \underbrace{\left( \left. \frac{\partial \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)} \right|_{\Delta \mathbf{x}^k = \mathbf{0}} \right)^{\scriptstyle{\mathsf{T}}}}_{dN \times \bullet} \underbrace{\Omega_j}_{\bullet \times \bullet} \underbrace{\left( \left. \frac{\partial \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)} \right|_{\Delta \mathbf{x}^k = \mathbf{0}} \right)}_{\bullet \times dN} \underbrace{\frac{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial \Delta \mathbf{x}^k}}_{dN \times cN}.\end{aligned}

采用这一记号，最优更新为

.. math:: \Delta \mathbf{x}^k = -H^{-1} \mathbf{b}.  \label{eq:deltax}

通过 :eq:`update` 把该更新作用到位姿上，并重复直到收敛。


.. [blanco2010tutorial] Blanco, J.-L.A tutorial onSE(3) transformation parameterization and on-manifold optimization.University of Malaga, Tech. Rep 3(2010)
.. [grisetti2010tutorial] Grisetti, G., Kummerle, R., Stachniss, C., and Burgard, W.A tutorial on graph-based SLAM.IEEE Intelligent Transportation Systems Magazine 2, 4 (2010), 31–43.

