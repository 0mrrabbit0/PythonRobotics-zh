# 版权所有 (c) 2020 Jeff Irion 及贡献者
#
# 本文件源自 `graphslam` 包：
#
#   https://github.com/JeffLIrion/python-graphslam

r"""里程计边的类。

"""


import numpy as np
import matplotlib.pyplot as plt


#: 数值微分所用的差分量
EPSILON = 1e-6


class EdgeOdometry:
    r"""表示基于图的同步定位与建图（Graph SLAM）中里程计边的类。

    Parameters
    ----------
    vertices : list[graphslam.vertex.Vertex]
        受该边约束的顶点列表
    information : np.ndarray
        与该边关联的信息矩阵 :math:`\Omega_j`
    estimate : graphslam.pose.se2.PoseSE2
        期望测量 :math:`\mathbf{z}_j`

    Attributes
    ----------
    vertices : list[graphslam.vertex.Vertex]
        受该边约束的顶点列表
    information : np.ndarray
        与该边关联的信息矩阵 :math:`\Omega_j`
    estimate : PoseSE2
        期望测量 :math:`\mathbf{z}_j`

    """
    def __init__(self, vertex_ids, information, estimate, vertices=None):
        self.vertex_ids = vertex_ids
        self.information = information
        self.estimate = estimate
        self.vertices = vertices

    def calc_error(self):
        r"""计算该边的误差：:math:`\mathbf{e}_j \in \mathbb{R}^\bullet`。

        .. math::

           \mathbf{e}_j = \mathbf{z}_j - (p_2 \ominus p_1)


        Returns
        -------
        np.ndarray
            该边的误差

        """
        return (self.estimate - (self.vertices[1].pose - self.vertices[0].pose)).to_compact()

    def calc_chi2(self):
        r"""计算该边的 :math:`\chi^2` 误差。

        .. math::

           \mathbf{e}_j^T \Omega_j \mathbf{e}_j


        Returns
        -------
        float
            该边的 :math:`\chi^2` 误差

        """
        err = self.calc_error()

        return np.dot(np.dot(np.transpose(err), self.information), err)

    def calc_chi2_gradient_hessian(self):
        r"""计算该边对图的 :math:`\chi^2` 误差、梯度（:math:`\mathbf{b}`）与 Hessian 矩阵（:math:`H`）的贡献。

        Returns
        -------
        float
            该边的 :math:`\chi^2` 误差
        dict
            该边对梯度的贡献
        dict
            该边对 Hessian 矩阵的贡献

        """
        chi2 = self.calc_chi2()

        err = self.calc_error()

        jacobians = self.calc_jacobians()

        return chi2, {v.index: np.dot(np.dot(np.transpose(err), self.information), jacobian) for v, jacobian in zip(self.vertices, jacobians)}, {(self.vertices[i].index, self.vertices[j].index): np.dot(np.dot(np.transpose(jacobians[i]), self.information), jacobians[j]) for i in range(len(jacobians)) for j in range(i, len(jacobians))}

    def calc_jacobians(self):
        r"""计算该边误差关于每个受约束位姿的雅可比矩阵。

        .. math::

           \frac{\partial}{\partial \Delta \mathbf{x}^k} \left[ \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k) \right]


        Returns
        -------
        list[np.ndarray]
            该边关于每个受约束位姿的雅可比矩阵

        """
        err = self.calc_error()

        # 紧凑位姿表示的维度
        dim = len(self.vertices[0].pose.to_compact())

        return [self._calc_jacobian(err, dim, i) for i in range(len(self.vertices))]

    def _calc_jacobian(self, err, dim, vertex_index):
        r"""计算该边关于指定顶点位姿的雅可比矩阵。

        Parameters
        ----------
        err : np.ndarray
            该边的当前误差（见 :meth:`EdgeOdometry.calc_error`）
        dim : int
            紧凑位姿表示的维度
        vertex_index : int
            正在计算雅可比矩阵的顶点（位姿）索引

        Returns
        -------
        np.ndarray
            该边关于指定顶点位姿的雅可比矩阵

        """
        jacobian = np.zeros(err.shape + (dim,))
        p0 = self.vertices[vertex_index].pose.copy()

        for d in range(dim):
            # 更新位姿
            delta_pose = np.zeros(dim)
            delta_pose[d] = EPSILON
            self.vertices[vertex_index].pose += delta_pose

            # 计算数值导数
            jacobian[:, d] = (self.calc_error() - err) / EPSILON

            # 恢复位姿
            self.vertices[vertex_index].pose = p0.copy()

        return jacobian

    def to_g2o(self):
        """将边导出为 .g2o 格式。

        Returns
        -------
        str
            .g2o 格式的边

        """
        return "EDGE_SE2 {} {} {} {} {} ".format(self.vertex_ids[0], self.vertex_ids[1], self.estimate[0], self.estimate[1], self.estimate[2]) + " ".join([str(x) for x in self.information[np.triu_indices(3, 0)]]) + "\n"

    def plot(self, color='b'):
        """绘制该边。

        Parameters
        ----------
        color : str
            绘制该边所用的颜色

        """
        xy = np.array([v.pose.position for v in self.vertices])
        plt.plot(xy[:, 0], xy[:, 1], color=color)
