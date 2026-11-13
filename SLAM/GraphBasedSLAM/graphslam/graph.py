# 版权所有 (c) 2020 Jeff Irion 及贡献者
#
# 本文件源自 `graphslam` 包：
#
#   https://github.com/JeffLIrion/python-graphslam

r"""存储基于图的同步定位与建图（Graph SLAM）所需边与顶点的 ``Graph`` 类。

"""

import warnings
from collections import defaultdict
from functools import reduce

import matplotlib.pyplot as plt
import numpy as np
from scipy.sparse import SparseEfficiencyWarning, lil_matrix
from scipy.sparse.linalg import spsolve

warnings.simplefilter("ignore", SparseEfficiencyWarning)
warnings.filterwarnings("ignore", category=SparseEfficiencyWarning)


# pylint: disable=too-few-public-methods
class _Chi2GradientHessian:
    r"""用于汇总 :math:`\chi^2` 误差、梯度与 Hessian 矩阵的类。

    Parameters
    ----------
    dim : int
        位姿的紧凑维度

    Attributes
    ----------
    chi2 : float
        :math:`\chi^2` 误差
    dim : int
        位姿的紧凑维度
    gradient : defaultdict
        对梯度向量的贡献
    hessian : defaultdict
        对 Hessian 矩阵的贡献

    """

    def __init__(self, dim):
        self.chi2 = 0.
        self.dim = dim
        self.gradient = defaultdict(lambda: np.zeros(dim))
        self.hessian = defaultdict(lambda: np.zeros((dim, dim)))

    @staticmethod
    def update(chi2_grad_hess, incoming):
        r"""更新 :math:`\chi^2` 误差以及梯度与 Hessian 字典。

        Parameters
        ----------
        chi2_grad_hess : _Chi2GradientHessian
            将被更新的 ``_Chi2GradientHessian``
        incoming : tuple

        """
        chi2_grad_hess.chi2 += incoming[0]

        for idx, contrib in incoming[1].items():
            chi2_grad_hess.gradient[idx] += contrib

        for (idx1, idx2), contrib in incoming[2].items():
            if idx1 <= idx2:
                chi2_grad_hess.hessian[idx1, idx2] += contrib
            else:
                chi2_grad_hess.hessian[idx2, idx1] += np.transpose(contrib)

        return chi2_grad_hess


class Graph(object):
    r"""将通过基于图的同步定位与建图（Graph SLAM）进行优化的图。

    Parameters
    ----------
    edges : list[graphslam.edge.edge_odometry.EdgeOdometry]
        图中边的列表
    vertices : list[graphslam.vertex.Vertex]
        图中顶点的列表

    Attributes
    ----------
    _chi2 : float, None
        当前的 :math:`\chi^2` 误差；若尚未计算则为 ``None``
    _edges : list[graphslam.edge.edge_odometry.EdgeOdometry]
        图中边（即约束）的列表
    _gradient : numpy.ndarray, None
        :math:`\chi^2` 误差的梯度 :math:`\mathbf{b}`；若尚未计算则为 ``None``
    _hessian : scipy.sparse.lil_matrix, None
        Hessian 矩阵 :math:`H`；若尚未计算则为 ``None``
    _vertices : list[graphslam.vertex.Vertex]
        图中顶点的列表

    """

    def __init__(self, edges, vertices):
        # 顶点列表与边列表
        self._edges = edges
        self._vertices = vertices

        # chi^2 误差、梯度与 Hessian 矩阵
        self._chi2 = None
        self._gradient = None
        self._hessian = None

        self._link_edges()

    def _link_edges(self):
        """填充图中各边的 ``vertices`` 属性。

        """
        index_id_dict = {i: v.id for i, v in enumerate(self._vertices)}
        id_index_dict = {v_id: v_index for v_index, v_id in
                         index_id_dict.items()}

        # 填充各顶点的 `index` 属性
        for v in self._vertices:
            v.index = id_index_dict[v.id]

        for e in self._edges:
            e.vertices = [self._vertices[id_index_dict[v_id]] for v_id in
                          e.vertex_ids]

    def calc_chi2(self):
        r"""计算 ``Graph`` 的 :math:`\chi^2` 误差。

        Returns
        -------
        float
            :math:`\chi^2` 误差

        """
        self._chi2 = sum((e.calc_chi2() for e in self._edges))
        return self._chi2

    def _calc_chi2_gradient_hessian(self):
        r"""计算 :math:`\chi^2` 误差、梯度 :math:`\mathbf{b}` 与 Hessian 矩阵 :math:`H`。

        """
        n = len(self._vertices)
        dim = len(self._vertices[0].pose.to_compact())
        chi2_gradient_hessian = reduce(_Chi2GradientHessian.update,
                                       (e.calc_chi2_gradient_hessian()
                                        for e in self._edges),
                                       _Chi2GradientHessian(dim))

        self._chi2 = chi2_gradient_hessian.chi2

        # 填充梯度向量
        self._gradient = np.zeros(n * dim, dtype=float)
        for idx, cont in chi2_gradient_hessian.gradient.items():
            self._gradient[idx * dim: (idx + 1) * dim] += cont

        # 填充 Hessian 矩阵
        self._hessian = lil_matrix((n * dim, n * dim), dtype=float)
        for (row_idx, col_idx), cont in chi2_gradient_hessian.hessian.items():
            x_start = row_idx * dim
            x_end = (row_idx + 1) * dim
            y_start = col_idx * dim
            y_end = (col_idx + 1) * dim
            self._hessian[x_start:x_end, y_start:y_end] = cont

            if row_idx != col_idx:
                x_start = col_idx * dim
                x_end = (col_idx + 1) * dim
                y_start = row_idx * dim
                y_end = (row_idx + 1) * dim
                self._hessian[x_start:x_end, y_start:y_end] = \
                    np.transpose(cont)

    def optimize(self, tol=1e-4, max_iter=20, fix_first_pose=True):
        r"""优化 ``Graph`` 的 :math:`\chi^2` 误差。

        Parameters
        ----------
        tol : float
            若迭代间 :math:`\chi^2` 误差的相对下降小于 ``tol``，则停止
        max_iter : int
            最大迭代次数
        fix_first_pose : bool
            若为 ``True``，则固定第一个位姿

        """
        n = len(self._vertices)
        dim = len(self._vertices[0].pose.to_compact())

        # 上一次迭代的 chi^2 误差
        chi2_prev = -1.

        # 用于显示优化过程
        print("\nIteration                chi^2        rel. change")
        print("---------                -----        -----------")

        for i in range(max_iter):
            self._calc_chi2_gradient_hessian()

            # 检查（上一次迭代的）收敛性；这样可避免把 chi^2 计算两次
            if i > 0:
                rel_diff = (chi2_prev - self._chi2) / (
                            chi2_prev + np.finfo(float).eps)
                print(
                    "{:9d} {:20.4f} {:18.6f}".format(i, self._chi2, -rel_diff))
                if self._chi2 < chi2_prev and rel_diff < tol:
                    return
            else:
                print("{:9d} {:20.4f}".format(i, self._chi2))

            # 更新上一次迭代的 chi^2 误差
            chi2_prev = self._chi2

            # 固定第一个位姿
            if fix_first_pose:
                self._hessian[:dim, :] = 0.
                self._hessian[:, :dim] = 0.
                self._hessian[:dim, :dim] += np.eye(dim)
                self._gradient[:dim] = 0.

            # 求解更新量
            dx = spsolve(self._hessian, -self._gradient)

            # 应用更新量
            for v, dx_i in zip(self._vertices, np.split(dx, n)):
                v.pose += dx_i

        # 若已达到最大迭代次数，则打印最后一次迭代的结果
        self.calc_chi2()
        rel_diff = (chi2_prev - self._chi2) / (chi2_prev + np.finfo(float).eps)
        print("{:9d} {:20.4f} {:18.6f}".format(
            max_iter, self._chi2, -rel_diff))

    def to_g2o(self, outfile):
        """以 .g2o 格式保存图。

        Parameters
        ----------
        outfile : str
            图的保存路径

        """
        with open(outfile, 'w') as f:
            for v in self._vertices:
                f.write(v.to_g2o())

            for e in self._edges:
                f.write(e.to_g2o())

    def plot(self, vertex_color='r', vertex_marker='o', vertex_markersize=3,
             edge_color='b', title=None):
        """绘制该图。

        Parameters
        ----------
        vertex_color : str
            绘制顶点所用的颜色
        vertex_marker : str
            绘制顶点所用的标记
        vertex_markersize : int
            所绘制顶点的大小
        edge_color : str
            绘制边所用的颜色
        title : str, None
            图的标题

        """
        plt.figure()

        for e in self._edges:
            e.plot(edge_color)

        for v in self._vertices:
            v.plot(vertex_color, vertex_marker, vertex_markersize)

        if title:
            plt.title(title)

        plt.show()
