# 版权所有 (c) 2020 Jeff Irion 及贡献者
#
# 本文件源自 `graphslam` 包：
#
#   https://github.com/JeffLIrion/python-graphslam

"""本包中使用的工具函数。

"""

import numpy as np


TWO_PI = 2 * np.pi


def neg_pi_to_pi(angle):
    r"""将 ``angle`` 归一化到 :math:`[-\pi, \pi)`。

    Parameters
    ----------
    angle : float
        角度（弧度）

    Returns
    -------
    float
        归一化到 :math:`[-\pi, \pi)` 的角度

    """
    return (angle + np.pi) % (TWO_PI) - np.pi


def solve_for_edge_dimensionality(n):
    r"""求解一条边的维度。

    在 .g2o 文件中，一条边表示为 ``<estimate> <information matrix>``，其中只给出信息矩阵的上三角部分。

    本函数求解如下问题：

    .. math::

       d + \frac{d (d + 1)}{2} = n

    Returns
    -------
    int
        边的维度

    """
    return int(round(np.sqrt(2 * n + 2.25) - 1.5))


def upper_triangular_matrix_to_full_matrix(arr, n):
    """给定上三角矩阵，返回完整矩阵。

    Parameters
    ----------
    arr : np.ndarray
        矩阵的上三角部分
    n : int
        矩阵的阶数

    Returns
    -------
    mat : np.ndarray
        完整矩阵

    """
    triu0 = np.triu_indices(n, 0)
    tril1 = np.tril_indices(n, -1)

    mat = np.zeros((n, n), dtype=float)
    mat[triu0] = arr
    mat[tril1] = mat.T[tril1]

    return mat
