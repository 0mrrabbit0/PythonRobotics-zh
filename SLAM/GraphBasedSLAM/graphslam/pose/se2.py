# 版权所有 (c) 2020 Jeff Irion 及贡献者
#
# 本文件源自 `graphslam` 包：
#
#   https://github.com/JeffLIrion/python-graphslam

r""":math:`SE(2)` 中的位姿表示。

"""

import math
import numpy as np

from ..util import neg_pi_to_pi


class PoseSE2(np.ndarray):
    r""":math:`SE(2)` 中的位姿表示。

    Parameters
    ----------
    position : np.ndarray, list
        :math:`\mathbb{R}^2` 中的位置
    orientation : float
        位姿的角度（弧度）

    """

    def __new__(cls, position, orientation):
        obj = np.array([position[0], position[1], neg_pi_to_pi(orientation)],
                       dtype=float).view(cls)
        return obj

    # pylint: disable=arguments-differ
    def copy(self):
        """返回位姿的副本。

        Returns
        -------
        PoseSE2
            位姿的副本

        """
        return PoseSE2(self[:2], self[2])

    def to_array(self):
        """以 numpy 数组形式返回位姿。

        Returns
        -------
        np.ndarray
            作为 numpy 数组的位姿

        """
        return np.array(self)

    def to_compact(self):
        """以紧凑 numpy 数组形式返回位姿。

        Returns
        -------
        np.ndarray
            作为紧凑 numpy 数组的位姿

        """
        return np.array(self)

    def to_matrix(self):
        """以 :math:`SE(2)` 矩阵形式返回位姿。

        Returns
        -------
        np.ndarray
            作为 :math:`SE(2)` 矩阵的位姿

        """
        return np.array([[np.cos(self[2]), -np.sin(self[2]), self[0]],
                         [np.sin(self[2]), np.cos(self[2]), self[1]],
                         [0., 0., 1.]], dtype=float)

    @classmethod
    def from_matrix(cls, matrix):
        """由 :math:`SE(2)` 矩阵得到位姿。

        Parameters
        ----------
        matrix : np.ndarray
            将转换为 `PoseSE2` 实例的 :math:`SE(2)` 矩阵

        Returns
        -------
        PoseSE2
            作为 `PoseSE2` 对象的矩阵

        """
        return cls([matrix[0, 2], matrix[1, 2]],
                   math.atan2(matrix[1, 0], matrix[0, 0]))

    # ======================================================================= #
    #                                                                         #
    #                                  属性                                   #
    #                                                                         #
    # ======================================================================= #
    @property
    def position(self):
        """返回位姿的位置。

        Returns
        -------
        np.ndarray
            位姿中的位置部分

        """
        return np.array(self[:2])

    @property
    def orientation(self):
        """返回位姿的朝向。

        Returns
        -------
        float
            位姿的角度

        """
        return self[2]

    @property
    def inverse(self):
        """返回位姿的逆。

        Returns
        -------
        PoseSE2
            位姿的逆

        """
        return PoseSE2([-self[0] * np.cos(self[2]) - self[1] * np.sin(self[2]),
                        self[0] * np.sin(self[2]) - self[1] * np.cos(self[2])],
                        -self[2])

    # ======================================================================= #
    #                                                                         #
    #                                魔术方法                                 #
    #                                                                         #
    # ======================================================================= #
    def __add__(self, other):
        r"""位姿相加（即位姿复合）：:math:`p_1 \oplus p_2`。

        Parameters
        ----------
        other : PoseSE2
            另一个位姿

        Returns
        -------
        PoseSE2
            位姿复合的结果

        """
        return PoseSE2(
            [self[0] + other[0] * np.cos(self[2]) - other[1] * np.sin(self[2]),
             self[1] + other[0] * np.sin(self[2]) + other[1] * np.cos(self[2])
             ], neg_pi_to_pi(self[2] + other[2]))

    def __sub__(self, other):
        r"""位姿相减（即逆位姿复合）：:math:`p_1 \ominus p_2`。

        Parameters
        ----------
        other : PoseSE2
            另一个位姿

        Returns
        -------
        PoseSE2
            逆位姿复合的结果

        """
        return PoseSE2([(self[0] - other[0]) * np.cos(other[2]) + (
                    self[1] - other[1]) * np.sin(other[2]),
                        (other[0] - self[0]) * np.sin(other[2]) + (
                                    self[1] - other[1]) * np.cos(other[2])],
                       neg_pi_to_pi(self[2] - other[2]))
