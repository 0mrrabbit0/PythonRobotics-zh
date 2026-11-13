# 版权所有 (c) 2020 Jeff Irion 及贡献者
#
# 本文件源自 `graphslam` 包：
#
#   https://github.com/JeffLIrion/python-graphslam

"""用于加载图的函数。

"""

import logging

import numpy as np

from .edge.edge_odometry import EdgeOdometry
from .graph import Graph
from .pose.se2 import PoseSE2
from .util import upper_triangular_matrix_to_full_matrix
from .vertex import Vertex

_LOGGER = logging.getLogger(__name__)


def load_g2o_se2(infile):
    """从 .g2o 文件加载 :math:`SE(2)` 图。

    Parameters
    ----------
    infile : str
        .g2o 文件路径

    Returns
    -------
    Graph
        加载得到的图

    """
    edges = []
    vertices = []

    with open(infile) as f:
        for line in f.readlines():
            if line.startswith("VERTEX_SE2"):
                numbers = line[10:].split()
                arr = np.array([float(number) for number in numbers[1:]],
                               dtype=float)
                p = PoseSE2(arr[:2], arr[2])
                v = Vertex(int(numbers[0]), p)
                vertices.append(v)
                continue

            if line.startswith("EDGE_SE2"):
                numbers = line[9:].split()
                arr = np.array([float(number) for number in numbers[2:]],
                               dtype=float)
                vertex_ids = [int(numbers[0]), int(numbers[1])]
                estimate = PoseSE2(arr[:2], arr[2])
                information = upper_triangular_matrix_to_full_matrix(arr[3:], 3)
                e = EdgeOdometry(vertex_ids, information, estimate)
                edges.append(e)
                continue

            if line.strip():
                _LOGGER.warning("Line not supported -- '%s'", line.rstrip())

    return Graph(edges, vertices)
