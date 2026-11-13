# 版权所有 (c) 2020 Jeff Irion 及贡献者
#
# 本文件源自 `graphslam` 包：
#
#   https://github.com/JeffLIrion/python-graphslam

"""``Vertex`` 类。

"""

import matplotlib.pyplot as plt


# pylint: disable=too-few-public-methods
class Vertex:
    """表示基于图的同步定位与建图（Graph SLAM）中一个顶点的类。

    Parameters
    ----------
    vertex_id : int
        顶点的唯一 ID
    pose : graphslam.pose.se2.PoseSE2
        与该顶点关联的位姿
    vertex_index : int, None
        该顶点在图的 ``vertices`` 列表中的索引

    Attributes
    ----------
    id : int
        顶点的唯一 ID
    index : int, None
        该顶点在图的 ``vertices`` 列表中的索引
    pose : graphslam.pose.se2.PoseSE2
        与该顶点关联的位姿

    """
    def __init__(self, vertex_id, pose, vertex_index=None):
        self.id = vertex_id
        self.pose = pose
        self.index = vertex_index

    def to_g2o(self):
        """将顶点导出为 .g2o 格式。

        Returns
        -------
        str
            .g2o 格式的顶点

        """
        return "VERTEX_SE2 {} {} {} {}\n".format(self.id, self.pose[0], self.pose[1], self.pose[2])

    def plot(self, color='r', marker='o', markersize=3):
        """绘制顶点。

        Parameters
        ----------
        color : str
            绘制顶点所用的颜色
        marker : str
            绘制顶点所用的标记
        markersize : int
            所绘制顶点的大小

        """
        x, y = self.pose.position
        plt.plot(x, y, color=color, marker=marker, markersize=markersize)
