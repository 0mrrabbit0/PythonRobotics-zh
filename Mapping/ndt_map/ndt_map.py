"""
正态分布变换（NDT）栅格建图示例
"""
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

from Mapping.grid_map_lib.grid_map_lib import GridMap
from utils.plot import plot_covariance_ellipse


class NDTMap:
    """
    正态分布变换（NDT）地图类

    :param ox: 障碍物 x 位置列表
    :param oy: 障碍物 y 位置列表
    :param resolution: 栅格分辨率 [m]
    """

    class NDTGrid:
        """
        正态分布变换（NDT）栅格
        """

        def __init__(self):
            #: NDTGrid 栅格中的点数
            self.n_points = 0
            #: NDTGrid 单元中点的 x 均值位置
            self.mean_x = None
            #: NDTGrid 单元中点的 y 均值位置
            self.mean_y = None
            #: 正态分布变换（NDT）栅格的中心 x 位置
            self.center_grid_x = None
            #: 正态分布变换（NDT）栅格的中心 y 位置
            self.center_grid_y = None
            #: 正态分布变换（NDT）栅格的协方差矩阵
            self.covariance = None
            #: 正态分布变换（NDT）栅格的特征向量
            self.eig_vec = None
            #: 正态分布变换（NDT）栅格的特征值
            self.eig_values = None

    def __init__(self, ox, oy, resolution):
        #: 正态分布变换（NDT）栅格中的最少点数
        self.min_n_points = 3
        #: 正态分布变换（NDT）栅格的分辨率 [m]
        self.resolution = resolution
        width = int((max(ox) - min(ox))/resolution) + 3  # 向上取整并加上左右边距
        height = int((max(oy) - min(oy))/resolution) + 3
        center_x = np.mean(ox)
        center_y = np.mean(oy)
        self.ox = ox
        self.oy = oy
        #: 正态分布变换（NDT）栅格索引图
        self.grid_index_map = self._create_grid_index_map(ox, oy)

        #: 正态分布变换（NDT）栅格地图。每个栅格包含一个 NDTGrid 对象
        self._construct_grid_map(center_x, center_y, height, ox, oy, resolution, width)

    def _construct_grid_map(self, center_x, center_y, height, ox, oy, resolution, width):
        self.grid_map = GridMap(width, height, resolution, center_x, center_y, self.NDTGrid())
        for grid_index, inds in self.grid_index_map.items():
            ndt = self.NDTGrid()
            ndt.n_points = len(inds)
            if ndt.n_points >= self.min_n_points:
                ndt.mean_x = np.mean(ox[inds])
                ndt.mean_y = np.mean(oy[inds])
                ndt.center_grid_x, ndt.center_grid_y = \
                    self.grid_map.calc_grid_central_xy_position_from_grid_index(grid_index)
                ndt.covariance = np.cov(ox[inds], oy[inds])
                ndt.eig_values, ndt.eig_vec = np.linalg.eig(ndt.covariance)
                self.grid_map.data[grid_index] = ndt

    def _create_grid_index_map(self, ox, oy):
        grid_index_map = defaultdict(list)
        for i in range(len(ox)):
            grid_index = self.grid_map.calc_grid_index_from_xy_pos(ox[i], oy[i])
            grid_index_map[grid_index].append(i)
        return grid_index_map


def create_dummy_observation_data():
    ox = []
    oy = []
    # 左侧走廊
    for y in range(-50, 50):
        ox.append(-20.0)
        oy.append(y)
    # 右侧走廊 1
    for y in range(-50, 0):
        ox.append(20.0)
        oy.append(y)
    # 右侧走廊 2
    for x in range(20, 50):
        ox.append(x)
        oy.append(0)
    # 右侧走廊 3
    for x in range(20, 50):
        ox.append(x)
        oy.append(x/2.0+10)
    # 右侧走廊 4
    for y in range(20, 50):
        ox.append(20)
        oy.append(y)
    ox = np.array(ox)
    oy = np.array(oy)
    # 添加随机噪声
    ox += np.random.rand(len(ox)) * 1.0
    oy += np.random.rand(len(ox)) * 1.0
    return ox, oy


def main():
    print(__file__ + " start!!")

    ox, oy = create_dummy_observation_data()
    grid_resolution = 10.0
    ndt_map = NDTMap(ox, oy, grid_resolution)

    # 绘制原始观测
    plt.plot(ox, oy, ".r")

    # 绘制栅格聚类
    [plt.plot(ox[inds], oy[inds], "x") for inds in ndt_map.grid_index_map.values()]

    # 绘制正态分布变换（NDT）栅格地图
    [plot_covariance_ellipse(ndt.mean_x, ndt.mean_y, ndt.covariance, color="-k") for ndt in ndt_map.grid_map.data if ndt.n_points > 0]

    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()
