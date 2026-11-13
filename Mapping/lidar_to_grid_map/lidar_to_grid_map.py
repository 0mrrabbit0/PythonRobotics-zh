"""

激光雷达转二维栅格地图示例

作者：Erno Horvath、Csaba Hajdu，基于 Atsushi Sakai 的脚本

"""

import math
from collections import deque

import matplotlib.pyplot as plt
import numpy as np

EXTEND_AREA = 1.0


def file_read(f):
    """
    读取激光雷达光束（角度及对应的距离数据）
    """
    with open(f) as data:
        measures = [line.split(",") for line in data]
    angles = []
    distances = []
    for measure in measures:
        angles.append(float(measure[0]))
        distances.append(float(measure[1]))
    angles = np.array(angles)
    distances = np.array(distances)
    return angles, distances


def bresenham(start, end):
    """
    Bresenham 直线绘制算法的实现
    参见 en.wikipedia.org/wiki/Bresenham's_line_algorithm
    Bresenham 直线算法
    由起点和终点生成 np.array（原始实现来自 roguebasin.com）
    >>> points1 = bresenham((4, 4), (6, 10))
    >>> print(points1)
    np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
    """
    # 设置初始条件
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx)  # 判断直线的陡峭程度
    if is_steep:  # 旋转直线
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    # 必要时交换起点与终点，并记录交换状态
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    dx = x2 - x1  # 重新计算差分
    dy = y2 - y1  # 重新计算差分
    error = int(dx / 2.0)  # 计算误差
    y_step = 1 if y1 < y2 else -1
    # 在包围盒上迭代，生成起点与终点之间的点
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = [y, x] if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += y_step
            error += dx
    if swapped:  # 若坐标被交换过，则反转列表
        points.reverse()
    points = np.array(points)
    return points


def calc_grid_map_config(ox, oy, xy_resolution):
    """
    根据测量中心计算尺寸与最大距离
    """
    min_x = round(min(ox) - EXTEND_AREA / 2.0)
    min_y = round(min(oy) - EXTEND_AREA / 2.0)
    max_x = round(max(ox) + EXTEND_AREA / 2.0)
    max_y = round(max(oy) + EXTEND_AREA / 2.0)
    xw = int(round((max_x - min_x) / xy_resolution))
    yw = int(round((max_y - min_y) / xy_resolution))
    print("The grid map is ", xw, "x", yw, ".")
    return min_x, min_y, max_x, max_y, xw, yw


def atan_zero_to_twopi(y, x):
    angle = math.atan2(y, x)
    if angle < 0.0:
        angle += math.pi * 2.0
    return angle


def init_flood_fill(center_point, obstacle_points, xy_points, min_coord,
                    xy_resolution):
    """
    center_point: 中心点
    obstacle_points: 检测到的障碍物点 (x,y)
    xy_points: (x,y) 点对
    """
    center_x, center_y = center_point
    prev_ix, prev_iy = center_x - 1, center_y
    ox, oy = obstacle_points
    xw, yw = xy_points
    min_x, min_y = min_coord
    occupancy_map = (np.ones((xw, yw))) * 0.5
    for (x, y) in zip(ox, oy):
        # 占据区域的 x 坐标
        ix = int(round((x - min_x) / xy_resolution))
        # 占据区域的 y 坐标
        iy = int(round((y - min_y) / xy_resolution))
        free_area = bresenham((prev_ix, prev_iy), (ix, iy))
        for fa in free_area:
            occupancy_map[fa[0]][fa[1]] = 0  # 自由区域 0.0
        prev_ix = ix
        prev_iy = iy
    return occupancy_map


def flood_fill(center_point, occupancy_map):
    """
    center_point: 填充的起始点 (x,y)
    occupancy_map: 由 Bresenham 射线追踪生成的占据地图
    """
    # 用队列方法填充空白区域
    sx, sy = occupancy_map.shape
    fringe = deque()
    fringe.appendleft(center_point)
    while fringe:
        n = fringe.pop()
        nx, ny = n
        # 西
        if nx > 0:
            if occupancy_map[nx - 1, ny] == 0.5:
                occupancy_map[nx - 1, ny] = 0.0
                fringe.appendleft((nx - 1, ny))
        # 东
        if nx < sx - 1:
            if occupancy_map[nx + 1, ny] == 0.5:
                occupancy_map[nx + 1, ny] = 0.0
                fringe.appendleft((nx + 1, ny))
        # 北
        if ny > 0:
            if occupancy_map[nx, ny - 1] == 0.5:
                occupancy_map[nx, ny - 1] = 0.0
                fringe.appendleft((nx, ny - 1))
        # 南
        if ny < sy - 1:
            if occupancy_map[nx, ny + 1] == 0.5:
                occupancy_map[nx, ny + 1] = 0.0
                fringe.appendleft((nx, ny + 1))


def generate_ray_casting_grid_map(ox, oy, xy_resolution, breshen=True):
    """
    布尔量 breshen 表示采用 Bresenham 射线投射计算（True）
    还是采用泛洪填充计算（False）
    """
    min_x, min_y, max_x, max_y, x_w, y_w = calc_grid_map_config(
        ox, oy, xy_resolution)
    # 默认 0.5 -- [[0.5 for i in range(y_w)] for i in range(x_w)]
    occupancy_map = np.ones((x_w, y_w)) / 2
    center_x = int(
        round(-min_x / xy_resolution))  # 栅格地图的中心 x 坐标
    center_y = int(
        round(-min_y / xy_resolution))  # 栅格地图的中心 y 坐标
    # 用 Bresenham 射线投射计算的占据栅格
    if breshen:
        for (x, y) in zip(ox, oy):
            # 占据区域的 x 坐标
            ix = int(round((x - min_x) / xy_resolution))
            # 占据区域的 y 坐标
            iy = int(round((y - min_y) / xy_resolution))
            laser_beams = bresenham((center_x, center_y), (
                ix, iy))  # 从激光雷达到占据点的线段
            for laser_beam in laser_beams:
                occupancy_map[laser_beam[0]][
                    laser_beam[1]] = 0.0  # 自由区域 0.0
            occupancy_map[ix][iy] = 1.0  # 占据区域 1.0
            occupancy_map[ix + 1][iy] = 1.0  # 扩展占据区域
            occupancy_map[ix][iy + 1] = 1.0  # 扩展占据区域
            occupancy_map[ix + 1][iy + 1] = 1.0  # 扩展占据区域
    # 用泛洪填充计算的占据栅格
    else:
        occupancy_map = init_flood_fill((center_x, center_y), (ox, oy),
                                        (x_w, y_w),
                                        (min_x, min_y), xy_resolution)
        flood_fill((center_x, center_y), occupancy_map)
        occupancy_map = np.array(occupancy_map, dtype=float)
        for (x, y) in zip(ox, oy):
            ix = int(round((x - min_x) / xy_resolution))
            iy = int(round((y - min_y) / xy_resolution))
            occupancy_map[ix][iy] = 1.0  # 占据区域 1.0
            occupancy_map[ix + 1][iy] = 1.0  # 扩展占据区域
            occupancy_map[ix][iy + 1] = 1.0  # 扩展占据区域
            occupancy_map[ix + 1][iy + 1] = 1.0  # 扩展占据区域
    return occupancy_map, min_x, max_x, min_y, max_y, xy_resolution


def main():
    """
    使用示例
    """
    print(__file__, "start")
    xy_resolution = 0.02  # x-y 栅格分辨率
    ang, dist = file_read("lidar01.csv")
    ox = np.sin(ang) * dist
    oy = np.cos(ang) * dist
    occupancy_map, min_x, max_x, min_y, max_y, xy_resolution = \
        generate_ray_casting_grid_map(ox, oy, xy_resolution, True)
    xy_res = np.array(occupancy_map).shape
    plt.figure(1, figsize=(10, 4))
    plt.subplot(122)
    plt.imshow(occupancy_map, cmap="PiYG_r")
    # cmap = "binary" "PiYG_r" "PiYG_r" "bone" "bone_r" "RdYlGn_r"
    plt.clim(-0.4, 1.4)
    plt.gca().set_xticks(np.arange(-.5, xy_res[1], 1), minor=True)
    plt.gca().set_yticks(np.arange(-.5, xy_res[0], 1), minor=True)
    plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
    plt.colorbar()
    plt.subplot(121)
    plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-")
    plt.axis("equal")
    plt.plot(0.0, 0.0, "ob")
    plt.gca().set_aspect("equal", "box")
    bottom, top = plt.ylim()  # 返回当前 y 轴范围
    plt.ylim((top, bottom))  # 重新缩放 y 轴，以匹配栅格朝向
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()
