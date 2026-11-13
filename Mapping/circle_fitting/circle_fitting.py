"""

使用圆拟合进行目标形状识别

作者：Atsushi Sakai (@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import math
import random
import numpy as np

show_animation = True


def circle_fitting(x, y):
    """
    用最小二乘法将圆拟合到给定点集。

    本函数计算二维平面上最拟合给定点集的圆的圆心 (x, y) 与半径。
    它最小化圆与所给点之间的平方误差，并返回计算出的圆心坐标、半径与拟合误差。

    Raises
    ------
    ValueError
        若输入列表 x 与 y 的元素个数不同。

    Parameters
    ----------
    x : list[float]
        点的 x 坐标。
    y : list[float]
        点的 y 坐标。

    Returns
    -------
    tuple[float, float, float, float]
        包含以下内容的元组：
        - 拟合圆圆心的 x 坐标 (float)。
        - 拟合圆圆心的 y 坐标 (float)。
        - 拟合圆的半径 (float)。
        - 作为偏差度量的拟合误差 (float)。
    """

    sumx = sum(x)
    sumy = sum(y)
    sumx2 = sum([ix ** 2 for ix in x])
    sumy2 = sum([iy ** 2 for iy in y])
    sumxy = sum([ix * iy for (ix, iy) in zip(x, y)])

    F = np.array([[sumx2, sumxy, sumx],
                  [sumxy, sumy2, sumy],
                  [sumx, sumy, len(x)]])

    G = np.array([[-sum([ix ** 3 + ix * iy ** 2 for (ix, iy) in zip(x, y)])],
                  [-sum([ix ** 2 * iy + iy ** 3 for (ix, iy) in zip(x, y)])],
                  [-sum([ix ** 2 + iy ** 2 for (ix, iy) in zip(x, y)])]])

    T = np.linalg.inv(F).dot(G)

    cxe = float(T[0, 0] / -2)
    cye = float(T[1, 0] / -2)
    re = math.sqrt(cxe**2 + cye**2 - T[2, 0])

    error = sum([np.hypot(cxe - ix, cye - iy) - re for (ix, iy) in zip(x, y)])

    return (cxe, cye, re, error)


def get_sample_points(cx, cy, cr, angle_reso):
    x, y, angle, r = [], [], [], []

    # 点采样
    for theta in np.arange(0.0, 2.0 * math.pi, angle_reso):
        nx = cx + cr * math.cos(theta)
        ny = cy + cr * math.sin(theta)
        nangle = math.atan2(ny, nx)
        nr = math.hypot(nx, ny) * random.uniform(0.95, 1.05)

        x.append(nx)
        y.append(ny)
        angle.append(nangle)
        r.append(nr)

    # 射线投射滤波
    rx, ry = ray_casting_filter(x, y, angle, r, angle_reso)

    return rx, ry


def ray_casting_filter(xl, yl, thetal, rangel, angle_reso):
    rx, ry = [], []
    rangedb = [float("inf") for _ in range(
        int(math.floor((math.pi * 2.0) / angle_reso)) + 1)]

    for i, _ in enumerate(thetal):
        angleid = math.floor(thetal[i] / angle_reso)

        if rangedb[angleid] > rangel[i]:
            rangedb[angleid] = rangel[i]

    for i, _ in enumerate(rangedb):
        t = i * angle_reso
        if rangedb[i] != float("inf"):
            rx.append(rangedb[i] * math.cos(t))
            ry.append(rangedb[i] * math.sin(t))

    return rx, ry


def plot_circle(x, y, size, color="-b"):  # pragma: no cover
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
    yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
    plt.plot(xl, yl, color)


def main():

    # 仿真参数
    simtime = 15.0  # 仿真时间
    dt = 1.0  # 时间步长

    cx = -2.0  # 障碍物初始 x 位置
    cy = -8.0  # 障碍物初始 y 位置
    cr = 1.0  # 障碍物半径
    theta = np.deg2rad(30.0)  # 障碍物运动方向
    angle_reso = np.deg2rad(3.0)  # 传感器角度分辨率

    time = 0.0
    while time <= simtime:
        time += dt

        cx += math.cos(theta)
        cy += math.cos(theta)

        x, y = get_sample_points(cx, cy, cr, angle_reso)

        ex, ey, er, error = circle_fitting(x, y)
        print("Error:", error)

        if show_animation:  # pragma: no cover
            plt.cla()
            # 按 esc 键停止仿真。
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.axis("equal")
            plt.plot(0.0, 0.0, "*r")
            plot_circle(cx, cy, cr)
            plt.plot(x, y, "xr")
            plot_circle(ex, ey, er, "-r")
            plt.pause(dt)

    print("Done")


if __name__ == '__main__':
    main()
