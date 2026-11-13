"""

使用贝塞尔曲线的路径规划。

作者: Atsushi Sakai(@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import numpy as np
import scipy.special

show_animation = True


def calc_4points_bezier_path(sx, sy, syaw, ex, ey, eyaw, offset):
    """
    根据起点和终点计算控制点与路径。

    :param sx: (float) 起点的 x 坐标
    :param sy: (float) 起点的 y 坐标
    :param syaw: (float) 起点航向角
    :param ex: (float) 终点的 x 坐标
    :param ey: (float) 终点的 y 坐标
    :param eyaw: (float) 终点航向角
    :param offset: (float)
    :return: (numpy array, numpy array)
    """
    dist = np.hypot(sx - ex, sy - ey) / offset
    control_points = np.array(
        [[sx, sy],
         [sx + dist * np.cos(syaw), sy + dist * np.sin(syaw)],
         [ex - dist * np.cos(eyaw), ey - dist * np.sin(eyaw)],
         [ex, ey]])

    path = calc_bezier_path(control_points, n_points=100)

    return path, control_points


def calc_bezier_path(control_points, n_points=100):
    """
    根据控制点计算贝塞尔曲线路径（轨迹）。

    :param control_points: (numpy array)
    :param n_points: (int) 轨迹中的点数
    :return: (numpy array)
    """
    traj = []
    for t in np.linspace(0, 1, n_points):
        traj.append(bezier(t, control_points))

    return np.array(traj)


def bernstein_poly(n, i, t):
    """
    Bernstein 多项式。

    :param n: (int) 多项式次数
    :param i: (int)
    :param t: (float)
    :return: (float)
    """
    return scipy.special.comb(n, i) * t ** i * (1 - t) ** (n - i)


def bezier(t, control_points):
    """
    返回贝塞尔曲线上的一个点。

    :param t: (float) [0, 1] 内的数值
    :param control_points: (numpy array)
    :return: (numpy array) 该点的坐标
    """
    n = len(control_points) - 1
    return np.sum([bernstein_poly(n, i, t) * control_points[i] for i in range(n + 1)], axis=0)


def bezier_derivatives_control_points(control_points, n_derivatives):
    """
    计算给定贝塞尔曲线各阶导数的控制点。

    贝塞尔曲线的导数仍是贝塞尔曲线。
    详细说明见 https://pomax.github.io/bezierinfo/#derivatives

    :param control_points: (numpy array)
    :param n_derivatives: (int)
    例如，n_derivatives=2 -> 计算一阶与二阶导数的控制点
    :return: ([numpy array])
    """
    w = {0: control_points}
    for i in range(n_derivatives):
        n = len(w[i])
        w[i + 1] = np.array([(n - 1) * (w[i][j + 1] - w[i][j])
                             for j in range(n - 1)])
    return w


def curvature(dx, dy, ddx, ddy):
    """
    由一阶与二阶导数计算某一点的曲率。

    :param dx: (float) 沿 x 轴的一阶导数
    :param dy: (float)
    :param ddx: (float) 沿 x 轴的二阶导数
    :param ddy: (float)
    :return: (float)
    """
    return (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** (3 / 2)


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):  # pragma: no cover
    """绘制箭头。"""
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def main():
    """绘制一条示例贝塞尔曲线。"""
    start_x = 10.0  # [m]
    start_y = 1.0  # [m]
    start_yaw = np.radians(180.0)  # [rad]

    end_x = -0.0  # [m]
    end_y = -3.0  # [m]
    end_yaw = np.radians(-45.0)  # [rad]
    offset = 3.0

    path, control_points = calc_4points_bezier_path(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset)

    # 注：也可以不指定起点和终点位置，
    # 而直接定义 n 个控制点并计算路径：
    # control_points = np.array([[5., 1.], [-2.78, 1.], [-11.5, -4.5], [-6., -8.]])
    # path = calc_bezier_path(control_points, n_points=100)

    # 显示给定点处的切线、法线与曲率半径
    t = 0.86  # [0, 1] 内的数值
    x_target, y_target = bezier(t, control_points)
    derivatives_cp = bezier_derivatives_control_points(control_points, 2)
    point = bezier(t, control_points)
    dt = bezier(t, derivatives_cp[1])
    ddt = bezier(t, derivatives_cp[2])
    # 曲率半径
    radius = 1 / curvature(dt[0], dt[1], ddt[0], ddt[1])
    # 归一化导数
    dt /= np.linalg.norm(dt, 2)
    tangent = np.array([point, point + dt])
    normal = np.array([point, point + [- dt[1], dt[0]]])
    curvature_center = point + np.array([- dt[1], dt[0]]) * radius
    circle = plt.Circle(tuple(curvature_center), radius,
                        color=(0, 0.8, 0.8), fill=False, linewidth=1)

    assert path.T[0][0] == start_x, "path is invalid"
    assert path.T[1][0] == start_y, "path is invalid"
    assert path.T[0][-1] == end_x, "path is invalid"
    assert path.T[1][-1] == end_y, "path is invalid"

    if show_animation:  # pragma: no cover
        fig, ax = plt.subplots()
        ax.plot(path.T[0], path.T[1], label="Bezier Path")
        ax.plot(control_points.T[0], control_points.T[1],
                '--o', label="Control Points")
        ax.plot(x_target, y_target)
        ax.plot(tangent[:, 0], tangent[:, 1], label="Tangent")
        ax.plot(normal[:, 0], normal[:, 1], label="Normal")
        ax.add_artist(circle)
        plot_arrow(start_x, start_y, start_yaw)
        plot_arrow(end_x, end_y, end_yaw)
        ax.legend()
        ax.axis("equal")
        ax.grid(True)
        plt.show()


def main2():
    """展示偏移量的影响。"""
    start_x = 10.0  # [m]
    start_y = 1.0  # [m]
    start_yaw = np.radians(180.0)  # [rad]

    end_x = -0.0  # [m]
    end_y = -3.0  # [m]
    end_yaw = np.radians(-45.0)  # [rad]

    for offset in np.arange(1.0, 5.0, 1.0):
        path, control_points = calc_4points_bezier_path(
            start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset)
        assert path.T[0][0] == start_x, "path is invalid"
        assert path.T[1][0] == start_y, "path is invalid"
        assert path.T[0][-1] == end_x, "path is invalid"
        assert path.T[1][-1] == end_y, "path is invalid"

        if show_animation:  # pragma: no cover
            plt.plot(path.T[0], path.T[1], label="Offset=" + str(offset))

    if show_animation:  # pragma: no cover
        plot_arrow(start_x, start_y, start_yaw)
        plot_arrow(end_x, end_y, end_yaw)
        plt.legend()
        plt.axis("equal")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    main()
    #  main2()
