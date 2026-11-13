"""

B 样条路径规划器

作者: Atsushi Sakai (@Atsushi_twi)

"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as interpolate

from utils.plot import plot_curvature


def approximate_b_spline_path(x: list,
                              y: list,
                              n_path_points: int,
                              degree: int = 3,
                              s=None,
                              ) -> tuple:
    """
    用 B 样条路径逼近点列

    Parameters
    ----------
    x : array_like
        被逼近点的 x 位置列表
    y : array_like
        被逼近点的 y 位置列表
    n_path_points : int
        路径点数量
    degree : int, 可选
        B 样条曲线次数。必须满足 2<= k <= 5。默认值：3。
    s : int, 可选
        平滑参数。该值越大，路径越平滑，但精度越低。该值越小，
        路径越精确，但越不平滑。
        当 `s` 为 0 时，等价于插值。默认值为 None，
        此时 `s` 取 `len(x)`。

    Returns
    -------
    x : array
        结果路径的 x 坐标
    y : array
        结果路径的 y 坐标
    heading : array
        结果路径的航向
    curvature : array
        结果路径的曲率

    """
    distances = _calc_distance_vector(x, y)

    spl_i_x = interpolate.UnivariateSpline(distances, x, k=degree, s=s)
    spl_i_y = interpolate.UnivariateSpline(distances, y, k=degree, s=s)

    sampled = np.linspace(0.0, distances[-1], n_path_points)
    return _evaluate_spline(sampled, spl_i_x, spl_i_y)


def interpolate_b_spline_path(x, y,
                              n_path_points: int,
                              degree: int = 3) -> tuple:
    """
    用 B 样条路径插值 x-y 点列

    Parameters
    ----------
    x : array_like
        插值点的 x 坐标
    y : array_like
        插值点的 y 坐标
    n_path_points : int
        路径点数量
    degree : int, 可选
        B 样条次数。必须满足 2<= k <= 5。默认值：3

    Returns
    -------
    x : array
        结果路径的 x 坐标
    y : array
        结果路径的 y 坐标
    heading : array
        结果路径的航向
    curvature : array
        结果路径的曲率

    """
    return approximate_b_spline_path(x, y, n_path_points, degree, s=0.0)


def _calc_distance_vector(x, y):
    dx, dy = np.diff(x), np.diff(y)
    distances = np.cumsum([np.hypot(idx, idy) for idx, idy in zip(dx, dy)])
    distances = np.concatenate(([0.0], distances))
    distances /= distances[-1]
    return distances


def _evaluate_spline(sampled, spl_i_x, spl_i_y):
    x = spl_i_x(sampled)
    y = spl_i_y(sampled)
    dx = spl_i_x.derivative(1)(sampled)
    dy = spl_i_y.derivative(1)(sampled)
    heading = np.arctan2(dy, dx)
    ddx = spl_i_x.derivative(2)(sampled)
    ddy = spl_i_y.derivative(2)(sampled)
    curvature = (ddy * dx - ddx * dy) / np.power(dx * dx + dy * dy, 2.0 / 3.0)
    return np.array(x), y, heading, curvature,


def main():
    print(__file__ + " start!!")
    # 路径点
    way_point_x = [-1.0, 3.0, 4.0, 2.0, 1.0]
    way_point_y = [0.0, -3.0, 1.0, 1.0, 3.0]
    n_course_point = 50  # 采样点数

    plt.subplots()
    rax, ray, heading, curvature = approximate_b_spline_path(
        way_point_x, way_point_y, n_course_point, s=0.5)
    plt.plot(rax, ray, '-r', label="Approximated B-Spline path")
    plot_curvature(rax, ray, heading, curvature)

    plt.title("B-Spline approximation")
    plt.plot(way_point_x, way_point_y, '-og', label="way points")
    plt.grid(True)
    plt.legend()
    plt.axis("equal")

    plt.subplots()
    rix, riy, heading, curvature = interpolate_b_spline_path(
        way_point_x, way_point_y, n_course_point)
    plt.plot(rix, riy, '-b', label="Interpolated B-Spline path")
    plot_curvature(rix, riy, heading, curvature)

    plt.title("B-Spline interpolation")
    plt.plot(way_point_x, way_point_y, '-og', label="way points")
    plt.grid(True)
    plt.legend()
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()
