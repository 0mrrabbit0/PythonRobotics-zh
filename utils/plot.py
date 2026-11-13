"""
基于 Matplotlib 的绘图工具
"""
import math
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import art3d
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d.proj3d import proj_transform
from mpl_toolkits.mplot3d import Axes3D

from utils.angle import rot_mat_2d


def plot_covariance_ellipse(x, y, cov, chi2=3.0, color="-r", ax=None):
    """
    绘制表示协方差矩阵的椭圆。椭圆中心位于 (x, y)，其形状、大小与旋转由协方差矩阵决定。

    Parameters:
    x : (float) 椭圆中心的 x 坐标。
    y : (float) 椭圆中心的 y 坐标。
    cov : (numpy.ndarray) 2x2 协方差矩阵，决定椭圆的形状、大小与旋转。
    chi2 : (float, optional) 缩放椭圆大小的标量。该值通常按卡方分布分位数选取，以对应一定的置信水平（例如 3.0 约对应二维高斯的 95% 置信度）。默认值为 3.0。
    color : (str, optional) 椭圆的颜色与线型，遵循 matplotlib 约定。默认值为 "-r"（红色实线）。
    ax : (matplotlib.axes.Axes, optional) 用于绘制椭圆的 Axes 对象。若为 None（默认），则新建图形与坐标轴。

    Returns:
    None。本函数在指定坐标轴上绘制协方差椭圆。
    """
    eig_val, eig_vec = np.linalg.eig(cov)

    if eig_val[0] >= eig_val[1]:
        big_ind = 0
        small_ind = 1
    else:
        big_ind = 1
        small_ind = 0
    a = math.sqrt(chi2 * eig_val[big_ind])
    b = math.sqrt(chi2 * eig_val[small_ind])
    angle = math.atan2(eig_vec[1, big_ind], eig_vec[0, big_ind])
    plot_ellipse(x, y, a, b, angle, color=color, ax=ax)


def plot_ellipse(x, y, a, b, angle, color="-r", ax=None, **kwargs):
    """
    根据给定参数绘制椭圆。

    Parameters
    ----------
    x : (float) 椭圆中心的 x 坐标。
    y : (float) 椭圆中心的 y 坐标。
    a : (float) 椭圆半长轴的长度。
    b : (float) 椭圆半短轴的长度。
    angle : (float) 椭圆的旋转角，单位为弧度。
    color : (str, optional) 椭圆的颜色与线型，遵循 matplotlib 约定。默认值为 "-r"（红色实线）。
    ax : (matplotlib.axes.Axes, optional) 用于绘制椭圆的 Axes 对象。若为 None（默认），则新建图形与坐标轴。
    **kwargs: 传递给 plt.plot 或 ax.plot 的其他关键字参数。

    Returns
    ---------
    None。本函数根据指定参数绘制椭圆。
    """

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    px = [a * math.cos(it) for it in t]
    py = [b * math.sin(it) for it in t]
    fx = rot_mat_2d(angle) @ (np.array([px, py]))
    px = np.array(fx[0, :] + x).flatten()
    py = np.array(fx[1, :] + y).flatten()
    if ax is None:
        plt.plot(px, py, color, **kwargs)
    else:
        ax.plot(px, py, color, **kwargs)


def plot_arrow(x, y, yaw, arrow_length=1.0,
               origin_point_plot_style="xr",
               head_width=0.1, fc="r", ec="k", **kwargs):
    """
    根据二维状态 (x, y, yaw) 绘制一个或多个箭头

    可以使用 matplotlib.pyplot.arrow 的全部可选设置。
    - matplotlib.pyplot.arrow:
    https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.arrow.html

    Parameters
    ----------
    x : float 或 array_like
        箭头起点的 x 位置，或位置列表。
    y : float 或 array_like
        箭头起点的 y 位置，或位置列表。
    yaw : float 或 array_like
        箭头的航向角（朝向），或角度列表。
    arrow_length : float（可选）
        箭头长度。默认值为 1.0
    origin_point_plot_style : str（可选）
        起点的绘图样式。若为 None，则不绘制起点。
    head_width : float（可选）
        箭头头部宽度。默认值为 0.1
    fc : string（可选）
        填充颜色
    ec : string（可选）
        边缘颜色
    """
    if not isinstance(x, float):
        for (i_x, i_y, i_yaw) in zip(x, y, yaw):
            plot_arrow(i_x, i_y, i_yaw, head_width=head_width,
                       fc=fc, ec=ec, **kwargs)
    else:
        plt.arrow(x, y,
                  arrow_length * math.cos(yaw),
                  arrow_length * math.sin(yaw),
                  head_width=head_width,
                  fc=fc, ec=ec,
                  **kwargs)
        if origin_point_plot_style is not None:
            plt.plot(x, y, origin_point_plot_style)


def plot_curvature(x_list, y_list, heading_list, curvature,
                   k=0.01, c="-c", label="Curvature"):
    """
    在二维路径上绘制曲率。该图是从原始路径引出的一条线，
    相对原始路径的横向距离表示曲率大小。
    左转显示在路径右侧，右转显示在路径左侧。
    直线路径上曲率为 0，因此曲率图与路径重合。

    Parameters
    ----------
    x_list : array_like
        路径的 x 位置列表
    y_list : array_like
        路径的 y 位置列表
    heading_list : array_like
        路径的航向列表
    curvature : array_like
        路径的曲率列表
    k : float
        曲率比例因子，用于计算相对原始路径的距离
    c : string
        绘图颜色
    label : string
        绘图标签
    """
    cx = [x + d * k * np.cos(yaw - np.pi / 2.0) for x, y, yaw, d in
          zip(x_list, y_list, heading_list, curvature)]
    cy = [y + d * k * np.sin(yaw - np.pi / 2.0) for x, y, yaw, d in
          zip(x_list, y_list, heading_list, curvature)]

    plt.plot(cx, cy, c, label=label)
    for ix, iy, icx, icy in zip(x_list, y_list, cx, cy):
        plt.plot([ix, icx], [iy, icy], c)


class Arrow3D(FancyArrowPatch):

    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)

    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)

    def do_3d_projection(self, renderer=None):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))

        return np.min(zs)


def _arrow3D(ax, x, y, z, dx, dy, dz, *args, **kwargs):
    '''向 `Axes3D` 实例添加三维箭头。'''
    arrow = Arrow3D(x, y, z, dx, dy, dz, *args, **kwargs)
    ax.add_artist(arrow)


def plot_3d_vector_arrow(ax, p1, p2):
    setattr(Axes3D, 'arrow3D', _arrow3D)
    ax.arrow3D(p1[0], p1[1], p1[2],
               p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2],
               mutation_scale=20,
               arrowstyle="-|>",
               )


def plot_triangle(p1, p2, p3, ax):
    ax.add_collection3d(art3d.Poly3DCollection([[p1, p2, p3]], color='b'))


def set_equal_3d_axis(ax, x_lims, y_lims, z_lims):
    """将坐标轴设为等比例的辅助函数

    Args:
        ax (Axes3DSubplot): matplotlib 三维坐标轴，由
        `ax = fig.add_subplot(projection='3d')` 创建
        x_lims (np.array): 包含 x 最小值与最大值的数组
        y_lims (np.array): 包含 y 最小值与最大值的数组
        z_lims (np.array): 包含 z 最小值与最大值的数组
    """
    x_lims = np.asarray(x_lims)
    y_lims = np.asarray(y_lims)
    z_lims = np.asarray(z_lims)
    # 计算所需的最大范围
    max_range = np.array([x_lims.max() - x_lims.min(),
                          y_lims.max() - y_lims.min(),
                          z_lims.max() - z_lims.min()]).max() / 2.0
    # 计算各轴中点
    mid_x = (x_lims.max() + x_lims.min()) * 0.5
    mid_y = (y_lims.max() + y_lims.min()) * 0.5
    mid_z = (z_lims.max() + z_lims.min()) * 0.5

    # 设置坐标轴范围
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)


if __name__ == '__main__':
    plot_ellipse(0, 0, 1, 2, np.deg2rad(15))
    plt.axis('equal')
    plt.show()

