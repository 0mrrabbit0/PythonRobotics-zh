"""
三次样条规划器

作者：Atsushi Sakai(@Atsushi_twi)

"""
import math
import numpy as np
import bisect


class CubicSpline1D:
    """
    一维三次样条类

    Parameters
    ----------
    x : list
        数据点的 x 坐标。这些 x 坐标必须
        按升序
        排列。
    y : list
        数据点的 y 坐标

    Examples
    --------
    可以对一维数据点进行插值。

    >>> import numpy as np
    >>> import matplotlib.pyplot as plt
    >>> x = np.arange(5)
    >>> y = [1.7, -6, 5, 6.5, 0.0]
    >>> sp = CubicSpline1D(x, y)
    >>> xi = np.linspace(0.0, 5.0)
    >>> yi = [sp.calc_position(x) for x in xi]
    >>> plt.plot(x, y, "xb", label="Data points")
    >>> plt.plot(xi, yi , "r", label="Cubic spline interpolation")
    >>> plt.grid(True)
    >>> plt.legend()
    >>> plt.show()

    .. image:: cubic_spline_1d.png

    """

    def __init__(self, x, y):

        h = np.diff(x)
        if np.any(h < 0):
            raise ValueError("x coordinates must be sorted in ascending order")

        self.a, self.b, self.c, self.d = [], [], [], []
        self.x = x
        self.y = y
        self.nx = len(x)  # x 的维数

        # 计算系数 a
        self.a = [iy for iy in y]

        # 计算系数 c
        A = self.__calc_A(h)
        B = self.__calc_B(h, self.a)
        self.c = np.linalg.solve(A, B)

        # 计算样条系数 b 和 d
        for i in range(self.nx - 1):
            d = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            b = 1.0 / h[i] * (self.a[i + 1] - self.a[i]) \
                - h[i] / 3.0 * (2.0 * self.c[i] + self.c[i + 1])
            self.d.append(d)
            self.b.append(b)

    def calc_position(self, x):
        """
        计算给定 `x` 处的 `y` 位置。

        若 `x` 超出数据点的 `x` 范围，则返回 None。

        Parameters
        ----------
        x : float
            用于计算 y 的 x 位置。

        Returns
        -------
        y : float
            给定 x 对应的 y 位置。
        """
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        position = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return position

    def calc_first_derivative(self, x):
        """
        计算给定 x 处的一阶导数。

        若 x 超出输入 x 的范围，则返回 None

        Parameters
        ----------
        x : float
            用于计算一阶导数的 x 位置。

        Returns
        -------
        dy : float
            给定 x 对应的一阶导数。
        """

        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        dy = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return dy

    def calc_second_derivative(self, x):
        """
        计算给定 x 处的二阶导数。

        若 x 超出输入 x 的范围，则返回 None

        Parameters
        ----------
        x : float
            用于计算二阶导数的 x 位置。

        Returns
        -------
        ddy : float
            给定 x 对应的二阶导数。
        """

        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        ddy = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return ddy

    def calc_third_derivative(self, x):
        """
        计算给定 x 处的三阶导数。

        若 x 超出输入 x 的范围，则返回 None

        Parameters
        ----------
        x : float
            用于计算三阶导数的 x 位置。

        Returns
        -------
        dddy : float
            给定 x 对应的三阶导数。
        """
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dddy = 6.0 * self.d[i]
        return dddy

    def __search_index(self, x):
        """
        搜索数据段索引
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        计算样条系数 c 所用的矩阵 A
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def __calc_B(self, h, a):
        """
        计算样条系数 c 所用的矩阵 B
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1]\
                - 3.0 * (a[i + 1] - a[i]) / h[i]
        return B


class CubicSpline2D:
    """
    二维三次样条 CubicSpline2D 类

    Parameters
    ----------
    x : list
        数据点的 x 坐标。
    y : list
        数据点的 y 坐标。

    Examples
    --------
    可以对二维数据点进行插值。

    >>> import matplotlib.pyplot as plt
    >>> x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    >>> y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]
    >>> ds = 0.1  # [m] distance of each interpolated points
    >>> sp = CubicSpline2D(x, y)
    >>> s = np.arange(0, sp.s[-1], ds)
    >>> rx, ry, ryaw, rk = [], [], [], []
    >>> for i_s in s:
    ...     ix, iy = sp.calc_position(i_s)
    ...     rx.append(ix)
    ...     ry.append(iy)
    ...     ryaw.append(sp.calc_yaw(i_s))
    ...     rk.append(sp.calc_curvature(i_s))
    >>> plt.subplots(1)
    >>> plt.plot(x, y, "xb", label="Data points")
    >>> plt.plot(rx, ry, "-r", label="Cubic spline path")
    >>> plt.grid(True)
    >>> plt.axis("equal")
    >>> plt.xlabel("x[m]")
    >>> plt.ylabel("y[m]")
    >>> plt.legend()
    >>> plt.show()

    .. image:: cubic_spline_2d_path.png

    >>> plt.subplots(1)
    >>> plt.plot(s, [np.rad2deg(iyaw) for iyaw in ryaw], "-r", label="yaw")
    >>> plt.grid(True)
    >>> plt.legend()
    >>> plt.xlabel("line length[m]")
    >>> plt.ylabel("yaw angle[deg]")

    .. image:: cubic_spline_2d_yaw.png

    >>> plt.subplots(1)
    >>> plt.plot(s, rk, "-r", label="curvature")
    >>> plt.grid(True)
    >>> plt.legend()
    >>> plt.xlabel("line length[m]")
    >>> plt.ylabel("curvature [1/m]")

    .. image:: cubic_spline_2d_curvature.png
    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        计算位置

        Parameters
        ----------
        s : float
            距起点的距离。若 `s` 超出数据点
            范围，则返回 None。

        Returns
        -------
        x : float
            给定 s 对应的 x 位置。
        y : float
            给定 s 对应的 y 位置。
        """
        x = self.sx.calc_position(s)
        y = self.sy.calc_position(s)

        return x, y

    def calc_curvature(self, s):
        """
        计算曲率

        Parameters
        ----------
        s : float
            距起点的距离。若 `s` 超出数据点
            范围，则返回 None。

        Returns
        -------
        k : float
            给定 s 对应的曲率。
        """
        dx = self.sx.calc_first_derivative(s)
        ddx = self.sx.calc_second_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        ddy = self.sy.calc_second_derivative(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k

    def calc_curvature_rate(self, s):
        """
        计算曲率变化率

        Parameters
        ----------
        s : float
            距起点的距离。若 `s` 超出数据点
            范围，则返回 None。

        Returns
        -------
        k : float
            给定 s 对应的曲率变化率。
        """
        dx = self.sx.calc_first_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        ddx = self.sx.calc_second_derivative(s)
        ddy = self.sy.calc_second_derivative(s)
        dddx = self.sx.calc_third_derivative(s)
        dddy = self.sy.calc_third_derivative(s)
        a = dx * ddy - dy * ddx
        b = dx * dddy - dy * dddx
        c = dx * ddx + dy * ddy
        d = dx * dx + dy * dy
        return (b * d - 3.0 * a * c) / (d * d * d)

    def calc_yaw(self, s):
        """
        计算航向角

        Parameters
        ----------
        s : float
            距起点的距离。若 `s` 超出数据点
            范围，则返回 None。

        Returns
        -------
        yaw : float
            给定 s 对应的航向角（切向量）。
        """
        dx = self.sx.calc_first_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        yaw = math.atan2(dy, dx)
        return yaw


def calc_spline_course(x, y, ds=0.1):
    sp = CubicSpline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s


def main_1d():
    print("CubicSpline1D test")
    import matplotlib.pyplot as plt
    x = np.arange(5)
    y = [1.7, -6, 5, 6.5, 0.0]
    sp = CubicSpline1D(x, y)
    xi = np.linspace(0.0, 5.0)

    plt.plot(x, y, "xb", label="Data points")
    plt.plot(xi, [sp.calc_position(x) for x in xi], "r",
             label="Cubic spline interpolation")
    plt.grid(True)
    plt.legend()
    plt.show()


def main_2d():  # pragma: no cover
    print("CubicSpline1D 2D test")
    import matplotlib.pyplot as plt
    x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]
    ds = 0.1  # [m] 各插值点之间的距离

    sp = CubicSpline2D(x, y)
    s = np.arange(0, sp.s[-1], ds)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    plt.subplots(1)
    plt.plot(x, y, "xb", label="Data points")
    plt.plot(rx, ry, "-r", label="Cubic spline path")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()

    plt.subplots(1)
    plt.plot(s, [np.rad2deg(iyaw) for iyaw in ryaw], "-r", label="yaw")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("yaw angle[deg]")

    plt.subplots(1)
    plt.plot(s, rk, "-r", label="curvature")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("curvature [1/m]")

    plt.show()


if __name__ == '__main__':
    # main_1d()
    main_2d()
