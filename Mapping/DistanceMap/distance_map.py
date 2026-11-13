"""
距离图

作者：Wang Zheng (@Aglargil)

Reference:

- [距离图]
(https://cs.brown.edu/people/pfelzens/papers/dt-final.pdf)
"""

import numpy as np
import matplotlib.pyplot as plt
import scipy

INF = 1e20
ENABLE_PLOT = True


def compute_sdf_scipy(obstacles):
    """
    使用 scipy 由布尔场计算符号距离场（SDF）。
    本函数与 compute_sdf 功能相同。
    但借助 scipy.ndimage.distance_transform_edt，计算速度快得多。

    示例：500×500 地图
    • compute_sdf：3 秒
    • compute_sdf_scipy：0.05 秒

    Parameters
    ----------
    obstacles : array_like
        二维布尔数组，其中 '1' 表示障碍物，'0' 表示自由空间。

    Returns
    -------
    array_like
        表示符号距离场的二维数组，正值表示到最近障碍物的距离，
        负值表示到最近自由空间的距离。
    """
    # distance_transform_edt 将 '0' 视为障碍物，因此需要把障碍物转换为 '0'
    a = scipy.ndimage.distance_transform_edt(obstacles == 0)
    b = scipy.ndimage.distance_transform_edt(obstacles == 1)
    return a - b


def compute_udf_scipy(obstacles):
    """
    使用 scipy 由布尔场计算无符号距离场（UDF）。
    本函数与 compute_udf 功能相同。
    但借助 scipy.ndimage.distance_transform_edt，计算速度快得多。

    示例：500×500 地图
    • compute_udf：1.5 秒
    • compute_udf_scipy：0.02 秒

    Parameters
    ----------
    obstacles : array_like
        二维布尔数组，其中 '1' 表示障碍物，'0' 表示自由空间。

    Returns
    -------
    array_like
        到最近障碍物的距离的二维数组，维度与 `bool_field` 相同。
    """
    return scipy.ndimage.distance_transform_edt(obstacles == 0)


def compute_sdf(obstacles):
    """
    由布尔场计算符号距离场（SDF）。

    Parameters
    ----------
    obstacles : array_like
        二维布尔数组，其中 '1' 表示障碍物，'0' 表示自由空间。

    Returns
    -------
    array_like
        表示符号距离场的二维数组，正值表示到最近障碍物的距离，
        负值表示到最近自由空间的距离。
    """
    a = compute_udf(obstacles)
    b = compute_udf(obstacles == 0)
    return a - b


def compute_udf(obstacles):
    """
    由布尔场计算无符号距离场（UDF）。

    Parameters
    ----------
    obstacles : array_like
        二维布尔数组，其中 '1' 表示障碍物，'0' 表示自由空间。

    Returns
    -------
    array_like
        到最近障碍物的距离的二维数组，维度与 `bool_field` 相同。
    """
    edt = obstacles.copy()
    if not np.all(np.isin(edt, [0, 1])):
        raise ValueError("Input array should only contain 0 and 1")
    edt = np.where(edt == 0, INF, edt)
    edt = np.where(edt == 1, 0, edt)
    for row in range(len(edt)):
        dt(edt[row])
    edt = edt.T
    for row in range(len(edt)):
        dt(edt[row])
    edt = edt.T
    return np.sqrt(edt)


def dt(d):
    """
    在平方欧氏距离下计算一维距离变换

    Parameters
    ----------
    d : array_like
        包含距离的输入数组。

    Returns:
    --------
    d : array_like
        已计算距离的变换后数组。
    """
    v = np.zeros(len(d) + 1)
    z = np.zeros(len(d) + 1)
    k = 0
    v[0] = 0
    z[0] = -INF
    z[1] = INF
    for q in range(1, len(d)):
        s = ((d[q] + q * q) - (d[int(v[k])] + v[k] * v[k])) / (2 * q - 2 * v[k])
        while s <= z[k]:
            k = k - 1
            s = ((d[q] + q * q) - (d[int(v[k])] + v[k] * v[k])) / (2 * q - 2 * v[k])
        k = k + 1
        v[k] = q
        z[k] = s
        z[k + 1] = INF
    k = 0
    for q in range(len(d)):
        while z[k + 1] < q:
            k = k + 1
        dx = q - v[k]
        d[q] = dx * dx + d[int(v[k])]


def main():
    obstacles = np.array(
        [
            [1, 0, 0, 0, 0],
            [0, 1, 1, 1, 0],
            [0, 1, 1, 1, 0],
            [0, 0, 1, 1, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ]
    )

    # 计算符号距离场
    sdf = compute_sdf(obstacles)
    udf = compute_udf(obstacles)

    if ENABLE_PLOT:
        _, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))

        obstacles_plot = ax1.imshow(obstacles, cmap="binary")
        ax1.set_title("Obstacles")
        ax1.set_xlabel("x")
        ax1.set_ylabel("y")
        plt.colorbar(obstacles_plot, ax=ax1)

        udf_plot = ax2.imshow(udf, cmap="viridis")
        ax2.set_title("Unsigned Distance Field")
        ax2.set_xlabel("x")
        ax2.set_ylabel("y")
        plt.colorbar(udf_plot, ax=ax2)

        sdf_plot = ax3.imshow(sdf, cmap="RdBu")
        ax3.set_title("Signed Distance Field")
        ax3.set_xlabel("x")
        ax3.set_ylabel("y")
        plt.colorbar(sdf_plot, ax=ax3)

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
