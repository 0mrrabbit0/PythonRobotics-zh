"""
点云采样示例代码。本代码支持
- 体素点采样
- 最远点采样
- 泊松盘采样

"""
import matplotlib.pyplot as plt
import numpy as np
import numpy.typing as npt
from collections import defaultdict

do_plot = True


def voxel_point_sampling(original_points: npt.NDArray, voxel_size: float):
    """
    体素点采样函数。
    本函数用体素栅格对 N 维点进行采样。
    同一体素栅格中的点通过均值运算合并以完成采样。

    Parameters
    ----------
    original_points :  (M, N) 用于采样的 N 维点。
                        点数为 M。
    voxel_size : 体素栅格尺寸

    Returns
    -------
    采样点 (M', N)
    """
    voxel_dict = defaultdict(list)
    for i in range(original_points.shape[0]):
        xyz = original_points[i, :]
        xyz_index = tuple(xyz // voxel_size)
        voxel_dict[xyz_index].append(xyz)
    points = np.vstack([np.mean(v, axis=0) for v in voxel_dict.values()])
    return points


def farthest_point_sampling(orig_points: npt.NDArray,
                            n_points: int, seed: int):
    """
    最远点采样函数
    本函数按最远点策略对 N 维点进行采样。

    Parameters
    ----------
    orig_points :  (M, N) 用于采样的 N 维点。
                    点数为 M。
    n_points : 采样点数
    seed : 随机种子

    Returns
    -------
    采样点 (n_points, N)

    """
    rng = np.random.default_rng(seed)
    n_orig_points = orig_points.shape[0]
    first_point_id = rng.choice(range(n_orig_points))
    min_distances = np.ones(n_orig_points) * float("inf")
    selected_ids = [first_point_id]
    while len(selected_ids) < n_points:
        base_point = orig_points[selected_ids[-1], :]
        distances = np.linalg.norm(orig_points[np.newaxis, :] - base_point,
                                   axis=2).flatten()
        min_distances = np.minimum(min_distances, distances)
        distances_rank = np.argsort(-min_distances)  # 由远到近的顺序
        for i in distances_rank:  # 从最远点开始
            if i not in selected_ids:  # 若尚未选中，则选中它
                selected_ids.append(i)
                break
    return orig_points[selected_ids, :]


def poisson_disk_sampling(orig_points: npt.NDArray, n_points: int,
                          min_distance: float, seed: int, MAX_ITER=1000):
    """
    泊松盘采样函数
    本函数随机采样 N 维点，直到点数满足要求，
    并保持已选点之间的最小距离。

    Parameters
    ----------
    orig_points :  (M, N) 用于采样的 N 维点。
                    点数为 M。
    n_points : 采样点数
    min_distance : 已选点之间的最小距离。
    seed : 随机种子
    MAX_ITER : 最大迭代次数。默认值为 1000。

    Returns
    -------
    采样点（n_points 或更少, N）
    """
    rng = np.random.default_rng(seed)
    selected_id = rng.choice(range(orig_points.shape[0]))
    selected_ids = [selected_id]
    loop = 0
    while len(selected_ids) < n_points and loop <= MAX_ITER:
        selected_id = rng.choice(range(orig_points.shape[0]))
        base_point = orig_points[selected_id, :]
        distances = np.linalg.norm(
            orig_points[np.newaxis, selected_ids] - base_point,
            axis=2).flatten()
        if min(distances) >= min_distance:
            selected_ids.append(selected_id)
        loop += 1
    if len(selected_ids) != n_points:
        print("Could not find the specified number of points...")

    return orig_points[selected_ids, :]


def plot_sampled_points(original_points, sampled_points, method_name):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(original_points[:, 0], original_points[:, 1],
               original_points[:, 2], marker=".", label="Original points")
    ax.scatter(sampled_points[:, 0], sampled_points[:, 1],
               sampled_points[:, 2], marker="o",
               label="Filtered points")
    plt.legend()
    plt.title(method_name)
    plt.axis("equal")


def main():
    n_points = 1000
    seed = 1234
    rng = np.random.default_rng(seed)

    x = rng.normal(0.0, 10.0, n_points)
    y = rng.normal(0.0, 1.0, n_points)
    z = rng.normal(0.0, 10.0, n_points)
    original_points = np.vstack((x, y, z)).T
    print(f"{original_points.shape=}")
    print("Voxel point sampling")
    voxel_size = 20.0
    voxel_sampling_points = voxel_point_sampling(original_points, voxel_size)
    print(f"{voxel_sampling_points.shape=}")

    print("Farthest point sampling")
    n_points = 20
    farthest_sampling_points = farthest_point_sampling(original_points,
                                                       n_points, seed)
    print(f"{farthest_sampling_points.shape=}")

    print("Poisson disk sampling")
    n_points = 20
    min_distance = 10.0
    poisson_disk_points = poisson_disk_sampling(original_points, n_points,
                                                min_distance, seed)
    print(f"{poisson_disk_points.shape=}")

    if do_plot:
        plot_sampled_points(original_points, voxel_sampling_points,
                            "Voxel point sampling")
        plot_sampled_points(original_points, farthest_sampling_points,
                            "Farthest point sampling")
        plot_sampled_points(original_points, poisson_disk_points,
                            "poisson disk sampling")
        plt.show()


if __name__ == '__main__':
    main()
