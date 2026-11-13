"""

使用 k-均值算法的目标聚类

作者：Atsushi Sakai (@Atsushi_twi)

"""

import math
import matplotlib.pyplot as plt
import random

# k-均值参数
MAX_LOOP = 10
DCOST_TH = 0.1
show_animation = True


def kmeans_clustering(rx, ry, nc):
    """
    对给定数据集执行 k-均值聚类，迭代调整聚类质心，
    直到收敛到给定阈值以内或达到最大迭代次数。

    实现会初始化聚类、计算初始质心，并通过迭代更新细化聚类，
    以基于数据点与质心之间最小距离的代价函数进行优化。

    参数:
        rx: List[float]
            待聚类数据点的 x 坐标。
        ry: List[float]
            待聚类数据点的 y 坐标。
        nc: int
            将数据划分成的聚类数量。

    Returns:
        Clusters
            包含收敛后最终聚类分配与质心的实例。

    Raises:
        None

    """
    clusters = Clusters(rx, ry, nc)
    clusters.calc_centroid()

    pre_cost = float("inf")
    for loop in range(MAX_LOOP):
        cost = clusters.update_clusters()
        clusters.calc_centroid()

        d_cost = abs(cost - pre_cost)
        if d_cost < DCOST_TH:
            break
        pre_cost = cost

    return clusters


class Clusters:

    def __init__(self, x, y, n_label):
        self.x = x
        self.y = y
        self.n_data = len(self.x)
        self.n_label = n_label
        self.labels = [random.randint(0, n_label - 1)
                       for _ in range(self.n_data)]
        self.center_x = [0.0 for _ in range(n_label)]
        self.center_y = [0.0 for _ in range(n_label)]

    def plot_cluster(self):
        for label in set(self.labels):
            x, y = self._get_labeled_x_y(label)
            plt.plot(x, y, ".")

    def calc_centroid(self):
        for label in set(self.labels):
            x, y = self._get_labeled_x_y(label)
            n_data = len(x)
            self.center_x[label] = sum(x) / n_data
            self.center_y[label] = sum(y) / n_data

    def update_clusters(self):
        cost = 0.0

        for ip in range(self.n_data):
            px = self.x[ip]
            py = self.y[ip]

            dx = [icx - px for icx in self.center_x]
            dy = [icy - py for icy in self.center_y]

            dist_list = [math.hypot(idx, idy) for (idx, idy) in zip(dx, dy)]
            min_dist = min(dist_list)
            min_id = dist_list.index(min_dist)
            self.labels[ip] = min_id
            cost += min_dist

        return cost

    def _get_labeled_x_y(self, target_label):
        x = [self.x[i] for i, label in enumerate(self.labels) if label == target_label]
        y = [self.y[i] for i, label in enumerate(self.labels) if label == target_label]
        return x, y


def calc_raw_data(cx, cy, n_points, rand_d):
    rx, ry = [], []

    for (icx, icy) in zip(cx, cy):
        for _ in range(n_points):
            rx.append(icx + rand_d * (random.random() - 0.5))
            ry.append(icy + rand_d * (random.random() - 0.5))

    return rx, ry


def update_positions(cx, cy):
    # 目标运动参数
    DX1 = 0.4
    DY1 = 0.5
    DX2 = -0.3
    DY2 = -0.5

    cx[0] += DX1
    cy[0] += DY1
    cx[1] += DX2
    cy[1] += DY2

    return cx, cy


def main():
    print(__file__ + " start!!")

    cx = [0.0, 8.0]
    cy = [0.0, 8.0]
    n_points = 10
    rand_d = 3.0
    n_cluster = 2
    sim_time = 15.0
    dt = 1.0
    time = 0.0

    while time <= sim_time:
        print("Time:", time)
        time += dt

        # 目标运动仿真
        cx, cy = update_positions(cx, cy)
        raw_x, raw_y = calc_raw_data(cx, cy, n_points, rand_d)

        clusters = kmeans_clustering(raw_x, raw_y, n_cluster)

        # 用于动画
        if show_animation:  # pragma: no cover
            plt.cla()
            # 按 esc 键停止仿真。
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            clusters.plot_cluster()
            plt.plot(cx, cy, "or")
            plt.xlim(-2.0, 10.0)
            plt.ylim(-2.0, 10.0)
            plt.pause(dt)

    print("Done")


if __name__ == '__main__':
    main()
