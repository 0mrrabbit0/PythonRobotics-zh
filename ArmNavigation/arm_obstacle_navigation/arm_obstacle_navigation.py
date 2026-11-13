"""
在环形网格上用 A* 进行避障导航

作者: Daniel Ingram (daniel-s-ingram)
"""
from math import pi
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import from_levels_and_colors

plt.ion()

# 仿真参数
M = 100
obstacles = [[1.75, 0.75, 0.6], [0.55, 1.5, 0.5], [0, -1, 0.25]]


def main():
    arm = NLinkArm([1, 1], [0, 0])
    start = (10, 50)
    goal = (58, 56)
    grid = get_occupancy_grid(arm, obstacles)
    plt.imshow(grid)
    plt.show()
    route = astar_torus(grid, start, goal)
    for node in route:
        theta1 = 2 * pi * node[0] / M - pi
        theta2 = 2 * pi * node[1] / M - pi
        arm.update_joints([theta1, theta2])
        arm.plot(obstacles=obstacles)


def detect_collision(line_seg, circle):
    """
    判断线段（机械臂连杆）是否与圆（障碍物）接触。
    出处: https://web.archive.org/web/20200130224918/http://doswa.com/2009/07/13/circle-segment-intersectioncollision.html
    Args:
        line_seg: 线段端点坐标列表，例如 [[1, 1], [2, 2]]
        circle: 圆心坐标与半径列表，例如 [0, 0, 0.5] 表示圆心在
                原点、半径为 0.5 的圆

    Returns:
        若线段与圆接触则为 True
        否则为 False
    """
    a_vec = np.array([line_seg[0][0], line_seg[0][1]])
    b_vec = np.array([line_seg[1][0], line_seg[1][1]])
    c_vec = np.array([circle[0], circle[1]])
    radius = circle[2]
    line_vec = b_vec - a_vec
    line_mag = np.linalg.norm(line_vec)
    circle_vec = c_vec - a_vec
    proj = circle_vec.dot(line_vec / line_mag)
    if proj <= 0:
        closest_point = a_vec
    elif proj >= line_mag:
        closest_point = b_vec
    else:
        closest_point = a_vec + line_vec * proj / line_mag
    if np.linalg.norm(closest_point - c_vec) > radius:
        return False

    return True


def get_occupancy_grid(arm, obstacles):
    """
    将关节空间在 -pi 到 +pi 之间离散为 M 个取值，
    并判断关节空间中给定坐标是否会使机械臂
    与环境中的障碍物发生碰撞。

    Args:
        arm: NLinkArm 的实例
        obstacles: 障碍物列表，每个障碍物由
                   xy 坐标与半径组成的列表定义。

    Returns:
        关节空间中的占据栅格
    """
    grid = [[0 for _ in range(M)] for _ in range(M)]
    theta_list = [2 * i * pi / M for i in range(-M // 2, M // 2 + 1)]
    for i in range(M):
        for j in range(M):
            arm.update_joints([theta_list[i], theta_list[j]])
            points = arm.points
            collision_detected = False
            for k in range(len(points) - 1):
                for obstacle in obstacles:
                    line_seg = [points[k], points[k + 1]]
                    collision_detected = detect_collision(line_seg, obstacle)
                    if collision_detected:
                        break
                if collision_detected:
                    break
            grid[i][j] = int(collision_detected)
    return np.array(grid)


def astar_torus(grid, start_node, goal_node):
    """
    在环形网格上用 A* 算法，寻找初始关节构型与目标关节构型之间的路径。

    Args:
        grid: 占据栅格（ndarray）
        start_node: 初始关节构型（tuple）
        goal_node: 目标关节构型（tuple）

    Returns:
        关节空间中从 start_node 到 goal_node 的无障碍路径
    """
    colors = ['white', 'black', 'red', 'pink', 'yellow', 'green', 'orange']
    levels = [0, 1, 2, 3, 4, 5, 6, 7]
    cmap, norm = from_levels_and_colors(levels, colors)

    grid[start_node] = 4
    grid[goal_node] = 5

    parent_map = [[() for _ in range(M)] for _ in range(M)]

    heuristic_map = calc_heuristic_map(M, goal_node)

    explored_heuristic_map = np.full((M, M), np.inf)
    distance_map = np.full((M, M), np.inf)
    explored_heuristic_map[start_node] = heuristic_map[start_node]
    distance_map[start_node] = 0
    while True:
        grid[start_node] = 4
        grid[goal_node] = 5

        current_node = np.unravel_index(
            np.argmin(explored_heuristic_map, axis=None), explored_heuristic_map.shape)
        min_distance = np.min(explored_heuristic_map)
        if (current_node == goal_node) or np.isinf(min_distance):
            break

        grid[current_node] = 2
        explored_heuristic_map[current_node] = np.inf

        i, j = current_node[0], current_node[1]

        neighbors = find_neighbors(i, j)

        for neighbor in neighbors:
            if grid[neighbor] == 0 or grid[neighbor] == 5:
                distance_map[neighbor] = distance_map[current_node] + 1
                explored_heuristic_map[neighbor] = heuristic_map[neighbor]
                parent_map[neighbor[0]][neighbor[1]] = current_node
                grid[neighbor] = 3

    if np.isinf(explored_heuristic_map[goal_node]):
        route = []
        print("No route found.")
    else:
        route = [goal_node]
        while parent_map[route[0][0]][route[0][1]] != ():
            route.insert(0, parent_map[route[0][0]][route[0][1]])

        print(f"The route found covers {len(route)} grid cells.")
        for i in range(1, len(route)):
            grid[route[i]] = 6
            plt.cla()
            # 按 Esc 键停止仿真。
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.imshow(grid, cmap=cmap, norm=norm, interpolation=None)
            plt.show()
            plt.pause(1e-2)

    return route


def find_neighbors(i, j):
    neighbors = []
    if i - 1 >= 0:
        neighbors.append((i - 1, j))
    else:
        neighbors.append((M - 1, j))

    if i + 1 < M:
        neighbors.append((i + 1, j))
    else:
        neighbors.append((0, j))

    if j - 1 >= 0:
        neighbors.append((i, j - 1))
    else:
        neighbors.append((i, M - 1))

    if j + 1 < M:
        neighbors.append((i, j + 1))
    else:
        neighbors.append((i, 0))

    return neighbors


def calc_heuristic_map(M, goal_node):
    X, Y = np.meshgrid([i for i in range(M)], [i for i in range(M)])
    heuristic_map = np.abs(X - goal_node[1]) + np.abs(Y - goal_node[0])
    for i in range(heuristic_map.shape[0]):
        for j in range(heuristic_map.shape[1]):
            heuristic_map[i, j] = min(heuristic_map[i, j],
                                      M - i - 1 + heuristic_map[M - 1, j],
                                      i + heuristic_map[0, j],
                                      M - j - 1 + heuristic_map[i, M - 1],
                                      j + heuristic_map[i, 0]
                                      )

    return heuristic_map


class NLinkArm:
    """
    用于控制并绘制任意连杆数平面机械臂的类。
    """

    def __init__(self, link_lengths, joint_angles):
        self.n_links = len(link_lengths)
        if self.n_links != len(joint_angles):
            raise ValueError()

        self.link_lengths = np.array(link_lengths)
        self.joint_angles = np.array(joint_angles)
        self.points = [[0, 0] for _ in range(self.n_links + 1)]

        self.lim = sum(link_lengths)
        self.update_points()

    def update_joints(self, joint_angles):
        self.joint_angles = joint_angles
        self.update_points()

    def update_points(self):
        for i in range(1, self.n_links + 1):
            self.points[i][0] = self.points[i - 1][0] + \
                self.link_lengths[i - 1] * \
                np.cos(np.sum(self.joint_angles[:i]))
            self.points[i][1] = self.points[i - 1][1] + \
                self.link_lengths[i - 1] * \
                np.sin(np.sum(self.joint_angles[:i]))

        self.end_effector = np.array(self.points[self.n_links]).T

    def plot(self, obstacles=[]):  # pragma: no cover
        plt.cla()

        for obstacle in obstacles:
            circle = plt.Circle(
                (obstacle[0], obstacle[1]), radius=0.5 * obstacle[2], fc='k')
            plt.gca().add_patch(circle)

        for i in range(self.n_links + 1):
            if i is not self.n_links:
                plt.plot([self.points[i][0], self.points[i + 1][0]],
                         [self.points[i][1], self.points[i + 1][1]], 'r-')
            plt.plot(self.points[i][0], self.points[i][1], 'k.')

        plt.xlim([-self.lim, self.lim])
        plt.ylim([-self.lim, self.lim])
        plt.draw()
        plt.pause(1e-5)


if __name__ == '__main__':
    main()
