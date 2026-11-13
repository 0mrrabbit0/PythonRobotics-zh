"""

带路径平滑的 RRT 路径规划示例代码

作者: AtsushiSakai(@Atsushi_twi)

"""

import math
import random
import matplotlib.pyplot as plt
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent))

from rrt import RRT

show_animation = True


def get_path_length(path):
    le = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.hypot(dx, dy)
        le += d

    return le


def get_target_point(path, targetL):
    le = 0
    ti = 0
    lastPairLen = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.hypot(dx, dy)
        le += d
        if le >= targetL:
            ti = i - 1
            lastPairLen = d
            break

    partRatio = (le - targetL) / lastPairLen

    x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
    y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio

    return [x, y, ti]


def is_point_collision(x, y, obstacle_list, robot_radius):
    """
    检查单个点是否与任一障碍物发生碰撞。

    本函数计算给定点 (x, y) 与每个障碍物中心之间的欧氏距离。若该距离小于或等于
    障碍物半径与机器人半径之和，则判定发生碰撞。

    Args:
        x (float): 待检查点的 X 坐标。
        y (float): 待检查点的 Y 坐标。
        obstacle_list (List[Tuple[float, float, float]]): 障碍物列表，每个元素为 (ox, oy, radius)。
        robot_radius (float): 机器人半径，用于膨胀障碍物。

    Returns:
        bool: 点与任一障碍物发生碰撞时为 True，否则为 False。
    """
    for (ox, oy, obstacle_radius) in obstacle_list:
        d = math.hypot(ox - x, oy - y)
        if d <= obstacle_radius + robot_radius:
            return True  # 发生碰撞
    return False


def line_collision_check(first, second, obstacle_list, robot_radius=0.0, sample_step=0.2):
    """
    检查 `first` 与 `second` 之间的线段是否与任一障碍物发生碰撞。
    通过膨胀障碍物尺寸来计入 robot_radius。

    Args:
        first (List[float]): 线段起点 [x, y]
        second (List[float]): 线段终点 [x, y]
        obstacle_list (List[Tuple[float, float, float]]): 障碍物，格式为 (x, y, radius)
        robot_radius (float): 机器人半径
        sample_step (float): 沿线段相邻采样点之间的距离

    Returns:
        bool: 无碰撞时为 True，否则为 False
    """
    x1, y1 = first[0], first[1]
    x2, y2 = second[0], second[1]

    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)

    if length == 0:
        # 退化情形：点碰撞检测
        return not is_point_collision(x1, y1, obstacle_list, robot_radius)

    steps = int(length / sample_step) + 1  # 沿线段每隔 sample_step 采样一次

    for i in range(steps + 1):
        t = i / steps
        x = x1 + t * dx
        y = y1 + t * dy

        if is_point_collision(x, y, obstacle_list, robot_radius):
            return False  # 检测到碰撞

    return True  # 安全


def path_smoothing(path, max_iter, obstacle_list, robot_radius=0.0):
    """
    通过迭代地用捷径连接替换路径段来平滑给定路径，
    同时保证新路径段无碰撞。

    算法沿原始路径随机选取两个点，并尝试用直线
    连接它们。若该直线不与任何障碍物碰撞
    （计入机器人半径），则两点之间的中间路径点
    被替换为这条直接连接。

    Args:
        path (List[List[float]]): 原始路径，为 [x, y] 坐标列表。
        max_iter (int): 平滑尝试的迭代次数。
        obstacle_list (List[Tuple[float, float, float]]): 障碍物列表，每个元素表示为
            (x, y, radius)。
        robot_radius (float, optional): 机器人半径，碰撞检测时用于膨胀障碍物尺寸。
            默认值为 0.0。

    Returns:
        List[List[float]]: 平滑后的路径，为 [x, y] 坐标列表。

    Example:
        >>> smoothed = path_smoothing(path, 1000, obstacle_list, robot_radius=0.5)
    """
    le = get_path_length(path)

    for i in range(max_iter):
        # 采样两个点
        pickPoints = [random.uniform(0, le), random.uniform(0, le)]
        pickPoints.sort()
        first = get_target_point(path, pickPoints[0])
        second = get_target_point(path, pickPoints[1])

        if first[2] <= 0 or second[2] <= 0:
            continue

        if (second[2] + 1) > len(path):
            continue

        if second[2] == first[2]:
            continue

        # 碰撞检测
        if not line_collision_check(first, second, obstacle_list, robot_radius):
            continue

        # 创建新路径
        newPath = []
        newPath.extend(path[:first[2] + 1])
        newPath.append([first[0], first[1]])
        newPath.append([second[0], second[1]])
        newPath.extend(path[second[2] + 1:])
        path = newPath
        le = get_path_length(path)

    return path


def main():
    # ====使用 RRT 搜索路径====
    # 参数
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,半径]
    rrt = RRT(start=[0, 0], goal=[6, 10],
              rand_area=[-2, 15], obstacle_list=obstacleList,
              robot_radius=0.3)
    path = rrt.planning(animation=show_animation)

    # 路径平滑
    maxIter = 1000
    smoothedPath = path_smoothing(path, maxIter, obstacleList,
                                  robot_radius=rrt.robot_radius)

    # 绘制最终路径
    if show_animation:
        rrt.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')

        plt.plot([x for (x, y) in smoothedPath], [
            y for (x, y) in smoothedPath], '-c')

        plt.grid(True)
        plt.pause(0.01)  # Mac 上需要
        plt.show()


if __name__ == '__main__':
    main()
