"""

使用 RRT 与 Reeds-Shepp 路径的路径规划示例代码

作者: AtsushiSakai(@Atsushi_twi)

"""
import copy
import math
import random
import sys
import pathlib
import matplotlib.pyplot as plt
import numpy as np
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from ReedsSheppPath import reeds_shepp_path_planning
from RRTStar.rrt_star import RRTStar

show_animation = True


class RRTStarReedsShepp(RRTStar):
    """
    使用 Reeds-Shepp 路径进行 RRT* 规划的类
    """

    class Node(RRTStar.Node):
        """
        RRT 节点
        """

        def __init__(self, x, y, yaw):
            super().__init__(x, y)
            self.yaw = yaw
            self.path_yaw = []

    def __init__(self, start, goal, obstacle_list, rand_area,
                 max_iter=200, step_size=0.2,
                 connect_circle_dist=50.0,
                 robot_radius=0.0
                 ):
        """
        参数设置

        start:起点位置 [x,y]
        goal:目标点位置 [x,y]
        obstacleList:障碍物位置 [[x,y,尺寸],...]
        randArea:随机采样区域 [min,max]
        robot_radius: 将机器人本体建模为给定半径的圆

        """
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.max_iter = max_iter
        self.step_size = step_size
        self.obstacle_list = obstacle_list
        self.connect_circle_dist = connect_circle_dist
        self.robot_radius = robot_radius

        self.curvature = 1.0
        self.goal_yaw_th = np.deg2rad(1.0)
        self.goal_xy_th = 0.5

    def set_random_seed(self, seed):
        random.seed(seed)

    def planning(self, animation=True, search_until_max_iter=True):
        """
        规划

        animation: 是否开启动画的标志
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd)

            if self.check_collision(
                    new_node, self.obstacle_list, self.robot_radius):
                near_indexes = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_indexes)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_indexes)
                    self.try_goal_path(new_node)

            if animation and i % 5 == 0:
                self.plot_start_goal_arrow()
                self.draw_graph(rnd)

            if (not search_until_max_iter) and new_node:  # 检查是否到达目标点
                last_index = self.search_best_goal_node()
                if last_index:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index:
            return self.generate_final_course(last_index)
        else:
            print("Cannot find path")

        return None

    def try_goal_path(self, node):

        goal = self.Node(self.end.x, self.end.y, self.end.yaw)

        new_node = self.steer(node, goal)
        if new_node is None:
            return

        if self.check_collision(
                new_node, self.obstacle_list, self.robot_radius):
            self.node_list.append(new_node)

    def draw_graph(self, rnd=None):
        plt.clf()
        # 按 Esc 键停止仿真。
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        self.plot_start_goal_arrow()
        plt.pause(0.01)

    def plot_start_goal_arrow(self):
        reeds_shepp_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        reeds_shepp_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)

    def steer(self, from_node, to_node):

        px, py, pyaw, mode, course_lengths = reeds_shepp_path_planning.reeds_shepp_path_planning(
            from_node.x, from_node.y, from_node.yaw, to_node.x,
            to_node.y, to_node.yaw, self.curvature, self.step_size)

        if not px:
            return None

        new_node = copy.deepcopy(from_node)
        new_node.x = px[-1]
        new_node.y = py[-1]
        new_node.yaw = pyaw[-1]

        new_node.path_x = px
        new_node.path_y = py
        new_node.path_yaw = pyaw
        new_node.cost += sum([abs(l) for l in course_lengths])
        new_node.parent = from_node

        return new_node

    def calc_new_cost(self, from_node, to_node):

        _, _, _, _, course_lengths = reeds_shepp_path_planning.reeds_shepp_path_planning(
            from_node.x, from_node.y, from_node.yaw, to_node.x,
            to_node.y, to_node.yaw, self.curvature, self.step_size)
        if not course_lengths:
            return float("inf")

        return from_node.cost + sum([abs(l) for l in course_lengths])

    def get_random_node(self):

        rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                        random.uniform(self.min_rand, self.max_rand),
                        random.uniform(-math.pi, math.pi)
                        )

        return rnd

    def search_best_goal_node(self):

        goal_indexes = []
        for (i, node) in enumerate(self.node_list):
            if self.calc_dist_to_goal(node.x, node.y) <= self.goal_xy_th:
                goal_indexes.append(i)
        print("goal_indexes:", len(goal_indexes))

        # 航向角检查
        final_goal_indexes = []
        for i in goal_indexes:
            if abs(self.node_list[i].yaw - self.end.yaw) <= self.goal_yaw_th:
                final_goal_indexes.append(i)

        print("final_goal_indexes:", len(final_goal_indexes))

        if not final_goal_indexes:
            return None

        min_cost = min([self.node_list[i].cost for i in final_goal_indexes])
        print("min_cost:", min_cost)
        for i in final_goal_indexes:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def generate_final_course(self, goal_index):
        path = [[self.end.x, self.end.y, self.end.yaw]]
        node = self.node_list[goal_index]
        while node.parent:
            for (ix, iy, iyaw) in zip(reversed(node.path_x), reversed(node.path_y), reversed(node.path_yaw)):
                path.append([ix, iy, iyaw])
            node = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw])
        return path


def main(max_iter=100):
    print("Start " + __file__)

    # ====使用 RRT 搜索路径====
    obstacleList = [
        (5, 5, 1),
        (4, 6, 1),
        (4, 8, 1),
        (4, 10, 1),
        (6, 5, 1),
        (7, 5, 1),
        (8, 6, 1),
        (8, 8, 1),
        (8, 10, 1)
    ]  # [x,y,尺寸(半径)]

    # 设置初始参数
    start = [0.0, 0.0, np.deg2rad(0.0)]
    goal = [6.0, 7.0, np.deg2rad(90.0)]

    rrt_star_reeds_shepp = RRTStarReedsShepp(start, goal,
                                             obstacleList,
                                             [-2.0, 15.0], max_iter=max_iter)
    path = rrt_star_reeds_shepp.planning(animation=show_animation)

    # 绘制最终路径
    if path and show_animation:  # pragma: no cover
        rrt_star_reeds_shepp.draw_graph()
        plt.plot([x for (x, y, yaw) in path], [y for (x, y, yaw) in path], '-r')
        plt.grid(True)
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()
