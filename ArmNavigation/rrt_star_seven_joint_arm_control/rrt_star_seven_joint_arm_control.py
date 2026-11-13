"""
七关节机械臂的 RRT* 路径规划器
作者: Mahyar Abdeetedal (mahyaret)
"""
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from n_joint_arm_3d.NLinkArm3d import NLinkArm

show_animation = True
verbose = False


class RobotArm(NLinkArm):
    def get_points(self, joint_angle_list):
        self.set_joint_angles(joint_angle_list)

        x_list = []
        y_list = []
        z_list = []

        trans = np.identity(4)

        x_list.append(trans[0, 3])
        y_list.append(trans[1, 3])
        z_list.append(trans[2, 3])
        for i in range(len(self.link_list)):
            trans = np.dot(trans, self.link_list[i].transformation_matrix())
            x_list.append(trans[0, 3])
            y_list.append(trans[1, 3])
            z_list.append(trans[2, 3])

        return x_list, y_list, z_list


class RRTStar:
    """
    用于 RRT* 规划的类
    """

    class Node:
        def __init__(self, x):
            self.x = x
            self.parent = None
            self.cost = 0.0

    def __init__(self, start, goal, robot, obstacle_list, rand_area,
                 expand_dis=.30,
                 path_resolution=.1,
                 goal_sample_rate=20,
                 max_iter=300,
                 connect_circle_dist=50.0
                 ):
        """
        设置参数

        start:起始位置 [q1,...,qn]
        goal:目标位置 [q1,...,qn]
        obstacleList:障碍物位置 [[x,y,z,size],...]
        randArea:随机采样区域 [min,max]

        """
        self.start = self.Node(start)
        self.end = self.Node(goal)
        self.dimension = len(start)
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.robot = robot
        self.obstacle_list = obstacle_list
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal)
        self.node_list = []
        if show_animation:
            self.ax = plt.axes(projection='3d')

    def planning(self, animation=False, search_until_max_iter=False):
        """
        RRT* 路径规划

        animation: 是否开启动画的标志
        search_until_max_iter: 是否一直搜索到最大迭代次数，
        以改进路径
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            if verbose:
                print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind],
                                  rnd,
                                  self.expand_dis)

            if self.check_collision(new_node, self.robot, self.obstacle_list):
                near_inds = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_inds)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_inds)

            if animation and i % 5 == 0 and self.dimension <= 3:
                self.draw_graph(rnd)

            if (not search_until_max_iter) and new_node:
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)
        if verbose:
            print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)

        return None

    def choose_parent(self, new_node, near_inds):
        if not near_inds:
            return None

        # 在 near_inds 中搜索代价最小者
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node,
                                               self.robot,
                                               self.obstacle_list):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # 碰撞节点的代价
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        dist_to_goal_list = [self.calc_dist_to_goal(n.x)
                             for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(i)
                     for i in dist_to_goal_list if i <= self.expand_dis]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.check_collision(t_node, self.robot, self.obstacle_list):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt(math.log(nnode) / nnode)
        # 若存在 expand_dist，则在
        # 不超过 expand_dist 的范围内搜索顶点
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        dist_list = [np.sum((np.array(node.x) - np.array(new_node.x)) ** 2)
                     for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r ** 2]
        return near_inds

    def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node,
                                                self.robot,
                                                self.obstacle_list)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                self.node_list[i] = edge_node
                self.propagate_cost_to_leaves(new_node)

    def calc_new_cost(self, from_node, to_node):
        d, _, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    def generate_final_course(self, goal_ind):
        path = [self.end.x]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append(node.x)
            node = node.parent
        path.append(node.x)
        reversed(path)
        return path

    def calc_dist_to_goal(self, x):
        distance = np.linalg.norm(np.array(x) - np.array(self.end.x))
        return distance

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(np.random.uniform(self.min_rand,
                                              self.max_rand,
                                              self.dimension))
        else:  # 目标点采样
            rnd = self.Node(self.end.x)
        return rnd

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(list(from_node.x))
        d, phi, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [list(new_node.x)]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        start, end = np.array(from_node.x), np.array(to_node.x)
        v = end - start
        u = v / (np.sqrt(np.sum(v ** 2)))
        for _ in range(n_expand):
            new_node.x += u * self.path_resolution
            new_node.path_x.append(list(new_node.x))

        d, _, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(list(to_node.x))

        new_node.parent = from_node

        return new_node

    def draw_graph(self, rnd=None):
        plt.cla()
        self.ax.axis([-1, 1, -1, 1, -1, 1])
        self.ax.set_zlim(0, 1)
        self.ax.grid(True)
        for (ox, oy, oz, size) in self.obstacle_list:
            self.plot_sphere(self.ax, ox, oy, oz, size=size)
        if self.dimension > 3:
            return self.ax
        if rnd is not None:
            self.ax.plot([rnd.x[0]], [rnd.x[1]], [rnd.x[2]], "^k")
        for node in self.node_list:
            if node.parent:
                path = np.array(node.path_x)
                plt.plot(path[:, 0], path[:, 1], path[:, 2], "-g")
        self.ax.plot([self.start.x[0]], [self.start.x[1]],
                     [self.start.x[2]], "xr")
        self.ax.plot([self.end.x[0]], [self.end.x[1]], [self.end.x[2]], "xr")
        plt.pause(0.01)
        return self.ax

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [np.sum((np.array(node.x) - np.array(rnd_node.x))**2)
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def plot_sphere(ax, x, y, z, size=1, color="k"):
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        xl = x+size*np.cos(u)*np.sin(v)
        yl = y+size*np.sin(u)*np.sin(v)
        zl = z+size*np.cos(v)
        ax.plot_wireframe(xl, yl, zl, color=color)

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x[0] - from_node.x[0]
        dy = to_node.x[1] - from_node.x[1]
        dz = to_node.x[2] - from_node.x[2]
        d = np.sqrt(np.sum((np.array(to_node.x) - np.array(from_node.x))**2))
        phi = math.atan2(dy, dx)
        theta = math.atan2(math.hypot(dx, dy), dz)
        return d, phi, theta

    @staticmethod
    def calc_distance_and_angle2(from_node, to_node):
        dx = to_node.x[0] - from_node.x[0]
        dy = to_node.x[1] - from_node.x[1]
        dz = to_node.x[2] - from_node.x[2]
        d = math.sqrt(dx**2 + dy**2 + dz**2)
        phi = math.atan2(dy, dx)
        theta = math.atan2(math.hypot(dx, dy), dz)
        return d, phi, theta

    @staticmethod
    def check_collision(node, robot, obstacleList):

        if node is None:
            return False

        for (ox, oy, oz, size) in obstacleList:
            for x in node.path_x:
                x_list, y_list, z_list = robot.get_points(x)
                dx_list = [ox - x_point for x_point in x_list]
                dy_list = [oy - y_point for y_point in y_list]
                dz_list = [oz - z_point for z_point in z_list]
                d_list = [dx * dx + dy * dy + dz * dz
                          for (dx, dy, dz) in zip(dx_list,
                                                  dy_list,
                                                  dz_list)]

                if min(d_list) <= size ** 2:
                    return False  # 碰撞

        return True  # 安全


def main():
    print("Start " + __file__)

    # 用 panda 的 Denavit-Hartenberg 参数初始化 NLinkArm
    # https://frankaemika.github.io/docs/control_parameters.html
    # [theta, alpha, a, d]
    seven_joint_arm = RobotArm([[0., math.pi/2., 0., .333],
                                [0., -math.pi/2., 0., 0.],
                                [0., math.pi/2., 0.0825, 0.3160],
                                [0., -math.pi/2., -0.0825, 0.],
                                [0., math.pi/2., 0., 0.3840],
                                [0., math.pi/2., 0.088, 0.],
                                [0., 0., 0., 0.107]])
    # ====用 RRT 搜索路径====
    obstacle_list = [
        (-.3, -.3, .7, .1),
        (.0, -.3, .7, .1),
        (.2, -.1, .3, .15),
    ]  # [x,y,尺寸(半径)]
    start = [0 for _ in range(len(seven_joint_arm.link_list))]
    end = [1.5 for _ in range(len(seven_joint_arm.link_list))]
    # 设置初始参数
    rrt_star = RRTStar(start=start,
                       goal=end,
                       rand_area=[0, 2],
                       max_iter=200,
                       robot=seven_joint_arm,
                       obstacle_list=obstacle_list)
    path = rrt_star.planning(animation=show_animation,
                             search_until_max_iter=False)

    if path is None:
        print("Cannot find path.")
    else:
        print("Found path!")

        # 绘制最终路径
        if show_animation:
            ax = rrt_star.draw_graph()

            # 绘制最终构型
            x_points, y_points, z_points = seven_joint_arm.get_points(path[-1])
            ax.plot([x for x in x_points],
                    [y for y in y_points],
                    [z for z in z_points],
                    "o-", color="red", ms=5, mew=0.5)

            for i, q in enumerate(path):
                x_points, y_points, z_points = seven_joint_arm.get_points(q)
                ax.plot([x for x in x_points],
                        [y for y in y_points],
                        [z for z in z_points],
                        "o-", color="grey",  ms=4, mew=0.5)
                plt.pause(0.1)

            plt.show()


if __name__ == '__main__':
    main()
