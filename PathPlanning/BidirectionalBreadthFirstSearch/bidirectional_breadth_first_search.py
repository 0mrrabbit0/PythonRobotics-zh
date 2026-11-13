"""

双向广度优先栅格规划

作者: Erwin Lejeune (@spida_rwin)

参见维基百科条目 (https://en.wikipedia.org/wiki/Breadth-first_search)

"""

import math

import matplotlib.pyplot as plt

show_animation = True


class BidirectionalBreadthFirstSearchPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        为广度优先搜索规划初始化栅格地图

        ox: 障碍物的 x 位置列表 [m]
        oy: 障碍物的 y 位置列表 [m]
        resolution: 栅格分辨率 [m]
        rr: 机器人半径[m]
        """

        self.min_x, self.min_y = None, None
        self.max_x, self.max_y = None, None
        self.x_width, self.y_width, self.obstacle_map = None, None, None
        self.resolution = resolution
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index, parent):
            self.x = x  # 栅格索引
            self.y = y  # 栅格索引
            self.cost = cost
            self.parent_index = parent_index
            self.parent = parent

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        基于双向广度优先搜索的规划

        输入:
            s_x: 起点 x 位置 [m]
            s_y: 起点 y 位置 [m]
            gx: 目标点 x 位置 [m]
            gy: 目标点 y 位置 [m]

        输出:
            rx: 最终路径的 x 位置列表
            ry: 最终路径的 y 位置列表
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1,
                               None)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1,
                              None)

        open_set_A, closed_set_A = dict(), dict()
        open_set_B, closed_set_B = dict(), dict()
        open_set_B[self.calc_grid_index(goal_node)] = goal_node
        open_set_A[self.calc_grid_index(start_node)] = start_node

        meet_point_A, meet_point_B = None, None

        while True:
            if len(open_set_A) == 0:
                print("Open set A is empty..")
                break

            if len(open_set_B) == 0:
                print("Open set B is empty")
                break

            current_A = open_set_A.pop(list(open_set_A.keys())[0])
            current_B = open_set_B.pop(list(open_set_B.keys())[0])

            c_id_A = self.calc_grid_index(current_A)
            c_id_B = self.calc_grid_index(current_B)

            closed_set_A[c_id_A] = current_A
            closed_set_B[c_id_B] = current_B

            # 显示搜索图
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current_A.x, self.min_x),
                         self.calc_grid_position(current_A.y, self.min_y),
                         "xc")
                plt.plot(self.calc_grid_position(current_B.x, self.min_x),
                         self.calc_grid_position(current_B.y, self.min_y),
                         "xc")
                # 按 Esc 键停止仿真。
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set_A.keys()) % 10 == 0:
                    plt.pause(0.001)

            if c_id_A in closed_set_B:
                print("Find goal")
                meet_point_A = closed_set_A[c_id_A]
                meet_point_B = closed_set_B[c_id_A]
                break

            elif c_id_B in closed_set_A:
                print("Find goal")
                meet_point_A = closed_set_A[c_id_B]
                meet_point_B = closed_set_B[c_id_B]
                break

            # 基于运动模型扩展搜索栅格
            for i, _ in enumerate(self.motion):
                breakA = False
                breakB = False

                node_A = self.Node(current_A.x + self.motion[i][0],
                                   current_A.y + self.motion[i][1],
                                   current_A.cost + self.motion[i][2],
                                   c_id_A, None)
                node_B = self.Node(current_B.x + self.motion[i][0],
                                   current_B.y + self.motion[i][1],
                                   current_B.cost + self.motion[i][2],
                                   c_id_B, None)

                n_id_A = self.calc_grid_index(node_A)
                n_id_B = self.calc_grid_index(node_B)

                # 若节点不安全，则不做处理
                if not self.verify_node(node_A):
                    breakA = True

                if not self.verify_node(node_B):
                    breakB = True

                if (n_id_A not in closed_set_A) and \
                        (n_id_A not in open_set_A) and (not breakA):
                    node_A.parent = current_A
                    open_set_A[n_id_A] = node_A

                if (n_id_B not in closed_set_B) and \
                        (n_id_B not in open_set_B) and (not breakB):
                    node_B.parent = current_B
                    open_set_B[n_id_B] = node_B

        rx, ry = self.calc_final_path_bidir(
            meet_point_A, meet_point_B, closed_set_A, closed_set_B)
        return rx, ry

    # 根据两侧集合与汇合节点计算最优路径
    def calc_final_path_bidir(self, n1, n2, setA, setB):
        rxA, ryA = self.calc_final_path(n1, setA)
        rxB, ryB = self.calc_final_path(n2, setB)

        rxA.reverse()
        ryA.reverse()

        rx = rxA + rxB
        ry = ryA + ryB

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # 生成最终路径
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        n = closed_set[goal_node.parent_index]
        while n is not None:
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            n = n.parent

        return rx, ry

    def calc_grid_position(self, index, min_position):
        """
        计算栅格位置

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # 碰撞检测
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # 生成障碍物地图
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, 代价
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


def main():
    print(__file__ + " start!!")

    # 起点与目标点位置
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

    # 设置障碍物位置
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "ob")
        plt.grid(True)
        plt.axis("equal")

    bi_bfs = BidirectionalBreadthFirstSearchPlanner(
        ox, oy, grid_size, robot_radius)
    rx, ry = bi_bfs.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.01)
        plt.show()


if __name__ == '__main__':
    main()
