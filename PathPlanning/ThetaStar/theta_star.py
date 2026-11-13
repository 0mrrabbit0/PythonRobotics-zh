"""

Theta* 栅格路径规划

作者：Musab Kasbati (@Musab1Blaser)

参见论文 (https://cdn.aaai.org/AAAI/2007/AAAI07-187.pdf)

"""

import math

import matplotlib.pyplot as plt

show_animation = True
use_theta_star = True


class ThetaStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        为 Theta* 规划初始化栅格地图

        ox：障碍物的 x 坐标列表 [m]
        oy：障碍物的 y 坐标列表 [m]
        resolution：栅格分辨率 [m]
        rr：机器人半径 [m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # 栅格索引
            self.y = y  # 栅格索引
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy): 
        """
        Theta* 路径搜索

        输入：
            s_x：起点 x 坐标 [m]
            s_y：起点 y 坐标 [m]
            gx：目标点 x 坐标 [m]
            gy：目标点 y 坐标 [m]

        输出：
            rx：最终路径的 x 坐标列表
            ry：最终路径的 y 坐标列表
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # 显示搜索过程
            if show_animation:  # pragma: no cover
                x = self.calc_grid_position(current.x, self.min_x)
                y = self.calc_grid_position(current.y, self.min_y)

                # 绘制指向父节点的箭头
                if current.parent_index != -1 and current.parent_index in closed_set:
                    parent = closed_set[current.parent_index]
                    px = self.calc_grid_position(parent.x, self.min_x)
                    py = self.calc_grid_position(parent.y, self.min_y)

                    # 从当前节点指向父节点的向量
                    dx = px - x
                    dy = py - y

                    # 缩小向量以便观察
                    norm = math.hypot(dx, dy)
                    dx /= norm
                    dy /= norm

                    # 绘制小箭头（缩小以便观察）
                    plt.arrow(x, y, dx, dy,
                            head_width=0.5, head_length=0.5,
                            fc='c', ec='c', alpha=0.7)

                # 按 Esc 键停止仿真
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None]
                )

                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # 从开集中移除该项
            del open_set[c_id]

            # 将其加入闭集
            closed_set[c_id] = current

            # 根据运动模型扩展搜索栅格
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                    current.y + self.motion[i][1],
                                    current.cost + self.motion[i][2], c_id)  # 代价稍后可能被 Theta* 路径压缩更新
                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                # Theta* 修改：
                if use_theta_star and current.parent_index != -1 and current.parent_index in closed_set:
                    grandparent = closed_set[current.parent_index]
                    if self.line_of_sight(grandparent, node):
                        # 若当前节点的父节点与邻居之间存在视线
                        node.cost = grandparent.cost + math.hypot(node.x - grandparent.x, node.y - grandparent.y)
                        node.parent_index = current.parent_index # 将路径直接压缩到祖父节点

                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        # 这是迄今最优路径，记录它
                        open_set[n_id] = node 


        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # 生成最终路径
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # 启发式权重
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

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

    def line_of_sight(self, node1, node2):
        """
        检查两个节点之间是否存在直接视线。
        使用 Bresenham 直线算法遍历栅格。
        """
        x0 = node1.x
        y0 = node1.y
        x1 = node2.x
        y1 = node2.y

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = dx - dy

        while True:
            if not self.verify_node(self.Node(x0, y0, 0, -1)):
                return False
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return True


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

        # 碰撞检查
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
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    theta_star = ThetaStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = theta_star.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()
