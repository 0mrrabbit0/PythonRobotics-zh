"""
A* 算法
作者: Weicent
随机生成障碍物、起点与目标点
同时从起点和终点搜索路径
"""

import numpy as np
import matplotlib.pyplot as plt
import math

show_animation = True


class Node:
    """具有 g、h、坐标和父节点属性的节点"""

    def __init__(self, G=0, H=0, coordinate=None, parent=None):
        self.G = G
        self.H = H
        self.F = G + H
        self.parent = parent
        self.coordinate = coordinate

    def reset_f(self):
        self.F = self.G + self.H


def hcost(node_coordinate, goal):
    dx = abs(node_coordinate[0] - goal[0])
    dy = abs(node_coordinate[1] - goal[1])
    hcost = dx + dy
    return hcost


def gcost(fixed_node, update_node_coordinate):
    dx = abs(fixed_node.coordinate[0] - update_node_coordinate[0])
    dy = abs(fixed_node.coordinate[1] - update_node_coordinate[1])
    gc = math.hypot(dx, dy)  # gc = 从 fixed_node 移动到 update_node
    gcost = fixed_node.G + gc  # gcost = 从起点移动到 update_node
    return gcost


def boundary_and_obstacles(start, goal, top_vertex, bottom_vertex, obs_number):
    """
    :param start: 起点坐标
    :param goal: 目标点坐标
    :param top_vertex: 边界右上顶点坐标
    :param bottom_vertex: 边界左下顶点坐标
    :param obs_number: 地图中生成的障碍物数量
    :return: 边界与障碍物数组，障碍物列表
    """
    # 以下可合并为矩形边界
    ay = list(range(bottom_vertex[1], top_vertex[1]))
    ax = [bottom_vertex[0]] * len(ay)
    cy = ay
    cx = [top_vertex[0]] * len(cy)
    bx = list(range(bottom_vertex[0] + 1, top_vertex[0]))
    by = [bottom_vertex[1]] * len(bx)
    dx = [bottom_vertex[0]] + bx + [top_vertex[0]]
    dy = [top_vertex[1]] * len(dx)

    # 生成随机障碍物
    ob_x = np.random.randint(bottom_vertex[0] + 1,
                             top_vertex[0], obs_number).tolist()
    ob_y = np.random.randint(bottom_vertex[1] + 1,
                             top_vertex[1], obs_number).tolist()
    # 按一定顺序排列的边界 x、y 坐标
    x = ax + bx + cx + dx
    y = ay + by + cy + dy
    obstacle = np.vstack((ob_x, ob_y)).T.tolist()
    # 从障碍物列表中移除起点和目标点坐标
    obstacle = [coor for coor in obstacle if coor != start and coor != goal]
    obs_array = np.array(obstacle)
    bound = np.vstack((x, y)).T
    bound_obs = np.vstack((bound, obs_array))
    return bound_obs, obstacle


def find_neighbor(node, ob, closed):
    # 将障碍物转换为集合以加快查找
    ob_set = set(map(tuple, ob.tolist()))  
    neighbor_set = set()

    # 生成节点周围 3x3 栅格内的邻居
    for x in range(node.coordinate[0] - 1, node.coordinate[0] + 2):
        for y in range(node.coordinate[1] - 1, node.coordinate[1] + 2):
            coord = (x, y)
            if coord not in ob_set and coord != tuple(node.coordinate):
                neighbor_set.add(coord)

    # 定义基本方向与对角线方向上的邻居
    top_nei = (node.coordinate[0], node.coordinate[1] + 1)
    bottom_nei = (node.coordinate[0], node.coordinate[1] - 1)
    left_nei = (node.coordinate[0] - 1, node.coordinate[1])
    right_nei = (node.coordinate[0] + 1, node.coordinate[1])
    lt_nei = (node.coordinate[0] - 1, node.coordinate[1] + 1)
    rt_nei = (node.coordinate[0] + 1, node.coordinate[1] + 1)
    lb_nei = (node.coordinate[0] - 1, node.coordinate[1] - 1)
    rb_nei = (node.coordinate[0] + 1, node.coordinate[1] - 1)

    # 移除违反对角运动规则的邻居
    if top_nei in ob_set and left_nei in ob_set:
        neighbor_set.discard(lt_nei)
    if top_nei in ob_set and right_nei in ob_set:
        neighbor_set.discard(rt_nei)
    if bottom_nei in ob_set and left_nei in ob_set:
        neighbor_set.discard(lb_nei)
    if bottom_nei in ob_set and right_nei in ob_set:
        neighbor_set.discard(rb_nei)

    # 滤除位于闭集中的邻居
    closed_set = set(map(tuple, closed))
    neighbor_set -= closed_set

    return list(neighbor_set)



def find_node_index(coordinate, node_list):
    # 通过坐标在节点列表中查找节点索引
    ind = 0
    for node in node_list:
        if node.coordinate == coordinate:
            target_node = node
            ind = node_list.index(target_node)
            break
    return ind


def find_path(open_list, closed_list, goal, obstacle):
    # 搜索路径，更新开集与闭集
    # obstacle = 障碍物与边界
    flag = len(open_list)
    for i in range(flag):
        node = open_list[0]
        open_coordinate_list = [node.coordinate for node in open_list]
        closed_coordinate_list = [node.coordinate for node in closed_list]
        temp = find_neighbor(node, obstacle, closed_coordinate_list)
        for element in temp:
            if element in closed_list:
                continue
            elif element in open_coordinate_list:
                # 若节点已在开集中，则更新 g 值
                ind = open_coordinate_list.index(element)
                new_g = gcost(node, element)
                if new_g <= open_list[ind].G:
                    open_list[ind].G = new_g
                    open_list[ind].reset_f()
                    open_list[ind].parent = node
            else:  # 新坐标，创建对应节点
                ele_node = Node(coordinate=element, parent=node,
                                G=gcost(node, element), H=hcost(element, goal))
                open_list.append(ele_node)
        open_list.remove(node)
        closed_list.append(node)
        open_list.sort(key=lambda x: x.F)
    return open_list, closed_list


def node_to_coordinate(node_list):
    # 将节点列表转换为坐标列表和数组
    coordinate_list = [node.coordinate for node in node_list]
    return coordinate_list


def check_node_coincide(close_ls1, closed_ls2):
    """
    :param close_ls1: 从起点搜索得到的节点闭集
    :param closed_ls2: 从终点搜索得到的节点闭集
    :return: 上述两者的相交节点列表
    """
    # 检查 close_ls1 中的节点是否与 closed_ls2 中的节点相交
    cl1 = node_to_coordinate(close_ls1)
    cl2 = node_to_coordinate(closed_ls2)
    intersect_ls = [node for node in cl1 if node in cl2]
    return intersect_ls


def find_surrounding(coordinate, obstacle):
    # 查找节点周围的障碍物，用于绘制边界线
    boundary: list = []
    for x in range(coordinate[0] - 1, coordinate[0] + 2):
        for y in range(coordinate[1] - 1, coordinate[1] + 2):
            if [x, y] in obstacle:
                boundary.append([x, y])
    return boundary


def get_border_line(node_closed_ls, obstacle):
    # 若无路径，则找出限制目标点或机器人的边界线
    border: list = []
    coordinate_closed_ls = node_to_coordinate(node_closed_ls)
    for coordinate in coordinate_closed_ls:
        temp = find_surrounding(coordinate, obstacle)
        border = border + temp
    border_ary = np.array(border)
    return border_ary


def get_path(org_list, goal_list, coordinate):
    # 获取从起点到终点的路径
    path_org: list = []
    path_goal: list = []
    ind = find_node_index(coordinate, org_list)
    node = org_list[ind]
    while node != org_list[0]:
        path_org.append(node.coordinate)
        node = node.parent
    path_org.append(org_list[0].coordinate)
    ind = find_node_index(coordinate, goal_list)
    node = goal_list[ind]
    while node != goal_list[0]:
        path_goal.append(node.coordinate)
        node = node.parent
    path_goal.append(goal_list[0].coordinate)
    path_org.reverse()
    path = path_org + path_goal
    path = np.array(path)
    return path


def random_coordinate(bottom_vertex, top_vertex):
    # 在迷宫内部生成随机坐标
    coordinate = [np.random.randint(bottom_vertex[0] + 1, top_vertex[0]),
                  np.random.randint(bottom_vertex[1] + 1, top_vertex[1])]
    return coordinate


def draw(close_origin, close_goal, start, end, bound):
    # 绘制地图
    if not close_goal.tolist():  # 确保 close_goal 非空
        # 当障碍物数量非常大（>4500）时，
        # 起点很可能在第一次搜索时就被阻塞，随后
        # 程序结束，从目标点到起点的搜索
        # 不会开始，从而目标点的 closed_list 仍为 []
        # 为了绘制地图，将终点坐标加入数组
        close_goal = np.array([end])
    plt.cla()
    plt.gcf().set_size_inches(11, 9, forward=True)
    plt.axis('equal')
    plt.plot(close_origin[:, 0], close_origin[:, 1], 'oy')
    plt.plot(close_goal[:, 0], close_goal[:, 1], 'og')
    plt.plot(bound[:, 0], bound[:, 1], 'sk')
    plt.plot(end[0], end[1], '*b', label='Goal')
    plt.plot(start[0], start[1], '^b', label='Origin')
    plt.legend()
    plt.pause(0.0001)


def draw_control(org_closed, goal_closed, flag, start, end, bound, obstacle):
    """
    控制绘图过程，判断搜索是否结束
    flag == 0 : 绘制搜索过程并画出路径
    flag == 1 或 2 : 起点或终点被阻塞，绘制边界线
    """
    stop_loop = 0  # 搜索停止标志
    org_closed_ls = node_to_coordinate(org_closed)
    org_array = np.array(org_closed_ls)
    goal_closed_ls = node_to_coordinate(goal_closed)
    goal_array = np.array(goal_closed_ls)
    path = None
    if show_animation:  # 绘制搜索过程
        draw(org_array, goal_array, start, end, bound)
    if flag == 0:
        node_intersect = check_node_coincide(org_closed, goal_closed)
        if node_intersect:  # 已找到路径
            path = get_path(org_closed, goal_closed, node_intersect[0])
            stop_loop = 1
            print('Path found!')
            if show_animation:  # 绘制路径
                plt.plot(path[:, 0], path[:, 1], '-r')
                plt.title('Robot Arrived', size=20, loc='center')
                plt.pause(0.01)
                plt.show()
    elif flag == 1:  # 起点先被阻塞
        stop_loop = 1
        print('There is no path to the goal! Start point is blocked!')
    elif flag == 2:  # 终点先被阻塞
        stop_loop = 1
        print('There is no path to the goal! End point is blocked!')
    if show_animation:  # 阻塞情形，绘制边界线
        info = 'There is no path to the goal!' \
               ' Robot&Goal are split by border' \
               ' shown in red \'x\'!'
        if flag == 1:
            border = get_border_line(org_closed, obstacle)
            plt.plot(border[:, 0], border[:, 1], 'xr')
            plt.title(info, size=14, loc='center')
            plt.pause(0.01)
            plt.show()
        elif flag == 2:
            border = get_border_line(goal_closed, obstacle)
            plt.plot(border[:, 0], border[:, 1], 'xr')
            plt.title(info, size=14, loc='center')
            plt.pause(0.01)
            plt.show()
    return stop_loop, path


def searching_control(start, end, bound, obstacle):
    """管理搜索过程，从两侧开始搜索"""
    # 初始化起点节点和终点节点
    origin = Node(coordinate=start, H=hcost(start, end))
    goal = Node(coordinate=end, H=hcost(end, start))
    # 从起点向目标点搜索的列表
    origin_open: list = [origin]
    origin_close: list = []
    # 从目标点向起点搜索的列表
    goal_open = [goal]
    goal_close: list = []
    # 初始目标
    target_goal = end
    # flag = 0（未阻塞）1（起点被阻塞）2（终点被阻塞）
    flag = 0  # 初始化标志
    path = None
    while True:
        # 从起点向终点搜索
        origin_open, origin_close = \
            find_path(origin_open, origin_close, target_goal, bound)
        if not origin_open:  # 无路径情形
            flag = 1  # 起点节点被阻塞
            draw_control(origin_close, goal_close, flag, start, end, bound,
                         obstacle)
            break
        # 更新从终点向起点搜索的目标
        target_origin = min(origin_open, key=lambda x: x.F).coordinate

        # 从终点向起点搜索
        goal_open, goal_close = \
            find_path(goal_open, goal_close, target_origin, bound)
        if not goal_open:  # 无路径情形
            flag = 2  # 目标点被阻塞
            draw_control(origin_close, goal_close, flag, start, end, bound,
                         obstacle)
            break
        # 更新从起点向终点搜索的目标
        target_goal = min(goal_open, key=lambda x: x.F).coordinate

        # 继续搜索，绘制过程
        stop_sign, path = draw_control(origin_close, goal_close, flag, start,
                                       end, bound, obstacle)
        if stop_sign:
            break
    return path


def main(obstacle_number=1500):
    print(__file__ + ' start!')

    top_vertex = [60, 60]  # 边界右上顶点
    bottom_vertex = [0, 0]  # 边界左下顶点

    # 随机生成起点和目标点
    start = random_coordinate(bottom_vertex, top_vertex)
    end = random_coordinate(bottom_vertex, top_vertex)

    # 生成边界和障碍物
    bound, obstacle = boundary_and_obstacles(start, end, top_vertex,
                                             bottom_vertex,
                                             obstacle_number)

    path = searching_control(start, end, bound, obstacle)
    if not show_animation:
        print(path)


if __name__ == '__main__':
    main(obstacle_number=1500)
