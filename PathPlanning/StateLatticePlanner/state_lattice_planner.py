"""

结合模型预测轨迹生成器的状态格规划器

作者：Atsushi Sakai (@Atsushi_twi)

- plookuptable.csv 由以下脚本生成：
https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning
/ModelPredictiveTrajectoryGenerator/lookup_table_generator.py

Reference:

- 复杂环境中高性能移动机器人可行运动的状态空间采样
https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf
&doi=e2256b5b24137f89e473f01df288cb3aa72e56a0

"""
import sys
import os
from matplotlib import pyplot as plt
import numpy as np
import math
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.parent))
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import ModelPredictiveTrajectoryGenerator.trajectory_generator as planner
import ModelPredictiveTrajectoryGenerator.motion_model as motion_model

TABLE_PATH = os.path.dirname(os.path.abspath(__file__)) + "/lookup_table.csv"

show_animation = True


def search_nearest_one_from_lookup_table(t_x, t_y, t_yaw, lookup_table):
    mind = float("inf")
    minid = -1

    for (i, table) in enumerate(lookup_table):
        dx = t_x - table[0]
        dy = t_y - table[1]
        dyaw = t_yaw - table[2]
        d = math.sqrt(dx ** 2 + dy ** 2 + dyaw ** 2)
        if d <= mind:
            minid = i
            mind = d

    return lookup_table[minid]


def get_lookup_table(table_path):
    return np.loadtxt(table_path, delimiter=',', skiprows=1)


def generate_path(target_states, k0):
    # x, y, yaw（航向角）, s, km, kf
    lookup_table = get_lookup_table(TABLE_PATH)
    result = []

    for state in target_states:
        bestp = search_nearest_one_from_lookup_table(
            state[0], state[1], state[2], lookup_table)

        target = motion_model.State(x=state[0], y=state[1], yaw=state[2])
        init_p = np.array(
            [np.hypot(state[0], state[1]), bestp[4], bestp[5]]).reshape(3, 1)

        x, y, yaw, p = planner.optimize_trajectory(target, k0, init_p)

        if x is not None:
            print("find good path")
            result.append(
                [x[-1], y[-1], yaw[-1], float(p[0, 0]), float(p[1, 0]), float(p[2, 0])])

    print("finish path generation")
    return result


def calc_uniform_polar_states(nxy, nh, d, a_min, a_max, p_min, p_max):
    """

    Parameters
    ----------
    nxy :
        位置采样数量
    nh :
        航向采样数量
    d :
        终端状态的距离
    a_min :
        位置采样的最小角度
    a_max :
        位置采样的最大角度
    p_min :
        航向采样的最小角度
    p_max :
        航向采样的最大角度

    Returns
    -------

    """
    angle_samples = [i / (nxy - 1) for i in range(nxy)]
    states = sample_states(angle_samples, a_min, a_max, d, p_max, p_min, nh)

    return states


def calc_biased_polar_states(goal_angle, ns, nxy, nh, d, a_min, a_max, p_min, p_max):
    """
    计算有偏状态

    :param goal_angle: 有偏采样所用的目标航向
    :param ns: 有偏采样的数量
    :param nxy: 位置采样数量
    :param nxy: 位置采样数量
    :param nh: 航向采样数量
    :param d: 终端状态的距离
    :param a_min: 位置采样的最小角度
    :param a_max: 位置采样的最大角度
    :param p_min: 航向采样的最小角度
    :param p_max: 航向采样的最大角度
    :return: 状态列表
    """

    asi = [a_min + (a_max - a_min) * i / (ns - 1) for i in range(ns - 1)]
    cnav = [math.pi - abs(i - goal_angle) for i in asi]

    cnav_sum = sum(cnav)
    cnav_max = max(cnav)

    # 归一化
    cnav = [(cnav_max - cnav[i]) / (cnav_max * ns - cnav_sum)
            for i in range(ns - 1)]

    csumnav = np.cumsum(cnav)
    di = []
    li = 0
    for i in range(nxy):
        for ii in range(li, ns - 1):
            if ii / ns >= i / (nxy - 1):
                di.append(csumnav[ii])
                li = ii - 1
                break

    states = sample_states(di, a_min, a_max, d, p_max, p_min, nh)

    return states


def calc_lane_states(l_center, l_heading, l_width, v_width, d, nxy):
    """

    计算车道状态

    :param l_center: 车道横向位置
    :param l_heading: 车道航向角
    :param l_width: 车道宽度
    :param v_width: 车辆宽度
    :param d: 纵向位置
    :param nxy: 采样数量
    :return: 状态列表
    """
    xc = d
    yc = l_center

    states = []
    for i in range(nxy):
        delta = -0.5 * (l_width - v_width) + \
            (l_width - v_width) * i / (nxy - 1)
        xf = xc - delta * math.sin(l_heading)
        yf = yc + delta * math.cos(l_heading)
        yawf = l_heading
        states.append([xf, yf, yawf])

    return states


def sample_states(angle_samples, a_min, a_max, d, p_max, p_min, nh):
    states = []
    for i in angle_samples:
        a = a_min + (a_max - a_min) * i

        for j in range(nh):
            xf = d * math.cos(a)
            yf = d * math.sin(a)
            if nh == 1:
                yawf = (p_max - p_min) / 2 + a
            else:
                yawf = p_min + (p_max - p_min) * j / (nh - 1) + a
            states.append([xf, yf, yawf])

    return states


def uniform_terminal_state_sampling_test1():
    k0 = 0.0
    nxy = 5
    nh = 3
    d = 20
    a_min = - np.deg2rad(45.0)
    a_max = np.deg2rad(45.0)
    p_min = - np.deg2rad(45.0)
    p_max = np.deg2rad(45.0)
    states = calc_uniform_polar_states(nxy, nh, d, a_min, a_max, p_min, p_max)
    result = generate_path(states, k0)

    for table in result:
        xc, yc, yawc = motion_model.generate_trajectory(
            table[3], table[4], table[5], k0)

        if show_animation:
            plt.plot(xc, yc, "-r")

    if show_animation:
        plt.grid(True)
        plt.axis("equal")
        plt.show()

    print("Done")


def uniform_terminal_state_sampling_test2():
    k0 = 0.1
    nxy = 6
    nh = 3
    d = 20
    a_min = - np.deg2rad(-10.0)
    a_max = np.deg2rad(45.0)
    p_min = - np.deg2rad(20.0)
    p_max = np.deg2rad(20.0)
    states = calc_uniform_polar_states(nxy, nh, d, a_min, a_max, p_min, p_max)
    result = generate_path(states, k0)

    for table in result:
        xc, yc, yawc = motion_model.generate_trajectory(
            table[3], table[4], table[5], k0)

        if show_animation:
            plt.plot(xc, yc, "-r")

    if show_animation:
        plt.grid(True)
        plt.axis("equal")
        plt.show()

    print("Done")


def biased_terminal_state_sampling_test1():
    k0 = 0.0
    nxy = 30
    nh = 2
    d = 20
    a_min = np.deg2rad(-45.0)
    a_max = np.deg2rad(45.0)
    p_min = - np.deg2rad(20.0)
    p_max = np.deg2rad(20.0)
    ns = 100
    goal_angle = np.deg2rad(0.0)
    states = calc_biased_polar_states(
        goal_angle, ns, nxy, nh, d, a_min, a_max, p_min, p_max)
    result = generate_path(states, k0)

    for table in result:
        xc, yc, yawc = motion_model.generate_trajectory(
            table[3], table[4], table[5], k0)
        if show_animation:
            plt.plot(xc, yc, "-r")

    if show_animation:
        plt.grid(True)
        plt.axis("equal")
        plt.show()


def biased_terminal_state_sampling_test2():
    k0 = 0.0
    nxy = 30
    nh = 1
    d = 20
    a_min = np.deg2rad(0.0)
    a_max = np.deg2rad(45.0)
    p_min = - np.deg2rad(20.0)
    p_max = np.deg2rad(20.0)
    ns = 100
    goal_angle = np.deg2rad(30.0)
    states = calc_biased_polar_states(
        goal_angle, ns, nxy, nh, d, a_min, a_max, p_min, p_max)
    result = generate_path(states, k0)

    for table in result:
        xc, yc, yawc = motion_model.generate_trajectory(
            table[3], table[4], table[5], k0)

        if show_animation:
            plt.plot(xc, yc, "-r")

    if show_animation:
        plt.grid(True)
        plt.axis("equal")
        plt.show()


def lane_state_sampling_test1():
    k0 = 0.0

    l_center = 10.0
    l_heading = np.deg2rad(0.0)
    l_width = 3.0
    v_width = 1.0
    d = 10
    nxy = 5
    states = calc_lane_states(l_center, l_heading, l_width, v_width, d, nxy)
    result = generate_path(states, k0)

    if show_animation:
        plt.close("all")

    for table in result:
        x_c, y_c, yaw_c = motion_model.generate_trajectory(
            table[3], table[4], table[5], k0)

        if show_animation:
            plt.plot(x_c, y_c, "-r")

    if show_animation:
        plt.grid(True)
        plt.axis("equal")
        plt.show()


def main():
    planner.show_animation = show_animation
    uniform_terminal_state_sampling_test1()
    uniform_terminal_state_sampling_test2()
    biased_terminal_state_sampling_test1()
    biased_terminal_state_sampling_test2()
    lane_state_sampling_test1()


if __name__ == '__main__':
    main()
