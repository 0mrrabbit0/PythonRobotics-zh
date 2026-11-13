"""

用于 Hybrid A* 路径规划的车辆模型

作者: Zheng Zh (@Zhengzh)

"""

import sys
import pathlib
root_dir = pathlib.Path(__file__).parent.parent.parent
sys.path.append(str(root_dir))

from math import cos, sin, tan, pi

import matplotlib.pyplot as plt
import numpy as np

from utils.angle import rot_mat_2d

WB = 3.0  # 后轴到前轴
W = 2.0  # 车宽
LF = 3.3  # 后轴到车头的距离
LB = 1.0  # 后轴到车尾的距离
MAX_STEER = 0.6  # [rad] 最大转向角

BUBBLE_DIST = (LF - LB) / 2.0  # 后轴到车辆中心的距离。
BUBBLE_R = np.hypot((LF + LB) / 2.0, W / 2.0)  # 包围圆半径

# 车辆矩形顶点
VRX = [LF, LF, -LB, -LB, LF]
VRY = [W / 2, -W / 2, -W / 2, W / 2, W / 2]


def check_car_collision(x_list, y_list, yaw_list, ox, oy, kd_tree):
    for i_x, i_y, i_yaw in zip(x_list, y_list, yaw_list):
        cx = i_x + BUBBLE_DIST * cos(i_yaw)
        cy = i_y + BUBBLE_DIST * sin(i_yaw)

        ids = kd_tree.query_ball_point([cx, cy], BUBBLE_R)

        if not ids:
            continue

        if not rectangle_check(i_x, i_y, i_yaw,
                               [ox[i] for i in ids], [oy[i] for i in ids]):
            return False  # 碰撞

    return True  # 无碰撞


def rectangle_check(x, y, yaw, ox, oy):
    # 将障碍物变换到车体基坐标系
    rot = rot_mat_2d(yaw)
    for iox, ioy in zip(ox, oy):
        tx = iox - x
        ty = ioy - y
        converted_xy = np.stack([tx, ty]).T @ rot
        rx, ry = converted_xy[0], converted_xy[1]

        if not (rx > LF or rx < -LB or ry > W / 2.0 or ry < -W / 2.0):
            return False  # 碰撞

    return True  # 无碰撞


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """绘制箭头。"""
    if not isinstance(x, float):
        for (i_x, i_y, i_yaw) in zip(x, y, yaw):
            plot_arrow(i_x, i_y, i_yaw)
    else:
        plt.arrow(x, y, length * cos(yaw), length * sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width, alpha=0.4)


def plot_car(x, y, yaw):
    car_color = '-k'
    c, s = cos(yaw), sin(yaw)
    rot = rot_mat_2d(-yaw)
    car_outline_x, car_outline_y = [], []
    for rx, ry in zip(VRX, VRY):
        converted_xy = np.stack([rx, ry]).T @ rot
        car_outline_x.append(converted_xy[0]+x)
        car_outline_y.append(converted_xy[1]+y)

    arrow_x, arrow_y, arrow_yaw = c * 1.5 + x, s * 1.5 + y, yaw
    plot_arrow(arrow_x, arrow_y, arrow_yaw)

    plt.plot(car_outline_x, car_outline_y, car_color)


def pi_2_pi(angle):
    return (angle + pi) % (2 * pi) - pi


def move(x, y, yaw, distance, steer, L=WB):
    x += distance * cos(yaw)
    y += distance * sin(yaw)
    yaw = pi_2_pi(yaw + distance * tan(steer) / L)  # 距离/2

    return x, y, yaw


def main():
    x, y, yaw = 0., 0., 1.
    plt.axis('equal')
    plot_car(x, y, yaw)
    plt.show()


if __name__ == '__main__':
    main()
