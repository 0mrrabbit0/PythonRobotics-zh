"""
两关节机械臂的逆运动学
在图上左键单击以设置末端执行器的目标位置

作者: Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai (@Atsushi_twi)

参考文献: P. I. Corke, "Robotics, Vision & Control", Springer 2017,
 ISBN 978-3-319-54413-7 p102
- [Robotics, Vision and Control]
(https://link.springer.com/book/10.1007/978-3-642-20144-8)

"""

import matplotlib.pyplot as plt
import numpy as np
import math
from utils.angle import angle_mod


# 仿真参数
Kp = 15
dt = 0.01

# 连杆长度
l1 = l2 = 1

# 将初始目标位置设为末端执行器的初始位置
x = 2
y = 0

show_animation = True

if show_animation:
    plt.ion()


def two_joint_arm(GOAL_TH=0.0, theta1=0.0, theta2=0.0):
    """
    计算平面二自由度机械臂的逆运动学
    超出工作空间时，用上一次的正确值改写 x 与 y
    """
    global x, y
    x_prev, y_prev = None, None
    while True:
        try:
            if x is not None and y is not None:
                x_prev = x
                y_prev = y
            if np.hypot(x, y) > (l1 + l2):
                theta2_goal = 0
            else:
                theta2_goal = np.arccos(
                    (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2))
            tmp = math.atan2(l2 * np.sin(theta2_goal),
                                (l1 + l2 * np.cos(theta2_goal)))
            theta1_goal = math.atan2(y, x) - tmp

            if theta1_goal < 0:
                theta2_goal = -theta2_goal
                tmp = math.atan2(l2 * np.sin(theta2_goal),
                                    (l1 + l2 * np.cos(theta2_goal)))
                theta1_goal = math.atan2(y, x) - tmp

            theta1 = theta1 + Kp * ang_diff(theta1_goal, theta1) * dt
            theta2 = theta2 + Kp * ang_diff(theta2_goal, theta2) * dt
        except ValueError as e:
            print("Unreachable goal"+e)
        except TypeError:
            x = x_prev
            y = y_prev

        wrist = plot_arm(theta1, theta2, x, y)

        # 检查目标
        d2goal = None
        if x is not None and y is not None:
            d2goal = np.hypot(wrist[0] - x, wrist[1] - y)

        if abs(d2goal) < GOAL_TH and x is not None:
            return theta1, theta2


def plot_arm(theta1, theta2, target_x, target_y):  # pragma: no cover
    shoulder = np.array([0, 0])
    elbow = shoulder + np.array([l1 * np.cos(theta1), l1 * np.sin(theta1)])
    wrist = elbow + \
        np.array([l2 * np.cos(theta1 + theta2), l2 * np.sin(theta1 + theta2)])

    if show_animation:
        plt.cla()

        plt.plot([shoulder[0], elbow[0]], [shoulder[1], elbow[1]], 'k-')
        plt.plot([elbow[0], wrist[0]], [elbow[1], wrist[1]], 'k-')

        plt.plot(shoulder[0], shoulder[1], 'ro')
        plt.plot(elbow[0], elbow[1], 'ro')
        plt.plot(wrist[0], wrist[1], 'ro')

        plt.plot([wrist[0], target_x], [wrist[1], target_y], 'g--')
        plt.plot(target_x, target_y, 'g*')

        plt.xlim(-2, 2)
        plt.ylim(-2, 2)

        plt.show()
        plt.pause(dt)

    return wrist


def ang_diff(theta1, theta2):
    # 返回两个角度之差，范围在 -pi 到 +pi
    return angle_mod(theta1 - theta2)


def click(event):  # pragma: no cover
    global x, y
    x = event.xdata
    y = event.ydata


def animation():
    from random import random
    global x, y
    theta1 = theta2 = 0.0
    for i in range(5):
        x = 2.0 * random() - 1.0
        y = 2.0 * random() - 1.0
        theta1, theta2 = two_joint_arm(
            GOAL_TH=0.01, theta1=theta1, theta2=theta2)


def main():  # pragma: no cover
    fig = plt.figure()
    fig.canvas.mpl_connect("button_press_event", click)
    # 按 Esc 键停止仿真。
    fig.canvas.mpl_connect('key_release_event', lambda event: [
                           exit(0) if event.key == 'escape' else None])
    two_joint_arm()


if __name__ == "__main__":
    # animation()
    main()
