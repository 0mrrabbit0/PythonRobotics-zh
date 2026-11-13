"""

移动到指定位姿

作者：Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai (@Atsushi_twi)
        Seied Muhammad Yazdian (@Muhammad-Yazdian)
        Wang Zheng (@Aglargil)

P. I. Corke, 《机器人学、视觉与控制》, Springer 2017, ISBN 978-3-319-54413-7

"""

import matplotlib.pyplot as plt
import numpy as np
import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
from utils.angle import angle_mod
from random import random


class PathFinderController:
    """
    构造 PathFinderController 的一个实例，用于在二维平面上引导
    三自由度轮式机器人

    Parameters
    ----------
    Kp_rho : 沿指向目标的直线平移机器人的线速度增益
    Kp_alpha : 使机器人转向目标的角速度增益
    Kp_beta : 用于平滑并入目标角度的偏置角速度增益
              （即帮助机器人航向与目标角度平行）
    """

    def __init__(self, Kp_rho, Kp_alpha, Kp_beta):
        self.Kp_rho = Kp_rho
        self.Kp_alpha = Kp_alpha
        self.Kp_beta = Kp_beta

    def calc_control_command(self, x_diff, y_diff, theta, theta_goal):
        """
        返回线速度与角速度的控制指令，以及到目标的距离

        Parameters
        ----------
        x_diff : 目标相对当前机器人位置在 x 方向上的位置
        y_diff : 目标相对当前机器人位置在 y 方向上的位置
        theta : 机器人相对 x 轴的当前航向角
        theta_goal: 机器人相对 x 轴的目标角度

        Returns
        -------
        rho : 机器人与目标位置之间的距离
        v : 指令线速度
        w : 指令角速度
        """

        # 局部变量说明：
        # - alpha 是目标相对机器人航向的角度
        # - beta 是机器人位置与目标位置之间的角度再加上目标角度
        # - Kp_rho*rho 与 Kp_alpha*alpha 驱动机器人沿直线驶向目标
        # - Kp_beta*beta 旋转该直线，使其与目标角度平行
        #
        # 注意：
        # 将 alpha 与 beta（角度差）限制在 [-pi, pi] 范围内，
        # 以避免不稳定行为，例如轻微转向时差值从 0 rad 跳到 2*pi rad

        # 速度 v 的符号始终保持不变，该符号取决于 α 的初值。
        rho = np.hypot(x_diff, y_diff)
        v = self.Kp_rho * rho

        alpha = angle_mod(np.arctan2(y_diff, x_diff) - theta)
        beta = angle_mod(theta_goal - theta - alpha)
        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            # 重新计算 alpha，使其落在 [-pi/2, pi/2] 范围内
            alpha = angle_mod(np.arctan2(-y_diff, -x_diff) - theta)
            beta = angle_mod(theta_goal - theta - alpha)
            w = self.Kp_alpha * alpha - self.Kp_beta * beta
            v = -v
        else:
            w = self.Kp_alpha * alpha - self.Kp_beta * beta

        return rho, v, w


# 仿真参数
controller = PathFinderController(9, 15, 3)
dt = 0.01
MAX_SIM_TIME = 5  # 秒，超过该时间后机器人停止运动

# 机器人规格
MAX_LINEAR_SPEED = 15
MAX_ANGULAR_SPEED = 7

show_animation = True


def move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal):
    x = x_start
    y = y_start
    theta = theta_start

    x_diff = x_goal - x
    y_diff = y_goal - y

    x_traj, y_traj, v_traj, w_traj = [x], [y], [0], [0]

    rho = np.hypot(x_diff, y_diff)
    t = 0
    while rho > 0.001 and t < MAX_SIM_TIME:
        t += dt
        x_traj.append(x)
        y_traj.append(y)

        x_diff = x_goal - x
        y_diff = y_goal - y

        rho, v, w = controller.calc_control_command(x_diff, y_diff, theta, theta_goal)

        if abs(v) > MAX_LINEAR_SPEED:
            v = np.sign(v) * MAX_LINEAR_SPEED

        if abs(w) > MAX_ANGULAR_SPEED:
            w = np.sign(w) * MAX_ANGULAR_SPEED

        v_traj.append(v)
        w_traj.append(w)

        theta = theta + w * dt
        x = x + v * np.cos(theta) * dt
        y = y + v * np.sin(theta) * dt

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.arrow(
                x_start,
                y_start,
                np.cos(theta_start),
                np.sin(theta_start),
                color="r",
                width=0.1,
            )
            plt.arrow(
                x_goal,
                y_goal,
                np.cos(theta_goal),
                np.sin(theta_goal),
                color="g",
                width=0.1,
            )
            plot_vehicle(x, y, theta, x_traj, y_traj)

    return x_traj, y_traj, v_traj, w_traj


def plot_vehicle(x, y, theta, x_traj, y_traj):  # pragma: no cover
    # 三角车辆朝右（0 弧度）时的顶点
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], "k-")
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], "k-")
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], "k-")

    plt.plot(x_traj, y_traj, "b--")

    # 按 esc 键停止仿真。
    plt.gcf().canvas.mpl_connect(
        "key_release_event", lambda event: [exit(0) if event.key == "escape" else None]
    )

    plt.xlim(0, 20)
    plt.ylim(0, 20)

    plt.pause(dt)


def transformation_matrix(x, y, theta):
    return np.array(
        [
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta), np.cos(theta), y],
            [0, 0, 1],
        ]
    )


def main():
    for i in range(5):
        x_start = 20.0 * random()
        y_start = 20.0 * random()
        theta_start: float = 2 * np.pi * random() - np.pi
        x_goal = 20 * random()
        y_goal = 20 * random()
        theta_goal = 2 * np.pi * random() - np.pi
        print(
            f"Initial x: {round(x_start, 2)} m\nInitial y: {round(y_start, 2)} m\nInitial theta: {round(theta_start, 2)} rad\n"
        )
        print(
            f"Goal x: {round(x_goal, 2)} m\nGoal y: {round(y_goal, 2)} m\nGoal theta: {round(theta_goal, 2)} rad\n"
        )
        move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)


if __name__ == "__main__":
    main()
