"""

移动到指定位姿（使用 Robot 类）

作者：Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai (@Atsushi_twi)
        Seied Muhammad Yazdian (@Muhammad-Yazdian)

P.I. Corke, 《机器人学、视觉与控制》, Springer 2017, ISBN 978-3-319-54413-7

"""

import matplotlib.pyplot as plt
import numpy as np
import copy
from move_to_pose import PathFinderController

# 仿真参数
TIME_DURATION = 1000
TIME_STEP = 0.01
AT_TARGET_ACCEPTANCE_THRESHOLD = 0.01
SHOW_ANIMATION = True
PLOT_WINDOW_SIZE_X = 20
PLOT_WINDOW_SIZE_Y = 20
PLOT_FONT_SIZE = 8

simulation_running = True
all_robots_are_at_target = False


class Pose:
    """二维位姿"""

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


class Robot:
    """
    构造在二维平面上导航的三自由度轮式 Robot 的一个实例

    Parameters
    ----------
    name : (字符串)
        机器人名称
    color : (字符串)
        机器人颜色
    max_linear_speed : (float)
        机器人能够达到的最大线速率
    max_angular_speed : (float)
        机器人绕其竖直轴旋转的最大角速率
    path_finder_controller : (PathFinderController)
        可配置的控制器，用于寻找路径并计算指令线速度与角速度。
    """

    def __init__(self, name, color, max_linear_speed, max_angular_speed,
                 path_finder_controller):
        self.name = name
        self.color = color
        self.MAX_LINEAR_SPEED = max_linear_speed
        self.MAX_ANGULAR_SPEED = max_angular_speed
        self.path_finder_controller = path_finder_controller
        self.x_traj = []
        self.y_traj = []
        self.pose = Pose(0, 0, 0)
        self.pose_start = Pose(0, 0, 0)
        self.pose_target = Pose(0, 0, 0)
        self.is_at_target = False

    def set_start_target_poses(self, pose_start, pose_target):
        """
        设置机器人的起始位姿与目标位姿

        Parameters
        ----------
        pose_start : (Pose)
            机器人的起始位姿（参见 Pose 类）
        pose_target : (Pose)
            机器人的目标位姿（参见 Pose 类）
        """
        self.pose_start = copy.copy(pose_start)
        self.pose_target = pose_target
        self.pose = pose_start

    def move(self, dt):
        """
        将机器人推进一个时间步

        Parameters
        ----------
        dt : (float)
            时间步长
        """
        self.x_traj.append(self.pose.x)
        self.y_traj.append(self.pose.y)

        rho, linear_velocity, angular_velocity = \
            self.path_finder_controller.calc_control_command(
                self.pose_target.x - self.pose.x,
                self.pose_target.y - self.pose.y,
                self.pose.theta, self.pose_target.theta)

        if rho < AT_TARGET_ACCEPTANCE_THRESHOLD:
            self.is_at_target = True

        if abs(linear_velocity) > self.MAX_LINEAR_SPEED:
            linear_velocity = (np.sign(linear_velocity)
                               * self.MAX_LINEAR_SPEED)

        if abs(angular_velocity) > self.MAX_ANGULAR_SPEED:
            angular_velocity = (np.sign(angular_velocity)
                                * self.MAX_ANGULAR_SPEED)

        self.pose.theta = self.pose.theta + angular_velocity * dt
        self.pose.x = self.pose.x + linear_velocity * \
            np.cos(self.pose.theta) * dt
        self.pose.y = self.pose.y + linear_velocity * \
            np.sin(self.pose.theta) * dt


def run_simulation(robots):
    """同时仿真所有机器人"""
    global all_robots_are_at_target
    global simulation_running

    robot_names = []
    for instance in robots:
        robot_names.append(instance.name)

    time = 0
    while simulation_running and time < TIME_DURATION:
        time += TIME_STEP
        robots_are_at_target = []

        for instance in robots:
            if not instance.is_at_target:
                instance.move(TIME_STEP)
            robots_are_at_target.append(instance.is_at_target)

        if all(robots_are_at_target):
            simulation_running = False

        if SHOW_ANIMATION:
            plt.cla()
            plt.xlim(0, PLOT_WINDOW_SIZE_X)
            plt.ylim(0, PLOT_WINDOW_SIZE_Y)

            # 按 esc 键停止仿真。
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            plt.text(0.3, PLOT_WINDOW_SIZE_Y - 1,
                     'Time: {:.2f}'.format(time),
                     fontsize=PLOT_FONT_SIZE)

            plt.text(0.3, PLOT_WINDOW_SIZE_Y - 2,
                     'Reached target: {} = '.format(robot_names)
                     + str(robots_are_at_target),
                     fontsize=PLOT_FONT_SIZE)

            for instance in robots:
                plt.arrow(instance.pose_start.x,
                          instance.pose_start.y,
                          np.cos(instance.pose_start.theta),
                          np.sin(instance.pose_start.theta),
                          color='r',
                          width=0.1)
                plt.arrow(instance.pose_target.x,
                          instance.pose_target.y,
                          np.cos(instance.pose_target.theta),
                          np.sin(instance.pose_target.theta),
                          color='g',
                          width=0.1)
                plot_vehicle(instance.pose.x,
                             instance.pose.y,
                             instance.pose.theta,
                             instance.x_traj,
                             instance.y_traj, instance.color)

            plt.pause(TIME_STEP)


def plot_vehicle(x, y, theta, x_traj, y_traj, color):
    # 三角车辆朝右（0 弧度）时的顶点
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = T @ p1_i
    p2 = T @ p2_i
    p3 = T @ p3_i

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color+'-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], color+'-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], color+'-')

    plt.plot(x_traj, y_traj, color+'--')


def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])


def main():
    pose_target = Pose(15, 15, -1)

    pose_start_1 = Pose(5, 2, 0)
    pose_start_2 = Pose(5, 2, 0)
    pose_start_3 = Pose(5, 2, 0)

    controller_1 = PathFinderController(5, 8, 2)
    controller_2 = PathFinderController(5, 16, 4)
    controller_3 = PathFinderController(10, 25, 6)

    robot_1 = Robot("Yellow Robot", "y", 12, 5, controller_1)
    robot_2 = Robot("Black Robot", "k", 16, 5, controller_2)
    robot_3 = Robot("Blue Robot", "b", 20, 5, controller_3)

    robot_1.set_start_target_poses(pose_start_1, pose_target)
    robot_2.set_start_target_poses(pose_start_2, pose_target)
    robot_3.set_start_target_poses(pose_start_3, pose_target)

    robots: list[Robot] = [robot_1, robot_2, robot_3]

    run_simulation(robots)


if __name__ == '__main__':
    main()
