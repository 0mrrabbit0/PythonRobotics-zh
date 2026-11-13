"""
通过修正指定落脚点实现双足行走
作者: Takayuki Murooka (takayuki5168)
"""
import numpy as np
import math
from matplotlib import pyplot as plt
import matplotlib.patches as pat
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d


class BipedalPlanner(object):
    def __init__(self):
        self.act_p = []  # 实际落脚点位置
        self.ref_p = []  # 参考落脚点位置
        self.com_trajectory = []
        self.ref_footsteps = None
        self.g = 9.8

    def set_ref_footsteps(self, ref_footsteps):
        self.ref_footsteps = ref_footsteps

    def inverted_pendulum(self, x, x_dot, px_star, y, y_dot, py_star, z_c,
                          time_width):
        time_split = 100

        for i in range(time_split):
            delta_time = time_width / time_split

            x_dot2 = self.g / z_c * (x - px_star)
            x += x_dot * delta_time
            x_dot += x_dot2 * delta_time

            y_dot2 = self.g / z_c * (y - py_star)
            y += y_dot * delta_time
            y_dot += y_dot2 * delta_time

            if i % 10 == 0:
                self.com_trajectory.append([x, y])

        return x, x_dot, y, y_dot

    def walk(self, t_sup=0.8, z_c=0.8, a=10, b=1, plot=False):
        if self.ref_footsteps is None:
            print("No footsteps")
            return

        # 设置绘图器
        if plot:
            fig = plt.figure()
            ax = Axes3D(fig)
            fig.add_axes(ax)
            com_trajectory_for_plot = []

        px, py = 0.0, 0.0  # 参考落脚点位置
        px_star, py_star = px, py  # 修正后的落脚点位置
        xi, xi_dot, yi, yi_dot = 0.0, 0.0, 0.01, 0.0
        time = 0.0
        n = 0
        self.ref_p.append([px, py, 0])
        self.act_p.append([px, py, 0])
        for i in range(len(self.ref_footsteps)):
            # 仿真倒立摆的 x、y 及其导数
            xi, xi_dot, yi, yi_dot = self.inverted_pendulum(
                xi, xi_dot, px_star, yi, yi_dot, py_star, z_c, t_sup)

            # 更新时间
            time += t_sup
            n += 1

            # 计算 px、py、x_、y_、vx_、vy_
            f_x, f_y, f_theta = self.ref_footsteps[n - 1]
            rotate_mat = np.array([[math.cos(f_theta), -math.sin(f_theta)],
                                   [math.sin(f_theta), math.cos(f_theta)]])

            if n == len(self.ref_footsteps):
                f_x_next, f_y_next, f_theta_next = 0., 0., 0.
            else:
                f_x_next, f_y_next, f_theta_next = self.ref_footsteps[n]
            rotate_mat_next = np.array(
                [[math.cos(f_theta_next), -math.sin(f_theta_next)],
                 [math.sin(f_theta_next), math.cos(f_theta_next)]])

            Tc = math.sqrt(z_c / self.g)
            C = math.cosh(t_sup / Tc)
            S = math.sinh(t_sup / Tc)

            px, py = list(np.array([px, py])
                          + np.dot(rotate_mat,
                                   np.array([f_x, -1 * math.pow(-1, n) * f_y])
                                   ))
            x_, y_ = list(np.dot(rotate_mat_next, np.array(
                [f_x_next / 2., math.pow(-1, n) * f_y_next / 2.])))
            vx_, vy_ = list(np.dot(rotate_mat_next, np.array(
                [(1 + C) / (Tc * S) * x_, (C - 1) / (Tc * S) * y_])))
            self.ref_p.append([px, py, f_theta])

            # 计算参考质心
            xd, xd_dot = px + x_, vx_
            yd, yd_dot = py + y_, vy_

            # 计算修正后的落脚点
            D = a * math.pow(C - 1, 2) + b * math.pow(S / Tc, 2)
            px_star = -a * (C - 1) / D * (xd - C * xi - Tc * S * xi_dot) \
                      - b * S / (Tc * D) * (xd_dot - S / Tc * xi - C * xi_dot)
            py_star = -a * (C - 1) / D * (yd - C * yi - Tc * S * yi_dot) \
                      - b * S / (Tc * D) * (yd_dot - S / Tc * yi - C * yi_dot)
            self.act_p.append([px_star, py_star, f_theta])

            # 绘图
            if plot:
                self.plot_animation(ax, com_trajectory_for_plot, px_star,
                                    py_star, z_c)
        if plot:
            plt.show()

    def plot_animation(self, ax, com_trajectory_for_plot, px_star, py_star,
                       z_c):  # pragma: no cover
        # 为绘制轨迹，在循环中绘图
        for c in range(len(self.com_trajectory)):
            if c > len(com_trajectory_for_plot):
                # 设置绘图器
                plt.cla()
                # 按 Esc 键停止仿真。
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event:
                    [exit(0) if event.key == 'escape' else None])
                ax.set_zlim(0, z_c * 2)
                ax.set_xlim(0, 1)
                ax.set_ylim(-0.5, 0.5)

                # 更新用于绘图的质心轨迹
                com_trajectory_for_plot.append(self.com_trajectory[c])

                # 绘制质心
                ax.plot([p[0] for p in com_trajectory_for_plot],
                        [p[1] for p in com_trajectory_for_plot], [
                            0 for _ in com_trajectory_for_plot],
                        color="red")

                # 绘制倒立摆
                ax.plot([px_star, com_trajectory_for_plot[-1][0]],
                        [py_star, com_trajectory_for_plot[-1][1]],
                        [0, z_c], color="green", linewidth=3)
                ax.scatter([com_trajectory_for_plot[-1][0]],
                           [com_trajectory_for_plot[-1][1]],
                           [z_c], color="green", s=300)

                # self.ref_p 的足底矩形
                foot_width = 0.06
                foot_height = 0.04
                for j in range(len(self.ref_p)):
                    angle = self.ref_p[j][2] + \
                            math.atan2(foot_height,
                                       foot_width) - math.pi
                    r = math.sqrt(
                        math.pow(foot_width / 3., 2) + math.pow(
                            foot_height / 2., 2))
                    rec = pat.Rectangle(xy=(
                        self.ref_p[j][0] + r * math.cos(angle),
                        self.ref_p[j][1] + r * math.sin(angle)),
                        width=foot_width,
                        height=foot_height,
                        angle=self.ref_p[j][
                                  2] * 180 / math.pi,
                        color="blue", fill=False,
                        ls=":")
                    ax.add_patch(rec)
                    art3d.pathpatch_2d_to_3d(rec, z=0, zdir="z")

                # self.act_p 的足底矩形
                for j in range(len(self.act_p)):
                    angle = self.act_p[j][2] + \
                            math.atan2(foot_height,
                                       foot_width) - math.pi
                    r = math.sqrt(
                        math.pow(foot_width / 3., 2) + math.pow(
                            foot_height / 2., 2))
                    rec = pat.Rectangle(xy=(
                        self.act_p[j][0] + r * math.cos(angle),
                        self.act_p[j][1] + r * math.sin(angle)),
                        width=foot_width,
                        height=foot_height,
                        angle=self.act_p[j][
                                  2] * 180 / math.pi,
                        color="blue", fill=False)
                    ax.add_patch(rec)
                    art3d.pathpatch_2d_to_3d(rec, z=0, zdir="z")

                plt.draw()
                plt.pause(0.001)


if __name__ == "__main__":
    bipedal_planner = BipedalPlanner()

    footsteps = [[0.0, 0.2, 0.0],
                 [0.3, 0.2, 0.0],
                 [0.3, 0.2, 0.2],
                 [0.3, 0.2, 0.2],
                 [0.0, 0.2, 0.2]]
    bipedal_planner.set_ref_footsteps(footsteps)
    bipedal_planner.walk(plot=True)
