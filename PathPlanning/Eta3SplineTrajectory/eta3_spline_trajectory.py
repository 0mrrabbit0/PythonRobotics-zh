"""

eta^3 多项式轨迹规划器

作者：Joe Dinius 博士 (https://jwdinius.github.io)
        Atsushi Sakai (@Atsushi_twi)

Refs:
- https://jwdinius.github.io/blog/2018/eta3traj/
- [轮式移动机器人平滑路径生成的 eta^3 样条]
(https://ieeexplore.ieee.org/document/4339545/)

"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from Eta3SplinePath.eta3_spline_path import Eta3Path, Eta3PathSegment

show_animation = True


class MaxVelocityNotReached(Exception):
    def __init__(self, actual_vel, max_vel):
        self.message = f'Actual velocity {actual_vel} does not equal desired max velocity {max_vel}!'


class Eta3SplineTrajectory(Eta3Path):
    """
    eta3_trajectory

    输入
        segments：定义连续轨迹的 `eta3_trajectory_segment` 实例列表
    """

    def __init__(self, segments, max_vel, v0=0.0, a0=0.0, max_accel=2.0, max_jerk=5.0):
        # 确保所有输入满足模型假设
        assert max_vel > 0 and v0 >= 0 and a0 >= 0 and max_accel > 0 and max_jerk > 0 \
            and a0 <= max_accel and v0 <= max_vel
        super(__class__, self).__init__(segments=segments)
        self.total_length = sum([s.segment_length for s in self.segments])
        self.max_vel = float(max_vel)
        self.v0 = float(v0)
        self.a0 = float(a0)
        self.max_accel = float(max_accel)
        self.max_jerk = float(max_jerk)
        length_array = np.array([s.segment_length for s in self.segments])
        # 在开头补一个零，以便找到正确的 segment_id
        self.cum_lengths = np.concatenate(
            (np.array([0]), np.cumsum(length_array)))
        # 在路径上计算速度剖面
        self.velocity_profile()
        self.ui_prev = 0
        self.prev_seg_id = 0

    def velocity_profile(self):
        r"""                  /~~~~~----------------\
                             /                       \
                            /                         \
                           /                           \
                          /                             \
        (v=v0, a=a0) ~~~~~                               \
                                                          \
                                                           \ ~~~~~ (vf=0, af=0)
                     正 |正 |负 |   以最大速度   |负 | 负 |负
                     最大|最大|最大|     巡航       |最大| 最大|最大
                   加加速度|加速度|加加速度|    速度    |加加速度|加速度|加加速度
            索引      0    1    2      3（可选）     4     5     6
        """
        # delta_a：从初始位置到最大加加速度段结束时的加速度变化
        delta_a = self.max_accel - self.a0
        # t_s1：通过最大加加速度段的时间
        t_s1 = delta_a / self.max_jerk
        # v_s1：最大加加速度段结束时的速度
        v_s1 = self.v0 + self.a0 * t_s1 + self.max_jerk * t_s1**2 / 2.
        # s_s1：最大加加速度段的长度
        s_s1 = self.v0 * t_s1 + self.a0 * t_s1**2 / 2. + self.max_jerk * t_s1**3 / 6.
        # t_sf：末段的通过时间，该段同样为最大加加速度，但末速度为 0
        t_sf = self.max_accel / self.max_jerk
        # v_sf：末段开始时的速度
        v_sf = self.max_jerk * t_sf**2 / 2.
        # s_sf：末段长度
        s_sf = self.max_jerk * t_sf**3 / 6.
        # 根据 max_accel 与 max_jerk 施加的运动学约束，求解可达的最大速度
        # 由此得到关于 v_max 的二次方程：a*v_max**2 + b*v_max + c = 0
        a = 1 / self.max_accel
        b = 3. * self.max_accel / (2. * self.max_jerk) + v_s1 / self.max_accel - (
            self.max_accel**2 / self.max_jerk + v_s1) / self.max_accel
        c = s_s1 + s_sf - self.total_length - 7. * self.max_accel**3 / (3. * self.max_jerk**2) \
            - v_s1 * (self.max_accel / self.max_jerk + v_s1 / self.max_accel) \
            + (self.max_accel**2 / self.max_jerk + v_s1 /
               self.max_accel)**2 / (2. * self.max_accel)
        v_max = (-b + np.sqrt(b**2 - 4. * a * c)) / (2. * a)

        # v_max 表示若不存在巡航段时所能达到的最大速度
        # （即以恒定速度行驶，不再加速或加加速）
        # 若该速度小于期望的最大速度，则需要更新最大速度
        if self.max_vel > v_max:
            # 触发该条件时将没有巡航段（s_cruise=0）
            self.max_vel = v_max

        # 建立数组，存储各轨迹段结束时的值
        self.times = np.zeros((7,))
        self.vels = np.zeros((7,))
        self.seg_lengths = np.zeros((7,))

        # 第 0 段：以最大加加速度加速到最大加速度
        self.times[0] = t_s1
        self.vels[0] = v_s1
        self.seg_lengths[0] = s_s1

        # 第 1 段：以 max_accel 加速
        index = 1
        # 计算该段内的速度变化
        delta_v = (self.max_vel - self.max_jerk * (self.max_accel /
                                                   self.max_jerk)**2 / 2.) - self.vels[index - 1]
        self.times[index] = delta_v / self.max_accel
        self.vels[index] = self.vels[index - 1] + \
            self.max_accel * self.times[index]
        self.seg_lengths[index] = self.vels[index - 1] * \
            self.times[index] + self.max_accel * self.times[index]**2 / 2.

        # 第 2 段：减小加速度（直至 0），直到达到最大速度
        index = 2
        self.times[index] = self.max_accel / self.max_jerk
        self.vels[index] = self.vels[index - 1] + self.max_accel * self.times[index] \
            - self.max_jerk * self.times[index]**2 / 2.

        # 作为校验，该段结束时的速度应为 self.max_vel
        if not np.isclose(self.vels[index], self.max_vel):
            raise MaxVelocityNotReached(self.vels[index], self.max_vel)

        self.seg_lengths[index] = self.vels[index - 1] * self.times[index] + self.max_accel * self.times[index]**2 / 2. \
            - self.max_jerk * self.times[index]**3 / 6.

        # 第 3 段：最后处理

        # 第 4 段：施加最小加加速度，直到达到最小加速度
        index = 4
        self.times[index] = self.max_accel / self.max_jerk
        self.vels[index] = self.max_vel - \
            self.max_jerk * self.times[index]**2 / 2.
        self.seg_lengths[index] = self.max_vel * self.times[index] - \
            self.max_jerk * self.times[index]**3 / 6.

        # 第 5 段：以最大减速度继续减速
        index = 5
        # 计算各段上的速度变化
        delta_v = self.vels[index - 1] - v_sf
        self.times[index] = delta_v / self.max_accel
        self.vels[index] = self.vels[index - 1] - \
            self.max_accel * self.times[index]
        self.seg_lengths[index] = self.vels[index - 1] * \
            self.times[index] - self.max_accel * self.times[index]**2 / 2.

        # 第 6 段（末段）：以最大加加速度同时到达零速度和零加速度
        index = 6
        self.times[index] = t_sf
        self.vels[index] = self.vels[index - 1] - self.max_jerk * t_sf**2 / 2.

        try:
            assert np.isclose(self.vels[index], 0)
        except AssertionError as e:
            print(f'The final velocity {self.vels[index]} is not zero')
            raise e

        self.seg_lengths[index] = s_sf

        if self.seg_lengths.sum() < self.total_length:
            index = 3
            # 巡航段长度是尚未计入的剩余长度
            # 注意：对整个 self.seg_lengths 求和，是因为巡航段对应项
            # 被初始化为 0！
            self.seg_lengths[index] = self.total_length - \
                self.seg_lengths.sum()
            self.vels[index] = self.max_vel
            self.times[index] = self.seg_lengths[index] / self.max_vel

        # 确保所有时间均为正，否则所选运动学约束
        # 无法在该路径上实现
        assert(np.all(self.times >= 0))
        self.total_time = self.times.sum()

    def get_interp_param(self, seg_id, s, ui, tol=0.001):
        def f(u):
            return self.segments[seg_id].f_length(u)[0] - s

        def fprime(u):
            return self.segments[seg_id].s_dot(u)
        while (0 <= ui <= 1) and abs(f(ui)) > tol:
            ui -= f(ui) / fprime(ui)
        ui = max(0, min(ui, 1))
        return ui

    def calc_traj_point(self, time):
        # 计算该时刻的速度
        if time <= self.times[0]:
            linear_velocity = self.v0 + self.max_jerk * time**2 / 2.
            s = self.v0 * time + self.max_jerk * time**3 / 6
            linear_accel = self.max_jerk * time
        elif time <= self.times[:2].sum():
            delta_t = time - self.times[0]
            linear_velocity = self.vels[0] + self.max_accel * delta_t
            s = self.seg_lengths[0] + self.vels[0] * \
                delta_t + self.max_accel * delta_t**2 / 2.
            linear_accel = self.max_accel
        elif time <= self.times[:3].sum():
            delta_t = time - self.times[:2].sum()
            linear_velocity = self.vels[1] + self.max_accel * \
                delta_t - self.max_jerk * delta_t**2 / 2.
            s = self.seg_lengths[:2].sum() + self.vels[1] * delta_t + self.max_accel * delta_t**2 / 2. \
                - self.max_jerk * delta_t**3 / 6.
            linear_accel = self.max_accel - self.max_jerk * delta_t
        elif time <= self.times[:4].sum():
            delta_t = time - self.times[:3].sum()
            linear_velocity = self.vels[3]
            s = self.seg_lengths[:3].sum() + self.vels[3] * delta_t
            linear_accel = 0.
        elif time <= self.times[:5].sum():
            delta_t = time - self.times[:4].sum()
            linear_velocity = self.vels[3] - self.max_jerk * delta_t**2 / 2.
            s = self.seg_lengths[:4].sum() + self.vels[3] * \
                delta_t - self.max_jerk * delta_t**3 / 6.
            linear_accel = -self.max_jerk * delta_t
        elif time <= self.times[:-1].sum():
            delta_t = time - self.times[:5].sum()
            linear_velocity = self.vels[4] - self.max_accel * delta_t
            s = self.seg_lengths[:5].sum() + self.vels[4] * \
                delta_t - self.max_accel * delta_t**2 / 2.
            linear_accel = -self.max_accel
        elif time < self.times.sum():
            delta_t = time - self.times[:-1].sum()
            linear_velocity = self.vels[5] - self.max_accel * \
                delta_t + self.max_jerk * delta_t**2 / 2.
            s = self.seg_lengths[:-1].sum() + self.vels[5] * delta_t - self.max_accel * delta_t**2 / 2. \
                + self.max_jerk * delta_t**3 / 6.
            linear_accel = -self.max_accel + self.max_jerk * delta_t
        else:
            linear_velocity = 0.
            s = self.total_length
            linear_accel = 0.
        seg_id = np.max(np.argwhere(self.cum_lengths <= s))

        # 会发生在路段末端
        if seg_id == len(self.segments):
            seg_id -= 1
            ui = 1
        else:
            # 用从当前路段起点起的长度计算插值参数
            curr_segment_length = s - self.cum_lengths[seg_id]
            ui = self.get_interp_param(
                seg_id=seg_id, s=curr_segment_length, ui=self.ui_prev)

        if not seg_id == self.prev_seg_id:
            self.ui_prev = 0
        else:
            self.ui_prev = ui

        self.prev_seg_id = seg_id
        # 计算当前点的角速度 = (ydd*xd - xdd*yd) / (xd**2 + yd**2)
        d = self.segments[seg_id].calc_deriv(ui, order=1)
        dd = self.segments[seg_id].calc_deriv(ui, order=2)
        # su：弧长对 u 的变化率
        su = self.segments[seg_id].s_dot(ui)
        if not np.isclose(su, 0.) and not np.isclose(linear_velocity, 0.):
            # ut：插值参数 u 的时间导数
            ut = linear_velocity / su
            # utt：ut 的时间导数
            utt = linear_accel / su - \
                (d[0] * dd[0] + d[1] * dd[1]) / su**2 * ut
            xt = d[0] * ut
            yt = d[1] * ut
            xtt = dd[0] * ut**2 + d[0] * utt
            ytt = dd[1] * ut**2 + d[1] * utt
            angular_velocity = (ytt * xt - xtt * yt) / linear_velocity**2
        else:
            angular_velocity = 0.

        # 将路径点与航向及速度组合
        pos = self.segments[seg_id].calc_point(ui)
        state = np.array([pos[0], pos[1], np.arctan2(
            d[1], d[0]), linear_velocity, angular_velocity])
        return state


def test1(max_vel=0.5):

    for i in range(10):
        trajectory_segments = []
        # 路段 1：换道曲线
        start_pose = [0, 0, 0]
        end_pose = [4, 3.0, 0]
        # 注意：kappa 的顺序为 [kappa_A, kappad_A, kappa_B, kappad_B]，其中 kappad_* 为曲率导数
        kappa = [0, 0, 0, 0]
        eta = [i, i, 0, 0, 0, 0]
        trajectory_segments.append(Eta3PathSegment(
            start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

        traj = Eta3SplineTrajectory(trajectory_segments,
                                    max_vel=max_vel, max_accel=0.5)

        # 沿路径在若干点处插值
        times = np.linspace(0, traj.total_time, 101)
        state = np.empty((5, times.size))
        for j, t in enumerate(times):
            state[:, j] = traj.calc_traj_point(t)

        if show_animation:  # pragma: no cover
            # 绘制路径
            plt.plot(state[0, :], state[1, :])
            plt.pause(1.0)

    plt.show()
    if show_animation:
        plt.close("all")


def test2(max_vel=0.5):

    for i in range(10):
        trajectory_segments = []
        # 路段 1：换道曲线
        start_pose = [0, 0, 0]
        end_pose = [4, 3.0, 0]
        # 注意：kappa 的顺序为 [kappa_A, kappad_A, kappa_B, kappad_B]，其中 kappad_* 为曲率导数
        kappa = [0, 0, 0, 0]
        # 注意：当 eta[:1] 为零时积分误差会爆炸！
        # 原为：eta = [0, 0, (i - 5) * 20, (5 - i) * 20, 0, 0]，现为：
        eta = [0.1, 0.1, (i - 5) * 20, (5 - i) * 20, 0, 0]
        trajectory_segments.append(Eta3PathSegment(
            start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

        traj = Eta3SplineTrajectory(trajectory_segments,
                                    max_vel=max_vel, max_accel=0.5)

        # 沿路径在若干点处插值
        times = np.linspace(0, traj.total_time, 101)
        state = np.empty((5, times.size))
        for j, t in enumerate(times):
            state[:, j] = traj.calc_traj_point(t)

        if show_animation:
            # 绘制路径
            plt.plot(state[0, :], state[1, :])
            plt.pause(1.0)

    plt.show()
    if show_animation:
        plt.close("all")


def test3(max_vel=2.0):
    trajectory_segments = []

    # 路段 1：换道曲线
    start_pose = [0, 0, 0]
    end_pose = [4, 1.5, 0]
    # 注意：kappa 的顺序为 [kappa_A, kappad_A, kappa_B, kappad_B]，其中 kappad_* 为曲率导数
    kappa = [0, 0, 0, 0]
    eta = [4.27, 4.27, 0, 0, 0, 0]
    trajectory_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # 路段 2：直线段
    start_pose = [4, 1.5, 0]
    end_pose = [5.5, 1.5, 0]
    kappa = [0, 0, 0, 0]
    # 注意：当 eta[:1] 为零时积分误差会爆炸！
    # 原为：eta = [0, 0, 0, 0, 0, 0]，现为：
    eta = [0.5, 0.5, 0, 0, 0, 0]
    trajectory_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # 路段 3：三次螺旋线
    start_pose = [5.5, 1.5, 0]
    end_pose = [7.4377, 1.8235, 0.6667]
    kappa = [0, 0, 1, 1]
    eta = [1.88, 1.88, 0, 0, 0, 0]
    trajectory_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # 路段 4：一般扭转弧
    start_pose = [7.4377, 1.8235, 0.6667]
    end_pose = [7.8, 4.3, 1.8]
    kappa = [1, 1, 0.5, 0]
    eta = [7, 10, 10, -10, 4, 4]
    trajectory_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # 路段 5：圆弧
    start_pose = [7.8, 4.3, 1.8]
    end_pose = [5.4581, 5.8064, 3.3416]
    kappa = [0.5, 0, 0.5, 0]
    eta = [2.98, 2.98, 0, 0, 0, 0]
    trajectory_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # 构造整条轨迹
    traj = Eta3SplineTrajectory(trajectory_segments,
                                max_vel=max_vel, max_accel=0.5, max_jerk=1)

    # 沿轨迹在若干点处插值
    times = np.linspace(0, traj.total_time, 1001)
    state = np.empty((5, times.size))
    for i, t in enumerate(times):
        state[:, i] = traj.calc_traj_point(t)

    # 绘制路径

    if show_animation:
        fig, ax = plt.subplots()
        x, y = state[0, :], state[1, :]
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segs = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segs, cmap=plt.get_cmap('inferno'))
        ax.set_xlim(np.min(x) - 1, np.max(x) + 1)
        ax.set_ylim(np.min(y) - 1, np.max(y) + 1)
        lc.set_array(state[3, :])
        lc.set_linewidth(3)
        ax.add_collection(lc)
        axcb = fig.colorbar(lc)
        axcb.set_label('velocity(m/s)')
        ax.set_title('Trajectory')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.pause(1.0)

        fig1, ax1 = plt.subplots()
        ax1.plot(times, state[3, :], 'b-')
        ax1.set_xlabel('time(s)')
        ax1.set_ylabel('velocity(m/s)', color='b')
        ax1.tick_params('y', colors='b')
        ax1.set_title('Control')
        ax2 = ax1.twinx()
        ax2.plot(times, state[4, :], 'r-')
        ax2.set_ylabel('angular velocity(rad/s)', color='r')
        ax2.tick_params('y', colors='r')
        fig.tight_layout()
        plt.show()


def main():
    """
    复现参考文献中的路径（见表 1）
    """
    # test1()
    # test2()
    test3()


if __name__ == '__main__':
    main()
