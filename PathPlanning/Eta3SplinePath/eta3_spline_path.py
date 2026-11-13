"""

eta^3 多项式规划器

作者：Joe Dinius 博士 (https://jwdinius.github.io)
        Atsushi Sakai (@Atsushi_twi)

Reference:
- [轮式移动机器人平滑路径生成的 eta^3 样条]
(https://ieeexplore.ieee.org/document/4339545/)

"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import quad

# 注意：*_pose 是长度为 3 的数组：
# 0 - x 坐标，1 - y 坐标，2 - 航向角 \theta

show_animation = True


class Eta3Path(object):
    """
    Eta3Path

    输入
        segments：由 `Eta3PathSegment` 实例组成的列表，
        定义一条连续路径
    """

    def __init__(self, segments):
        # 确保输入格式正确
        assert(isinstance(segments, list) and isinstance(
            segments[0], Eta3PathSegment))
        # 确保每段从上一段的终点开始（连续性）
        for r, s in zip(segments[:-1], segments[1:]):
            assert(np.array_equal(r.end_pose, s.start_pose))
        self.segments = segments

    def calc_path_point(self, u):
        """
        Eta3Path::calc_path_point

        输入
            沿路径对象的归一化插值点，0 <= u <= len(self.segments)
        返回
            二维 (x, y) 位置向量
        """

        assert(0 <= u <= len(self.segments))
        if np.isclose(u, len(self.segments)):
            segment_idx = len(self.segments) - 1
            u = 1.
        else:
            segment_idx = int(np.floor(u))
            u -= segment_idx
        return self.segments[segment_idx].calc_point(u)


class Eta3PathSegment(object):
    """
    Eta3PathSegment：根据期望的形状参数 eta 与曲率向量 kappa
    构造一段 eta^3 路径。若初始化时未设置 eta 或 kappa
    中的任意一个或两者，它们将默认为零。

    输入
        start_pose - 起始位姿数组 (x, y, \theta)
        end_pose - 终止位姿数组 (x, y, \theta)
        eta - 形状参数，默认 None
        kappa - 曲率参数，默认 None
    """

    def __init__(self, start_pose, end_pose, eta=None, kappa=None):
        # 确保输入尺寸正确
        assert(len(start_pose) == 3 and len(start_pose) == len(end_pose))
        self.start_pose = start_pose
        self.end_pose = end_pose
        # 若未传入 eta，则初始化为零数组
        if not eta:
            eta = np.zeros((6,))
        else:
            # 确保 eta 的长度正确
            assert(len(eta) == 6)
        # 若未传入 kappa，则初始化为零数组
        if not kappa:
            kappa = np.zeros((4,))
        else:
            assert(len(kappa) == 4)
        # 预先计算角度的正弦和余弦，以简化后续计算
        ca = np.cos(start_pose[2])
        sa = np.sin(start_pose[2])
        cb = np.cos(end_pose[2])
        sb = np.sin(end_pose[2])
        # 2 个维度 (x, y)，每个维度 8 个系数
        self.coeffs = np.empty((2, 8))
        # 常数项（u^0）
        self.coeffs[0, 0] = start_pose[0]
        self.coeffs[1, 0] = start_pose[1]
        # 一次项（u^1）
        self.coeffs[0, 1] = eta[0] * ca
        self.coeffs[1, 1] = eta[0] * sa
        # 二次项（u^2）
        self.coeffs[0, 2] = 1. / 2 * eta[2] * \
            ca - 1. / 2 * eta[0]**2 * kappa[0] * sa
        self.coeffs[1, 2] = 1. / 2 * eta[2] * \
            sa + 1. / 2 * eta[0]**2 * kappa[0] * ca
        # 三次项（u^3）
        self.coeffs[0, 3] = 1. / 6 * eta[4] * ca - 1. / 6 * \
            (eta[0]**3 * kappa[1] + 3. * eta[0] * eta[2] * kappa[0]) * sa
        self.coeffs[1, 3] = 1. / 6 * eta[4] * sa + 1. / 6 * \
            (eta[0]**3 * kappa[1] + 3. * eta[0] * eta[2] * kappa[0]) * ca
        # 四次项（u^4）
        tmp1 = 35. * (end_pose[0] - start_pose[0])
        tmp2 = (20. * eta[0] + 5 * eta[2] + 2. / 3 * eta[4]) * ca
        tmp3 = (5. * eta[0] ** 2 * kappa[0] + 2. / 3 * eta[0] ** 3 * kappa[1]
                + 2. * eta[0] * eta[2] * kappa[0]) * sa
        tmp4 = (15. * eta[1] - 5. / 2 * eta[3] + 1. / 6 * eta[5]) * cb
        tmp5 = (5. / 2 * eta[1] ** 2 * kappa[2] - 1. / 6 * eta[1] ** 3 *
                kappa[3] - 1. / 2 * eta[1] * eta[3] * kappa[2]) * sb
        self.coeffs[0, 4] = tmp1 - tmp2 + tmp3 - tmp4 - tmp5
        tmp1 = 35. * (end_pose[1] - start_pose[1])
        tmp2 = (20. * eta[0] + 5. * eta[2] + 2. / 3 * eta[4]) * sa
        tmp3 = (5. * eta[0] ** 2 * kappa[0] + 2. / 3 * eta[0] ** 3 * kappa[1]
                + 2. * eta[0] * eta[2] * kappa[0]) * ca
        tmp4 = (15. * eta[1] - 5. / 2 * eta[3] + 1. / 6 * eta[5]) * sb
        tmp5 = (5. / 2 * eta[1] ** 2 * kappa[2] - 1. / 6 * eta[1] ** 3 *
                kappa[3] - 1. / 2 * eta[1] * eta[3] * kappa[2]) * cb
        self.coeffs[1, 4] = tmp1 - tmp2 - tmp3 - tmp4 + tmp5
        # 五次项（u^5）
        tmp1 = -84. * (end_pose[0] - start_pose[0])
        tmp2 = (45. * eta[0] + 10. * eta[2] + eta[4]) * ca
        tmp3 = (10. * eta[0] ** 2 * kappa[0] + eta[0] ** 3 * kappa[1] + 3. *
                eta[0] * eta[2] * kappa[0]) * sa
        tmp4 = (39. * eta[1] - 7. * eta[3] + 1. / 2 * eta[5]) * cb
        tmp5 = + (7. * eta[1] ** 2 * kappa[2] - 1. / 2 * eta[1] ** 3 * kappa[3]
                  - 3. / 2 * eta[1] * eta[3] * kappa[2]) * sb
        self.coeffs[0, 5] = tmp1 + tmp2 - tmp3 + tmp4 + tmp5
        tmp1 = -84. * (end_pose[1] - start_pose[1])
        tmp2 = (45. * eta[0] + 10. * eta[2] + eta[4]) * sa
        tmp3 = (10. * eta[0] ** 2 * kappa[0] + eta[0] ** 3 * kappa[1] + 3. *
                eta[0] * eta[2] * kappa[0]) * ca
        tmp4 = (39. * eta[1] - 7. * eta[3] + 1. / 2 * eta[5]) * sb
        tmp5 = - (7. * eta[1] ** 2 * kappa[2] - 1. / 2 * eta[1] ** 3 * kappa[3]
                  - 3. / 2 * eta[1] * eta[3] * kappa[2]) * cb
        self.coeffs[1, 5] = tmp1 + tmp2 + tmp3 + tmp4 + tmp5
        # 六次项（u^6）
        tmp1 = 70. * (end_pose[0] - start_pose[0])
        tmp2 = (36. * eta[0] + 15. / 2 * eta[2] + 2. / 3 * eta[4]) * ca
        tmp3 = + (15. / 2 * eta[0] ** 2 * kappa[0] + 2. / 3 * eta[0] ** 3 *
                  kappa[1] + 2. * eta[0] * eta[2] * kappa[0]) * sa
        tmp4 = (34. * eta[1] - 13. / 2 * eta[3] + 1. / 2 * eta[5]) * cb
        tmp5 = - (13. / 2 * eta[1] ** 2 * kappa[2] - 1. / 2 * eta[1] ** 3 *
                  kappa[3] - 3. / 2 * eta[1] * eta[3] * kappa[2]) * sb
        self.coeffs[0, 6] = tmp1 - tmp2 + tmp3 - tmp4 + tmp5
        tmp1 = 70. * (end_pose[1] - start_pose[1])
        tmp2 = - (36. * eta[0] + 15. / 2 * eta[2] + 2. / 3 * eta[4]) * sa
        tmp3 = - (15. / 2 * eta[0] ** 2 * kappa[0] + 2. / 3 * eta[0] ** 3 *
                  kappa[1] + 2. * eta[0] * eta[2] * kappa[0]) * ca
        tmp4 = - (34. * eta[1] - 13. / 2 * eta[3] + 1. / 2 * eta[5]) * sb
        tmp5 = + (13. / 2 * eta[1] ** 2 * kappa[2] - 1. / 2 * eta[1] ** 3 *
                  kappa[3] - 3. / 2 * eta[1] * eta[3] * kappa[2]) * cb
        self.coeffs[1, 6] = tmp1 + tmp2 + tmp3 + tmp4 + tmp5
        # 七次项（u^7）
        tmp1 = -20. * (end_pose[0] - start_pose[0])
        tmp2 = (10. * eta[0] + 2. * eta[2] + 1. / 6 * eta[4]) * ca
        tmp3 = - (2. * eta[0] ** 2 * kappa[0] + 1. / 6 * eta[0] ** 3 * kappa[1]
                  + 1. / 2 * eta[0] * eta[2] * kappa[0]) * sa
        tmp4 = (10. * eta[1] - 2. * eta[3] + 1. / 6 * eta[5]) * cb
        tmp5 = (2. * eta[1] ** 2 * kappa[2] - 1. / 6 * eta[1] ** 3 * kappa[3]
                - 1. / 2 * eta[1] * eta[3] * kappa[2]) * sb
        self.coeffs[0, 7] = tmp1 + tmp2 + tmp3 + tmp4 + tmp5

        tmp1 = -20. * (end_pose[1] - start_pose[1])
        tmp2 = (10. * eta[0] + 2. * eta[2] + 1. / 6 * eta[4]) * sa
        tmp3 = (2. * eta[0] ** 2 * kappa[0] + 1. / 6 * eta[0] ** 3 * kappa[1]
                + 1. / 2 * eta[0] * eta[2] * kappa[0]) * ca
        tmp4 = (10. * eta[1] - 2. * eta[3] + 1. / 6 * eta[5]) * sb
        tmp5 = - (2. * eta[1] ** 2 * kappa[2] - 1. / 6 * eta[1] ** 3 * kappa[3]
                  - 1. / 2 * eta[1] * eta[3] * kappa[2]) * cb
        self.coeffs[1, 7] = tmp1 + tmp2 + tmp3 + tmp4 + tmp5
        self.s_dot = lambda u: max(np.linalg.norm(
            self.coeffs[:, 1:].dot(np.array(
                [1, 2. * u, 3. * u**2, 4. * u**3,
                 5. * u**4, 6. * u**5, 7. * u**6]))), 1e-6)
        self.f_length = lambda ue: quad(lambda u: self.s_dot(u), 0, ue)
        self.segment_length = self.f_length(1)[0]

    def calc_point(self, u):
        """
        Eta3PathSegment::calc_point

        输入
            u - 路段上一点的参数表示，0 <= u <= 1
        返回
            路段上该点的 (x, y)
        """
        assert(0 <= u <= 1)
        return self.coeffs.dot(np.array([1, u, u**2, u**3, u**4, u**5, u**6, u**7]))

    def calc_deriv(self, u, order=1):
        """
        Eta3PathSegment::calc_deriv

        输入
            u - 路段上一点的参数表示，0 <= u <= 1
        返回
            路段上该点的 (d^nx/du^n, d^ny/du^n)，其中 0 < n <= 2
        """

        assert(0 <= u <= 1)
        assert(0 < order <= 2)
        if order == 1:
            return self.coeffs[:, 1:].dot(np.array([1, 2. * u, 3. * u**2, 4. * u**3, 5. * u**4, 6. * u**5, 7. * u**6]))

        return self.coeffs[:, 2:].dot(np.array([2, 6. * u, 12. * u**2, 20. * u**3, 30. * u**4, 42. * u**5]))


def test1():

    for i in range(10):
        path_segments = []
        # 路段 1：换道曲线
        start_pose = [0, 0, 0]
        end_pose = [4, 3.0, 0]
        # 注意：kappa 的顺序为 [kappa_A, kappad_A, kappa_B, kappad_B]，其中 kappad_* 为曲率导数
        kappa = [0, 0, 0, 0]
        eta = [i, i, 0, 0, 0, 0]
        path_segments.append(Eta3PathSegment(
            start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

        path = Eta3Path(path_segments)

        # 沿路径在若干点处插值
        ui = np.linspace(0, len(path_segments), 1001)
        pos = np.empty((2, ui.size))
        for j, u in enumerate(ui):
            pos[:, j] = path.calc_path_point(u)

        if show_animation:
            # 绘制路径
            plt.plot(pos[0, :], pos[1, :])
            # 按 Esc 键停止仿真。
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.pause(1.0)

    if show_animation:
        plt.close("all")


def test2():

    for i in range(10):
        path_segments = []
        # 路段 1：换道曲线
        start_pose = [0, 0, 0]
        end_pose = [4, 3.0, 0]
        # 注意：kappa 的顺序为 [kappa_A, kappad_A, kappa_B, kappad_B]，其中 kappad_* 为曲率导数
        kappa = [0, 0, 0, 0]
        eta = [0, 0, (i - 5) * 20, (5 - i) * 20, 0, 0]
        path_segments.append(Eta3PathSegment(
            start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

        path = Eta3Path(path_segments)

        # 沿路径在若干点处插值
        ui = np.linspace(0, len(path_segments), 1001)
        pos = np.empty((2, ui.size))
        for j, u in enumerate(ui):
            pos[:, j] = path.calc_path_point(u)

        if show_animation:
            # 绘制路径
            plt.plot(pos[0, :], pos[1, :])
            plt.pause(1.0)

    if show_animation:
        plt.close("all")


def test3():
    path_segments = []

    # 路段 1：换道曲线
    start_pose = [0, 0, 0]
    end_pose = [4, 1.5, 0]
    # 注意：kappa 的顺序为 [kappa_A, kappad_A, kappa_B, kappad_B]，其中 kappad_* 为曲率导数
    kappa = [0, 0, 0, 0]
    eta = [4.27, 4.27, 0, 0, 0, 0]
    path_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # 路段 2：直线段
    start_pose = [4, 1.5, 0]
    end_pose = [5.5, 1.5, 0]
    kappa = [0, 0, 0, 0]
    eta = [0, 0, 0, 0, 0, 0]
    path_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # 路段 3：三次螺旋线
    start_pose = [5.5, 1.5, 0]
    end_pose = [7.4377, 1.8235, 0.6667]
    kappa = [0, 0, 1, 1]
    eta = [1.88, 1.88, 0, 0, 0, 0]
    path_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # 路段 4：一般扭转弧
    start_pose = [7.4377, 1.8235, 0.6667]
    end_pose = [7.8, 4.3, 1.8]
    kappa = [1, 1, 0.5, 0]
    eta = [7, 10, 10, -10, 4, 4]
    path_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # 路段 5：圆弧
    start_pose = [7.8, 4.3, 1.8]
    end_pose = [5.4581, 5.8064, 3.3416]
    kappa = [0.5, 0, 0.5, 0]
    eta = [2.98, 2.98, 0, 0, 0, 0]
    path_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # 构造整条路径
    path = Eta3Path(path_segments)

    # 沿路径在若干点处插值
    ui = np.linspace(0, len(path_segments), 1001)
    pos = np.empty((2, ui.size))
    for i, u in enumerate(ui):
        pos[:, i] = path.calc_path_point(u)

    # 绘制路径

    if show_animation:
        plt.figure('Path from Reference')
        plt.plot(pos[0, :], pos[1, :])
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('Path')
        plt.pause(1.0)

        plt.show()


def main():
    """
    复现参考文献中的路径（见表 1）
    """
    test1()
    test2()
    test3()


if __name__ == '__main__':
    main()
