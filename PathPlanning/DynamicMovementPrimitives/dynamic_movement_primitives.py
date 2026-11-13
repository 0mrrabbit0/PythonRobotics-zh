"""
作者: Jonathan Schwartz (github.com/SchwartzCode)

本代码提供动态运动原语（DMP）的一个简单实现。DMP 将曲线
建模为高斯分布的加权和，从而学习曲线。该方法可用于抑制
曲线中的噪声，也可通过调整起点与终点来拉伸曲线。

动态运动原语（DMP）的更多信息见：
https://arxiv.org/abs/2102.03861
https://www.frontiersin.org/journals/computational-neuroscience/articles/10.3389/fncom.2013.00138/full

"""


from matplotlib import pyplot as plt
import numpy as np


class DMP:

    def __init__(self, training_data, data_period, K=156.25, B=25):
        """
        Arguments:
            training_data - 形式为 [N, dim] 的输入数据
            data_period   - 训练数据所覆盖的时间长度
            K 与 B        - 定义 DMP 行为的弹簧与阻尼常数
        """

        self.K = K  # 虚拟弹簧常数
        self.B = B  # 虚拟阻尼系数

        self.timesteps = training_data.shape[0]
        self.dt = data_period / self.timesteps

        self.weights = None  # 用于生成 DMP 轨迹的权重

        self.T_orig = data_period

        self.training_data = training_data
        self.find_basis_functions_weights(training_data, data_period)

    def find_basis_functions_weights(self, training_data, data_period,
                                     num_weights=10):
        """
        Arguments:
            data [(步数 x 空间维数) np array] - 要用 DMP 复现的数据
            data_period [float] - 数据的时间长度
        """

        if not isinstance(training_data, np.ndarray):
            print("Warning: you should input training data as an np.ndarray")
        elif training_data.shape[0] < training_data.shape[1]:
            print("Warning: you probably need to transpose your training data")

        dt = data_period / len(training_data)

        init_state = training_data[0]
        goal_state = training_data[-1]

        # 高斯基函数的均值 (C) 与标准差 (H)
        C = np.linspace(0, 1, num_weights)
        H = (0.65*(1./(num_weights-1))**2)

        for dim, _ in enumerate(training_data[0]):

            dimension_data = training_data[:, dim]

            q0 = init_state[dim]
            g = goal_state[dim]

            q = q0
            qd_last = 0

            phi_vals = []
            f_vals = []

            for i, _ in enumerate(dimension_data):
                if i + 1 == len(dimension_data):
                    qd = 0
                else:
                    qd = (dimension_data[i+1] - dimension_data[i]) / dt

                phi = [np.exp(-0.5 * ((i * dt / data_period) - c)**2 / H)
                       for c in C]
                phi = phi/np.sum(phi)

                qdd = (qd - qd_last)/dt

                f = (qdd * data_period**2 - self.K * (g - q) + self.B * qd
                     * data_period) / (g - q0)

                phi_vals.append(phi)
                f_vals.append(f)

                qd_last = qd
                q += qd * dt

            phi_vals = np.asarray(phi_vals)
            f_vals = np.asarray(f_vals)

            w = np.linalg.lstsq(phi_vals, f_vals, rcond=None)

            if self.weights is None:
                self.weights = np.asarray(w[0])
            else:
                self.weights = np.vstack([self.weights, w[0]])

    def recreate_trajectory(self, init_state, goal_state, T):
        """
        init_state - 初始状态/位置
        goal_state - 目标状态/位置
        T  - 从 q0 运动到 g 所用的时间
        """

        nrBasis = len(self.weights[0])  # 高斯基函数的数量

        # 高斯基函数的均值 (C) 与标准差 (H)
        C = np.linspace(0, 1, nrBasis)
        H = (0.65*(1./(nrBasis-1))**2)

        # 初始化虚拟系统
        time = 0

        q = init_state
        dimensions = self.weights.shape[0]
        qd = np.zeros(dimensions)

        positions = np.array([])
        for k in range(self.timesteps):
            time = time + self.dt

            qdd = np.zeros(dimensions)

            for dim in range(dimensions):

                if time <= T:
                    phi = [np.exp(-0.5 * ((time / T) - c)**2 / H) for c in C]
                    phi = phi / np.sum(phi)
                    f = np.dot(phi, self.weights[dim])
                else:
                    f = 0

                # 仿真动力学
                qdd[dim] = (self.K*(goal_state[dim] - q[dim])/T**2
                            - self.B*qd[dim]/T
                            + (goal_state[dim] - init_state[dim])*f/T**2)

            qd = qd + qdd * self.dt
            q = q + qd * self.dt

            if positions.size == 0:
                positions = q
            else:
                positions = np.vstack([positions, q])

        t = np.arange(0, self.timesteps * self.dt, self.dt)
        return t, positions

    @staticmethod
    def dist_between(p1, p2):
        return np.linalg.norm(p1 - p2)

    def view_trajectory(self, path, title=None, demo=False):

        path = np.asarray(path)

        plt.cla()
        plt.plot(self.training_data[:, 0], self.training_data[:, 1],
                 label="Training Data")
        plt.plot(path[:, 0], path[:, 1],
                 linewidth=2, label="DMP Approximation")

        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.legend()

        if title is not None:
            plt.title(title)

        if demo:
            plt.xlim([-0.5, 5])
            plt.ylim([-2, 2])
            plt.draw()
            plt.pause(0.02)
        else:
            plt.show()

    def show_DMP_purpose(self):
        """
        本函数说明 DMP 的用途：
            捕获一条轨迹，并能够在起点与终点位置
            或时间上对其进行拉伸与压缩
        """

        q0_orig = self.training_data[0]
        g_orig = self.training_data[-1]
        T_orig = self.T_orig

        data_range = (np.amax(self.training_data[:, 0])
                      - np.amin(self.training_data[:, 0])) / 4

        q0_right = q0_orig + np.array([data_range, 0])
        q0_up = q0_orig + np.array([0, data_range/2])
        g_left = g_orig - np.array([data_range, 0])
        g_down = g_orig - np.array([0, data_range/2])

        q0_vals = np.vstack([np.linspace(q0_orig, q0_right, 20),
                             np.linspace(q0_orig, q0_up, 20)])
        g_vals = np.vstack([np.linspace(g_orig, g_left, 20),
                            np.linspace(g_orig, g_down, 20)])
        T_vals = np.linspace(T_orig, 2*T_orig, 20)

        for new_q0_value in q0_vals:
            plot_title = (f"Initial Position = [{round(new_q0_value[0], 2)},"
                          f" {round(new_q0_value[1], 2)}]")

            _, path = self.recreate_trajectory(new_q0_value, g_orig, T_orig)
            self.view_trajectory(path, title=plot_title, demo=True)

        for new_g_value in g_vals:
            plot_title = (f"Goal Position = [{round(new_g_value[0], 2)},"
                          f" {round(new_g_value[1], 2)}]")

            _, path = self.recreate_trajectory(q0_orig, new_g_value, T_orig)
            self.view_trajectory(path, title=plot_title, demo=True)

        for new_T_value in T_vals:
            plot_title = f"Period = {round(new_T_value, 2)} [sec]"

            _, path = self.recreate_trajectory(q0_orig, g_orig, new_T_value)
            self.view_trajectory(path, title=plot_title, demo=True)


def example_DMP():
    """
    生成一条带噪声的轨迹，拟合其权重，然后通过移动
    起点位置、目标点位置或周期来调整该轨迹
    """
    t = np.arange(0, 3*np.pi/2, 0.01)
    t1 = np.arange(3*np.pi/2, 2*np.pi, 0.01)[:-1]
    t2 = np.arange(0, np.pi/2, 0.01)[:-1]
    t3 = np.arange(np.pi, 3*np.pi/2, 0.01)
    data_x = t + 0.02*np.random.rand(t.shape[0])
    data_y = np.concatenate([np.cos(t1) + 0.1*np.random.rand(t1.shape[0]),
                             np.cos(t2) + 0.1*np.random.rand(t2.shape[0]),
                             np.sin(t3) + 0.1*np.random.rand(t3.shape[0])])
    training_data = np.vstack([data_x, data_y]).T

    period = 3*np.pi/2
    DMP_controller = DMP(training_data, period)

    DMP_controller.show_DMP_purpose()


if __name__ == '__main__':
    example_DMP()
