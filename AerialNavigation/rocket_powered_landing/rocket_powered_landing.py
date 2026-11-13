"""

用逐次凸化求解火箭动力着陆

作者: Sven Niederberger
        Atsushi Sakai

参考文献:
- 论文 'Successive Convexification for 6-DoF Mars Rocket Powered Landing with Free-Final-Time' 的 Python 实现
作者为 Michael Szmuk 与 Behcet Acıkmese。

- EmbersArc/SuccessiveConvexificationFreeFinalTime：对 "Successive Convexification for 6-DoF Mars Rocket Powered Landing with Free-Final-Time" 的实现 https://github.com/EmbersArc/SuccessiveConvexificationFreeFinalTime

"""
import warnings
from time import time
import numpy as np
from scipy.integrate import odeint
import cvxpy
import matplotlib.pyplot as plt

# 轨迹点
K = 50

# 求解器最大迭代次数
iterations = 30

# 权重常数
W_SIGMA = 1  # 飞行时间
W_DELTA = 1e-3  # 状态/输入的差异
W_DELTA_SIGMA = 1e-1  # 飞行时间的差异
W_NU = 1e5  # 虚拟控制

print(cvxpy.installed_solvers())
solver = 'CLARABEL'
verbose_solver = False

show_animation = True


class Rocket_Model_6DoF:
    """
    六自由度火箭着陆问题。
    """

    def __init__(self, rng):
        """
        对小尺度问题使用过大的 r_scale
        会导致数值问题，因为参数会变得过小，
        并且（看起来）动力学中会损失精度。
        """
        self.n_x = 14
        self.n_u = 3

        # 质量
        self.m_wet = 3.0  # 30000 kg
        self.m_dry = 2.2  # 22000 kg

        # 飞行时间初值
        self.t_f_guess = 10.0  # 10 s

        # 状态约束
        self.r_I_final = np.array((0., 0., 0.))
        self.v_I_final = np.array((-1e-1, 0., 0.))
        self.q_B_I_final = self.euler_to_quat((0, 0, 0))
        self.w_B_final = np.deg2rad(np.array((0., 0., 0.)))

        self.w_B_max = np.deg2rad(60)

        # 角度
        max_gimbal = 20
        max_angle = 90
        glidelslope_angle = 20

        self.tan_delta_max = np.tan(np.deg2rad(max_gimbal))
        self.cos_theta_max = np.cos(np.deg2rad(max_angle))
        self.tan_gamma_gs = np.tan(np.deg2rad(glidelslope_angle))

        # 推力限制
        self.T_max = 5.0
        self.T_min = 0.3

        # 转动惯量
        self.J_B = 1e-2 * np.diag([1., 1., 1.])

        # 重力
        self.g_I = np.array((-1, 0., 0.))

        # 燃料消耗
        self.alpha_m = 0.01

        # 从推力作用点到质心的向量
        self.r_T_B = np.array([-1e-2, 0., 0.])

        self.set_random_initial_state(rng)

        self.x_init = np.concatenate(
            ((self.m_wet,), self.r_I_init, self.v_I_init, self.q_B_I_init, self.w_B_init))
        self.x_final = np.concatenate(
            ((self.m_dry,), self.r_I_final, self.v_I_final, self.q_B_I_final, self.w_B_final))

        self.r_scale = np.linalg.norm(self.r_I_init)
        self.m_scale = self.m_wet

    def set_random_initial_state(self, rng):
        if rng is None:
            rng = np.random.default_rng()

        self.r_I_init = np.array((0., 0., 0.))
        self.r_I_init[0] = rng.uniform(3, 4)
        self.r_I_init[1:3] = rng.uniform(-2, 2, size=2)

        self.v_I_init = np.array((0., 0., 0.))
        self.v_I_init[0] = rng.uniform(-1, -0.5)
        self.v_I_init[1:3] = rng.uniform(-0.5, -0.2,
                                         size=2) * self.r_I_init[1:3]

        self.q_B_I_init = self.euler_to_quat((0,
                                              rng.uniform(-30, 30),
                                              rng.uniform(-30, 30)))
        self.w_B_init = np.deg2rad((0,
                                    rng.uniform(-20, 20),
                                    rng.uniform(-20, 20)))

    def f_func(self, x, u):
        m, _, _, _, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz = x[0], x[1], x[
            2], x[3], x[4], x[5], x[6], x[7], x[8], x[9], x[10], x[11], x[12], x[13]
        ux, uy, uz = u[0], u[1], u[2]

        return np.array([
            [-0.01 * np.sqrt(ux**2 + uy**2 + uz**2)],
            [vx],
            [vy],
            [vz],
            [(-1.0 * m - ux * (2 * q2**2 + 2 * q3**2 - 1) - 2 * uy
              * (q0 * q3 - q1 * q2) + 2 * uz * (q0 * q2 + q1 * q3)) / m],
            [(2 * ux * (q0 * q3 + q1 * q2) - uy * (2 * q1**2
                                                   + 2 * q3**2 - 1) - 2 * uz * (q0 * q1 - q2 * q3)) / m],
            [(-2 * ux * (q0 * q2 - q1 * q3) + 2 * uy
              * (q0 * q1 + q2 * q3) - uz * (2 * q1**2 + 2 * q2**2 - 1)) / m],
            [-0.5 * q1 * wx - 0.5 * q2 * wy - 0.5 * q3 * wz],
            [0.5 * q0 * wx + 0.5 * q2 * wz - 0.5 * q3 * wy],
            [0.5 * q0 * wy - 0.5 * q1 * wz + 0.5 * q3 * wx],
            [0.5 * q0 * wz + 0.5 * q1 * wy - 0.5 * q2 * wx],
            [0],
            [1.0 * uz],
            [-1.0 * uy]
        ])

    def A_func(self, x, u):
        m, _, _, _, _, _, _, q0, q1, q2, q3, wx, wy, wz = x[0], x[1], x[
            2], x[3], x[4], x[5], x[6], x[7], x[8], x[9], x[10], x[11], x[12], x[13]
        ux, uy, uz = u[0], u[1], u[2]

        return np.array([
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [(ux * (2 * q2**2 + 2 * q3**2 - 1) + 2 * uy * (q0 * q3 - q1 * q2) - 2 * uz * (q0 * q2 + q1 * q3)) / m**2, 0, 0, 0, 0, 0, 0, 2 * (q2 * uz
                                                                                                                                             - q3 * uy) / m, 2 * (q2 * uy + q3 * uz) / m, 2 * (q0 * uz + q1 * uy - 2 * q2 * ux) / m, 2 * (-q0 * uy + q1 * uz - 2 * q3 * ux) / m, 0, 0, 0],
            [(-2 * ux * (q0 * q3 + q1 * q2) + uy * (2 * q1**2 + 2 * q3**2 - 1) + 2 * uz * (q0 * q1 - q2 * q3)) / m**2, 0, 0, 0, 0, 0, 0, 2 * (-q1 * uz
                                                                                                                                              + q3 * ux) / m, 2 * (-q0 * uz - 2 * q1 * uy + q2 * ux) / m, 2 * (q1 * ux + q3 * uz) / m, 2 * (q0 * ux + q2 * uz - 2 * q3 * uy) / m, 0, 0, 0],
            [(2 * ux * (q0 * q2 - q1 * q3) - 2 * uy * (q0 * q1 + q2 * q3) + uz * (2 * q1**2 + 2 * q2**2 - 1)) / m**2, 0, 0, 0, 0, 0, 0, 2 * (q1 * uy
                                                                                                                                             - q2 * ux) / m, 2 * (q0 * uy - 2 * q1 * uz + q3 * ux) / m, 2 * (-q0 * ux - 2 * q2 * uz + q3 * uy) / m, 2 * (q1 * ux + q2 * uy) / m, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, -0.5 * wx, -0.5 * wy,
             - 0.5 * wz, -0.5 * q1, -0.5 * q2, -0.5 * q3],
            [0, 0, 0, 0, 0, 0, 0, 0.5 * wx, 0, 0.5 * wz,
             - 0.5 * wy, 0.5 * q0, -0.5 * q3, 0.5 * q2],
            [0, 0, 0, 0, 0, 0, 0, 0.5 * wy, -0.5 * wz, 0,
             0.5 * wx, 0.5 * q3, 0.5 * q0, -0.5 * q1],
            [0, 0, 0, 0, 0, 0, 0, 0.5 * wz, 0.5 * wy,
             - 0.5 * wx, 0, -0.5 * q2, 0.5 * q1, 0.5 * q0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

    def B_func(self, x, u):
        m, _, _, _, _, _, _, q0, q1, q2, q3, _, _, _ = x[0], x[1], x[
            2], x[3], x[4], x[5], x[6], x[7], x[8], x[9], x[10], x[11], x[12], x[13]
        ux, uy, uz = u[0], u[1], u[2]

        return np.array([
            [-0.01 * ux / np.sqrt(ux**2 + uy**2 + uz**2),
             -0.01 * uy / np.sqrt(ux ** 2 + uy**2 + uz**2),
             -0.01 * uz / np.sqrt(ux**2 + uy**2 + uz**2)],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [(-2 * q2**2 - 2 * q3**2 + 1) / m, 2
             * (-q0 * q3 + q1 * q2) / m, 2 * (q0 * q2 + q1 * q3) / m],
            [2 * (q0 * q3 + q1 * q2) / m, (-2 * q1**2 - 2
                                           * q3**2 + 1) / m, 2 * (-q0 * q1 + q2 * q3) / m],
            [2 * (-q0 * q2 + q1 * q3) / m, 2 * (q0 * q1 + q2 * q3)
             / m, (-2 * q1**2 - 2 * q2**2 + 1) / m],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 1.0],
            [0, -1.0, 0]
        ])

    def euler_to_quat(self, a):
        a = np.deg2rad(a)

        cy = np.cos(a[1] * 0.5)
        sy = np.sin(a[1] * 0.5)
        cr = np.cos(a[0] * 0.5)
        sr = np.sin(a[0] * 0.5)
        cp = np.cos(a[2] * 0.5)
        sp = np.sin(a[2] * 0.5)

        q = np.zeros(4)

        q[0] = cy * cr * cp + sy * sr * sp
        q[1] = cy * sr * cp - sy * cr * sp
        q[3] = cy * cr * sp + sy * sr * cp
        q[2] = sy * cr * cp - cy * sr * sp

        return q

    def skew(self, v):
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])

    def dir_cosine(self, q):
        return np.array([
            [1 - 2 * (q[2] ** 2 + q[3] ** 2), 2 * (q[1] * q[2]
                                                   + q[0] * q[3]), 2 * (q[1] * q[3] - q[0] * q[2])],
            [2 * (q[1] * q[2] - q[0] * q[3]), 1 - 2
             * (q[1] ** 2 + q[3] ** 2), 2 * (q[2] * q[3] + q[0] * q[1])],
            [2 * (q[1] * q[3] + q[0] * q[2]), 2 * (q[2] * q[3]
                                                   - q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)]
        ])

    def omega(self, w):
        return np.array([
            [0, -w[0], -w[1], -w[2]],
            [w[0], 0, w[2], -w[1]],
            [w[1], -w[2], 0, w[0]],
            [w[2], w[1], -w[0], 0],
        ])

    def initialize_trajectory(self, X, U):
        """
        用线性近似初始化轨迹。
        """
        K = X.shape[1]

        for k in range(K):
            alpha1 = (K - k) / K
            alpha2 = k / K

            m_k = (alpha1 * self.x_init[0] + alpha2 * self.x_final[0],)
            r_I_k = alpha1 * self.x_init[1:4] + alpha2 * self.x_final[1:4]
            v_I_k = alpha1 * self.x_init[4:7] + alpha2 * self.x_final[4:7]
            q_B_I_k = np.array([1, 0, 0, 0])
            w_B_k = alpha1 * self.x_init[11:14] + alpha2 * self.x_final[11:14]

            X[:, k] = np.concatenate((m_k, r_I_k, v_I_k, q_B_I_k, w_B_k))
            U[:, k] = m_k * -self.g_I

        return X, U

    def get_constraints(self, X_v, U_v, X_last_p, U_last_p):
        """
        获取模型特定的约束。

        :param X_v: 当前状态的 cvx 变量
        :param U_v: 当前输入的 cvx 变量
        :param X_last_p: 上一时刻状态的 cvx 参数
        :param U_last_p: 上一时刻输入的 cvx 参数
        :return: cvx 约束列表
        """
        # 边界条件：
        constraints = [
            X_v[0, 0] == self.x_init[0],
            X_v[1:4, 0] == self.x_init[1:4],
            X_v[4:7, 0] == self.x_init[4:7],
            # X_v[7:11, 0] == self.x_init[7:11],  # 初始姿态自由
            X_v[11:14, 0] == self.x_init[11:14],

            # X_[0, -1] 末端质量自由
            X_v[1:, -1] == self.x_final[1:],
            U_v[1:3, -1] == 0,
        ]

        constraints += [
            # 状态约束：
            X_v[0, :] >= self.m_dry,  # 最小质量
            cvxpy.norm(X_v[2: 4, :], axis=0) <= X_v[1, :] / \
            self.tan_gamma_gs,  # 下滑道
            cvxpy.norm(X_v[9:11, :], axis=0) <= np.sqrt(
                (1 - self.cos_theta_max) / 2),  # 最大角度
            # 最大角速度
            cvxpy.norm(X_v[11: 14, :], axis=0) <= self.w_B_max,

            # 控制约束：
            cvxpy.norm(U_v[1:3, :], axis=0) <= self.tan_delta_max * \
            U_v[0, :],  # 万向节角度约束
            cvxpy.norm(U_v, axis=0) <= self.T_max,  # 推力上界约束
        ]

        # 线性化的推力下界约束
        rhs = [U_last_p[:, k] / cvxpy.norm(U_last_p[:, k]) @ U_v[:, k]
               for k in range(X_v.shape[1])]
        constraints += [
            self.T_min <= cvxpy.vstack(rhs)
        ]

        return constraints


class Integrator:
    def __init__(self, m, K):
        self.K = K
        self.m = m
        self.n_x = m.n_x
        self.n_u = m.n_u

        self.A_bar = np.zeros([m.n_x * m.n_x, K - 1])
        self.B_bar = np.zeros([m.n_x * m.n_u, K - 1])
        self.C_bar = np.zeros([m.n_x * m.n_u, K - 1])
        self.S_bar = np.zeros([m.n_x, K - 1])
        self.z_bar = np.zeros([m.n_x, K - 1])

        # 展平矩阵的向量下标
        x_end = m.n_x
        A_bar_end = m.n_x * (1 + m.n_x)
        B_bar_end = m.n_x * (1 + m.n_x + m.n_u)
        C_bar_end = m.n_x * (1 + m.n_x + m.n_u + m.n_u)
        S_bar_end = m.n_x * (1 + m.n_x + m.n_u + m.n_u + 1)
        z_bar_end = m.n_x * (1 + m.n_x + m.n_u + m.n_u + 2)
        self.x_ind = slice(0, x_end)
        self.A_bar_ind = slice(x_end, A_bar_end)
        self.B_bar_ind = slice(A_bar_end, B_bar_end)
        self.C_bar_ind = slice(B_bar_end, C_bar_end)
        self.S_bar_ind = slice(C_bar_end, S_bar_end)
        self.z_bar_ind = slice(S_bar_end, z_bar_end)

        self.f, self.A, self.B = m.f_func, m.A_func, m.B_func

        # 积分初值
        self.V0 = np.zeros((m.n_x * (1 + m.n_x + m.n_u + m.n_u + 2),))
        self.V0[self.A_bar_ind] = np.eye(m.n_x).reshape(-1)

        self.dt = 1. / (K - 1)

    def calculate_discretization(self, X, U, sigma):
        """
        对给定的状态、输入与总时间计算离散化。

        :param X: 全部时刻的状态矩阵
        :param U: 全部时刻的输入矩阵
        :param sigma: 总时间
        :return: 离散化矩阵
        """
        for k in range(self.K - 1):
            self.V0[self.x_ind] = X[:, k]
            V = np.array(odeint(self._ode_dVdt, self.V0, (0, self.dt),
                                args=(U[:, k], U[:, k + 1], sigma))[1, :])

            # 利用 \Phi_A(\tau_{k+1},\xi) = \Phi_A(\tau_{k+1},\tau_k)\Phi_A(\xi,\tau_k)^{-1}
            # 按列优先（Fortran）顺序展平矩阵，供 CVXPY 使用
            Phi = V[self.A_bar_ind].reshape((self.n_x, self.n_x))
            self.A_bar[:, k] = Phi.flatten(order='F')
            self.B_bar[:, k] = np.matmul(Phi, V[self.B_bar_ind].reshape(
                (self.n_x, self.n_u))).flatten(order='F')
            self.C_bar[:, k] = np.matmul(Phi, V[self.C_bar_ind].reshape(
                (self.n_x, self.n_u))).flatten(order='F')
            self.S_bar[:, k] = np.matmul(Phi, V[self.S_bar_ind])
            self.z_bar[:, k] = np.matmul(Phi, V[self.z_bar_ind])

        return self.A_bar, self.B_bar, self.C_bar, self.S_bar, self.z_bar

    def _ode_dVdt(self, V, t, u_t0, u_t1, sigma):
        """
        用于计算 dVdt 的常微分方程函数。

        :param V: 求值状态 V = [x, Phi_A, B_bar, C_bar, S_bar, z_bar]
        :param t: 求值时刻
        :param u_t0: 区间起点的输入
        :param u_t1: 区间终点的输入
        :param sigma: 总时间
        :return: 当前时刻与状态下的导数 dVdt
        """
        alpha = (self.dt - t) / self.dt
        beta = t / self.dt
        x = V[self.x_ind]
        u = u_t0 + beta * (u_t1 - u_t0)

        # 利用 \Phi_A(\tau_{k+1},\xi) = \Phi_A(\tau_{k+1},\tau_k)\Phi_A(\xi,\tau_k)^{-1}
        # 并在积分后左乘 \Phi_A(\tau_{k+1},\tau_k)
        Phi_A_xi = np.linalg.inv(
            V[self.A_bar_ind].reshape((self.n_x, self.n_x)))

        A_subs = sigma * self.A(x, u)
        B_subs = sigma * self.B(x, u)
        f_subs = self.f(x, u)

        dVdt = np.zeros_like(V)
        dVdt[self.x_ind] = sigma * f_subs.transpose()
        dVdt[self.A_bar_ind] = np.matmul(
            A_subs, V[self.A_bar_ind].reshape((self.n_x, self.n_x))).reshape(-1)
        dVdt[self.B_bar_ind] = np.matmul(Phi_A_xi, B_subs).reshape(-1) * alpha
        dVdt[self.C_bar_ind] = np.matmul(Phi_A_xi, B_subs).reshape(-1) * beta
        dVdt[self.S_bar_ind] = np.matmul(Phi_A_xi, f_subs).transpose()
        z_t = -np.matmul(A_subs, x) - np.matmul(B_subs, u)
        dVdt[self.z_bar_ind] = np.dot(Phi_A_xi, z_t.T).flatten()

        return dVdt


class SCProblem:
    """
    定义标准的逐次凸化问题，并
            加入模型特定的约束与目标函数。

    :param m: 模型对象
    :param K: 离散点数量
    """

    def __init__(self, m, K):
        # 变量：
        self.var = dict()
        self.var['X'] = cvxpy.Variable((m.n_x, K))
        self.var['U'] = cvxpy.Variable((m.n_u, K))
        self.var['sigma'] = cvxpy.Variable(nonneg=True)
        self.var['nu'] = cvxpy.Variable((m.n_x, K - 1))
        self.var['delta_norm'] = cvxpy.Variable(nonneg=True)
        self.var['sigma_norm'] = cvxpy.Variable(nonneg=True)

        # 参数：
        self.par = dict()
        self.par['A_bar'] = cvxpy.Parameter((m.n_x * m.n_x, K - 1))
        self.par['B_bar'] = cvxpy.Parameter((m.n_x * m.n_u, K - 1))
        self.par['C_bar'] = cvxpy.Parameter((m.n_x * m.n_u, K - 1))
        self.par['S_bar'] = cvxpy.Parameter((m.n_x, K - 1))
        self.par['z_bar'] = cvxpy.Parameter((m.n_x, K - 1))

        self.par['X_last'] = cvxpy.Parameter((m.n_x, K))
        self.par['U_last'] = cvxpy.Parameter((m.n_u, K))
        self.par['sigma_last'] = cvxpy.Parameter(nonneg=True)

        self.par['weight_sigma'] = cvxpy.Parameter(nonneg=True)
        self.par['weight_delta'] = cvxpy.Parameter(nonneg=True)
        self.par['weight_delta_sigma'] = cvxpy.Parameter(nonneg=True)
        self.par['weight_nu'] = cvxpy.Parameter(nonneg=True)

        # 约束：
        constraints = []

        # 模型：
        constraints += m.get_constraints(
            self.var['X'], self.var['U'], self.par['X_last'], self.par['U_last'])

        # 动力学：
        # x_t+1 = A_*x_t+B_*U_t+C_*U_T+1*S_*sigma+zbar+nu
        constraints += [
            self.var['X'][:, k + 1] ==
            cvxpy.reshape(self.par['A_bar'][:, k], (m.n_x, m.n_x), order='F') @
            self.var['X'][:, k] +
            cvxpy.reshape(self.par['B_bar'][:, k], (m.n_x, m.n_u), order='F') @
            self.var['U'][:, k] +
            cvxpy.reshape(self.par['C_bar'][:, k], (m.n_x, m.n_u), order='F') @
            self.var['U'][:, k + 1] +
            self.par['S_bar'][:, k] * self.var['sigma'] +
            self.par['z_bar'][:, k] +
            self.var['nu'][:, k]
            for k in range(K - 1)
        ]

        # 信赖域：
        dx = cvxpy.sum(cvxpy.square(
            self.var['X'] - self.par['X_last']), axis=0)
        du = cvxpy.sum(cvxpy.square(
            self.var['U'] - self.par['U_last']), axis=0)
        ds = self.var['sigma'] - self.par['sigma_last']
        constraints += [cvxpy.norm(dx + du, 1) <= self.var['delta_norm']]
        constraints += [cvxpy.norm(ds, 'inf') <= self.var['sigma_norm']]

        # 飞行时间为正：
        constraints += [self.var['sigma'] >= 0.1]

        # 目标函数：
        sc_objective = cvxpy.Minimize(
            self.par['weight_sigma'] * self.var['sigma'] +
            self.par['weight_nu'] * cvxpy.norm(self.var['nu'], 'inf') +
            self.par['weight_delta'] * self.var['delta_norm'] +
            self.par['weight_delta_sigma'] * self.var['sigma_norm']
        )

        objective = sc_objective

        self.prob = cvxpy.Problem(objective, constraints)

    def set_parameters(self, **kwargs):
        """
        调用 solve() 之前必须填入全部参数。
        接受以下关键字参数：

        A_bar
        B_bar
        C_bar
        S_bar
        z_bar
        X_last
        U_last
        sigma_last
        E
        weight_sigma
        weight_nu
        radius_trust_region
        """

        for key in kwargs:
            if key in self.par:
                self.par[key].value = kwargs[key]
            else:
                print(f'Parameter \'{key}\' does not exist.')

    def get_variable(self, name):
        if name in self.var:
            return self.var[name].value
        else:
            print(f'Variable \'{name}\' does not exist.')
            return None

    def solve(self, **kwargs):
        error = False
        try:
            with warnings.catch_warnings():  # 忽略求解器的 UserWarning
                warnings.simplefilter('ignore')
                self.prob.solve(verbose=verbose_solver,
                                solver=solver)
        except cvxpy.SolverError:
            error = True

        stats = self.prob.solver_stats

        info = {
            'setup_time': stats.setup_time,
            'solver_time': stats.solve_time,
            'iterations': stats.num_iters,
            'solver_error': error
        }

        return info


def axis3d_equal(X, Y, Z, ax):

    max_range = np.array([X.max() - X.min(), Y.max()
                          - Y.min(), Z.max() - Z.min()]).max()
    Xb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2,
                                    - 1:2:2][0].flatten() + 0.5 * (X.max() + X.min())
    Yb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2,
                                    - 1:2:2][1].flatten() + 0.5 * (Y.max() + Y.min())
    Zb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2,
                                    - 1:2:2][2].flatten() + 0.5 * (Z.max() + Z.min())
    # 注释或取消注释下面两行，以测试伪包围盒：
    for xb, yb, zb in zip(Xb, Yb, Zb):
        ax.plot([xb], [yb], [zb], 'w')


def plot_animation(X, U):  # pragma: no cover

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    # 按 Esc 键停止仿真。
    fig.canvas.mpl_connect('key_release_event',
                           lambda event: [exit(0) if event.key == 'escape' else None])

    for k in range(K):
        plt.cla()
        ax.plot(X[2, :], X[3, :], X[1, :])  # 轨迹
        ax.scatter3D([0.0], [0.0], [0.0], c="r",
                     marker="x")  # 目标着陆点
        axis3d_equal(X[2, :], X[3, :], X[1, :], ax)

        rx, ry, rz = X[1:4, k]
        # vx, vy, vz = X[4:7, k]
        qw, qx, qy, qz = X[7:11, k]

        CBI = np.array([
            [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy + qw * qz),
             2 * (qx * qz - qw * qy)],
            [2 * (qx * qy - qw * qz), 1 - 2
             * (qx ** 2 + qz ** 2), 2 * (qy * qz + qw * qx)],
            [2 * (qx * qz + qw * qy), 2 * (qy * qz - qw * qx),
             1 - 2 * (qx ** 2 + qy ** 2)]
        ])

        Fx, Fy, Fz = np.dot(np.transpose(CBI), U[:, k])
        dx, dy, dz = np.dot(np.transpose(CBI), np.array([1., 0., 0.]))

        # 姿态向量
        ax.quiver(ry, rz, rx, dy, dz, dx, length=0.5, linewidth=3.0,
                  arrow_length_ratio=0.0, color='black')

        # 推力向量
        ax.quiver(ry, rz, rx, -Fy, -Fz, -Fx, length=0.1,
                  arrow_length_ratio=0.0, color='red')

        ax.set_title("Rocket powered landing")
        plt.pause(0.5)


def main(rng=None):
    print("start!!")
    m = Rocket_Model_6DoF(rng)

    # 状态与输入列表
    X = np.empty(shape=[m.n_x, K])
    U = np.empty(shape=[m.n_u, K])

    # 初始化
    sigma = m.t_f_guess
    X, U = m.initialize_trajectory(X, U)

    integrator = Integrator(m, K)
    problem = SCProblem(m, K)

    converged = False
    w_delta = W_DELTA
    for it in range(iterations):
        t0_it = time()
        print('-' * 18 + f' Iteration {str(it + 1).zfill(2)} ' + '-' * 18)

        A_bar, B_bar, C_bar, S_bar, z_bar = integrator.calculate_discretization(
            X, U, sigma)

        problem.set_parameters(A_bar=A_bar, B_bar=B_bar, C_bar=C_bar, S_bar=S_bar, z_bar=z_bar,
                               X_last=X, U_last=U, sigma_last=sigma,
                               weight_sigma=W_SIGMA, weight_nu=W_NU,
                               weight_delta=w_delta, weight_delta_sigma=W_DELTA_SIGMA)
        problem.solve()

        X = problem.get_variable('X')
        U = problem.get_variable('U')
        sigma = problem.get_variable('sigma')

        delta_norm = problem.get_variable('delta_norm')
        sigma_norm = problem.get_variable('sigma_norm')
        nu_norm = np.linalg.norm(problem.get_variable('nu'), np.inf)

        print('delta_norm', delta_norm)
        print('sigma_norm', sigma_norm)
        print('nu_norm', nu_norm)

        if delta_norm < 1e-3 and sigma_norm < 1e-3 and nu_norm < 1e-7:
            converged = True

        w_delta *= 1.5

        print('Time for iteration', time() - t0_it, 's')

        if converged:
            print(f'Converged after {it + 1} iterations.')
            break

    if show_animation:  # pragma: no cover
        plot_animation(X, U)

    print("done!!")


if __name__ == '__main__':
    main()
