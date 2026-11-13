"""

无迹卡尔曼滤波（UKF）定位示例

作者：Atsushi Sakai (@Atsushi_twi)

"""

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import math

import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg

from utils.angle import rot_mat_2d

# 无迹卡尔曼滤波（UKF）仿真的协方差
Q = np.diag([
    0.1,  # x 轴位置方差
    0.1,  # y 轴位置方差
    np.deg2rad(1.0),  # 航向角方差
    1.0  # 速度方差
]) ** 2  # 预测状态协方差
R = np.diag([1.0, 1.0]) ** 2  # 观测 x、y 位置协方差

#  仿真参数
INPUT_NOISE = np.diag([1.0, np.deg2rad(30.0)]) ** 2
GPS_NOISE = np.diag([0.5, 0.5]) ** 2

DT = 0.1  # 时间步长 [s]
SIM_TIME = 50.0  # 仿真时间 [s]

#  无迹卡尔曼滤波（UKF）参数
ALPHA = 0.001
BETA = 2
KAPPA = 0

show_animation = True


def calc_input():
    v = 1.0  # [m/s]
    yawRate = 0.1  # [rad/s]
    u = np.array([[v, yawRate]]).T
    return u


def observation(xTrue, xd, u):
    xTrue = motion_model(xTrue, u)

    # 为 GPS 的 x-y 添加噪声
    z = observation_model(xTrue) + GPS_NOISE @ np.random.randn(2, 1)

    # 为输入添加噪声
    ud = u + INPUT_NOISE @ np.random.randn(2, 1)

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


def motion_model(x, u):
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F @ x + B @ u

    return x


def observation_model(x):
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = H @ x

    return z


def generate_sigma_points(xEst, PEst, gamma):
    """
    为无迹卡尔曼滤波（UKF）生成 sigma 点。

    sigma 点是确定性采样点，用于刻画状态分布的均值与协方差。

    Parameters
    ----------
    xEst : numpy.ndarray
        当前状态估计 (n x 1)
    PEst : numpy.ndarray
        当前状态协方差估计 (n x n)
    gamma : float
        缩放参数 sqrt(n + lambda)

    Returns
    -------
    sigma : numpy.ndarray
        sigma 点 (n x 2n+1)
        sigma[:, 0] = xEst
        sigma[:, 1:n+1] = xEst + gamma * sqrt(P)
        sigma[:, n+1:2n+1] = xEst - gamma * sqrt(P)
    """
    sigma = xEst
    Psqrt = scipy.linalg.sqrtm(PEst)
    n = len(xEst[:, 0])
    # 正方向
    for i in range(n):
        sigma = np.hstack((sigma, xEst + gamma * Psqrt[:, i:i + 1]))

    # 负方向
    for i in range(n):
        sigma = np.hstack((sigma, xEst - gamma * Psqrt[:, i:i + 1]))

    return sigma


def predict_sigma_motion(sigma, u):
    """
        用运动模型预测 sigma 点
    """
    for i in range(sigma.shape[1]):
        sigma[:, i:i + 1] = motion_model(sigma[:, i:i + 1], u)

    return sigma


def predict_sigma_observation(sigma):
    """
        用观测模型预测 sigma 点
    """
    for i in range(sigma.shape[1]):
        sigma[0:2, i] = observation_model(sigma[:, i])

    sigma = sigma[0:2, :]

    return sigma


def calc_sigma_covariance(x, sigma, wc, Pi):
    nSigma = sigma.shape[1]
    d = sigma - x[0:sigma.shape[0]]
    P = Pi
    for i in range(nSigma):
        P = P + wc[0, i] * d[:, i:i + 1] @ d[:, i:i + 1].T
    return P


def calc_pxz(sigma, x, z_sigma, zb, wc):
    nSigma = sigma.shape[1]
    dx = sigma - x
    dz = z_sigma - zb[0:2]
    P = np.zeros((dx.shape[0], dz.shape[0]))

    for i in range(nSigma):
        P = P + wc[0, i] * dx[:, i:i + 1] @ dz[:, i:i + 1].T

    return P


def ukf_estimation(xEst, PEst, z, u, wm, wc, gamma):
    """
    无迹卡尔曼滤波估计。

    利用无迹变换执行一次无迹卡尔曼滤波（UKF）状态估计迭代。

    Parameters
    ----------
    xEst : numpy.ndarray
        当前状态估计 [x, y, yaw, v]^T (4 x 1)
    PEst : numpy.ndarray
        当前状态协方差估计 (4 x 4)
    z : numpy.ndarray
        观测向量 [x_obs, y_obs]^T (2 x 1)
    u : numpy.ndarray
        控制输入 [速度, 航向角速度]^T (2 x 1)
    wm : numpy.ndarray
        用于计算均值的权重 (1 x 2n+1)
    wc : numpy.ndarray
        用于计算协方差的权重 (1 x 2n+1)
    gamma : float
        sigma 点缩放参数 sqrt(n + lambda)

    Returns
    -------
    xEst : numpy.ndarray
        更新后的状态估计 (4 x 1)
    PEst : numpy.ndarray
        更新后的状态协方差估计 (4 x 4)
    """
    #  预测
    sigma = generate_sigma_points(xEst, PEst, gamma)
    sigma = predict_sigma_motion(sigma, u)
    xPred = (wm @ sigma.T).T
    PPred = calc_sigma_covariance(xPred, sigma, wc, Q)

    #  更新
    zPred = observation_model(xPred)
    y = z - zPred
    sigma = generate_sigma_points(xPred, PPred, gamma)
    zb = (wm @ sigma.T).T
    z_sigma = predict_sigma_observation(sigma)
    st = calc_sigma_covariance(zb, z_sigma, wc, R)
    Pxz = calc_pxz(sigma, xPred, z_sigma, zb, wc)
    K = Pxz @ np.linalg.inv(st)
    xEst = xPred + K @ y
    PEst = PPred - K @ st @ K.T

    return xEst, PEst


def plot_covariance_ellipse(xEst, PEst):  # pragma: no cover
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[1, bigind], eigvec[0, bigind])
    fx = rot_mat_2d(angle) @ np.array([x, y])
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


def setup_ukf(nx):
    """
    设置无迹卡尔曼滤波（UKF）参数与权重。

    计算均值与协方差所用的权重，以及生成 sigma 点所用的缩放参数 gamma。

    Parameters
    ----------
    nx : int
        状态向量的维数

    Returns
    -------
    wm : numpy.ndarray
        用于计算均值的权重 (1 x 2n+1)
        wm[0] = lambda / (n + lambda)
        wm[i] = 1 / (2(n + lambda))，当 i > 0
    wc : numpy.ndarray
        用于计算协方差的权重 (1 x 2n+1)
        wc[0] = lambda / (n + lambda) + (1 - alpha^2 + beta)
        wc[i] = 1 / (2(n + lambda))，当 i > 0
    gamma : float
        sigma 点缩放参数 sqrt(n + lambda)
        其中 lambda = alpha^2 * (n + kappa) - n
    """
    lamb = ALPHA ** 2 * (nx + KAPPA) - nx
    # 计算权重
    wm = [lamb / (lamb + nx)]
    wc = [(lamb / (lamb + nx)) + (1 - ALPHA ** 2 + BETA)]
    for i in range(2 * nx):
        wm.append(1.0 / (2 * (nx + lamb)))
        wc.append(1.0 / (2 * (nx + lamb)))
    gamma = math.sqrt(nx + lamb)

    wm = np.array([wm])
    wc = np.array([wc])

    return wm, wc, gamma


def main():
    print(__file__ + " start!!")

    nx = 4  # 状态向量 [x y yaw v]'
    xEst = np.zeros((nx, 1))
    xTrue = np.zeros((nx, 1))
    PEst = np.eye(nx)
    xDR = np.zeros((nx, 1))  # 航位推算

    wm, wc, gamma = setup_ukf(nx)

    # 历史记录
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hz = np.zeros((2, 1))

    time = 0.0

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u)

        xEst, PEst = ukf_estimation(xEst, PEst, z, ud, wm, wc, gamma)

        # 存储数据历史
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.hstack((hz, z))

        if show_animation:
            plt.cla()
            # 按 esc 键停止仿真。
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(hz[0, :], hz[1, :], ".g")
            plt.plot(np.array(hxTrue[0, :]).flatten(),
                     np.array(hxTrue[1, :]).flatten(), "-b")
            plt.plot(np.array(hxDR[0, :]).flatten(),
                     np.array(hxDR[1, :]).flatten(), "-k")
            plt.plot(np.array(hxEst[0, :]).flatten(),
                     np.array(hxEst[1, :]).flatten(), "-r")
            plot_covariance_ellipse(xEst, PEst)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()
