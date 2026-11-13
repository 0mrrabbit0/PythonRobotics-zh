"""使用扩展卡尔曼滤波进行平面 GPS/惯性测量单元（IMU）融合。

状态：[x, y, vx, vy, yaw, accelerometer_bias_x, accelerometer_bias_y,
        gyroscope_bias]。位置与速度处于局部米制世界坐标系。
惯性测量单元（IMU）加速度处于机体坐标系，且已去除重力。
仅建模水平运动；横滚、俯仰、高度、磁力计与气压计不在本示例范围内。
假定 GPS 与惯性测量单元（IMU）时间对齐且安装在同一位置。
假定初始航向与速度近似已知。

运行本文件，可将带偏差估计与不带偏差估计的融合结果，与仅使用
惯性测量单元（IMU）的航位推算进行比较，其中包含一段 GPS 中断。
不估计偏差时，状态仅包含 [x, y, vx, vy, yaw]，并假定偏差为零。
除 NumPy/Matplotlib 外不需要外部数据或依赖。

Reference: Oliver J. Woodman, 《惯性导航导论》, 2007.
https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf
"""

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


DT = 0.05  # 惯性测量单元（IMU）采样周期 [s]（20 Hz）
GPS_INTERVAL = 20  # 每 20 个惯性测量单元（IMU）样本一个 GPS 位置（1 Hz）
SIM_TIME = 50.0  # [s]
IMU_STD = np.array([0.1, 0.1, np.deg2rad(0.3)])  # [m/s^2, m/s^2, rad/s]
GPS_STD = 0.8  # 各轴位置标准差 [m]
BIAS_RW_STD = np.array([0.002, 0.002, np.deg2rad(0.02)])  # 每 sqrt(秒)
TRUE_BIAS = np.array([0.04, -0.03, np.deg2rad(0.4)])
show_animation = True


def rotation(yaw):
    """将二维机体坐标系向量旋转到世界坐标系。"""
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def motion_model(state, imu, dt):
    """传播一个惯性测量单元（IMU）样本，并在 dt 内保持世界系加速度不变。"""
    bias = state[5:] if len(state) == 8 else np.zeros(3)
    acceleration = rotation(state[4]) @ (imu[:2] - bias[:2])
    predicted = state.copy()
    predicted[:2] += state[2:4] * dt + 0.5 * acceleration * dt**2
    predicted[2:4] += acceleration * dt
    predicted[4] = wrap_angle(state[4] + (imu[2] - bias[2]) * dt)
    return predicted


def motion_jacobians(state, imu, dt):
    """返回离散运动模型对状态与惯性测量单元（IMU）输入的雅可比矩阵。"""
    rot = rotation(state[4])
    bias = state[5:7] if len(state) == 8 else np.zeros(2)
    acceleration = rot @ (imu[:2] - bias)
    yaw_derivative = np.array([-acceleration[1], acceleration[0]])
    f = np.eye(len(state))
    f[:2, 2:4] = np.eye(2) * dt
    f[:2, 4] = 0.5 * yaw_derivative * dt**2
    f[2:4, 4] = yaw_derivative * dt
    if len(state) == 8:
        f[:2, 5:7] = -0.5 * rot * dt**2
        f[2:4, 5:7] = -rot * dt
        f[4, 7] = -dt

    g = np.zeros((len(state), 3))
    g[:2, :2] = 0.5 * rot * dt**2
    g[2:4, :2] = rot * dt
    g[4, 2] = dt
    return f, g


def predict(state, covariance, imu, dt=DT):
    """按惯性测量单元（IMU）频率进行预测，包含传感器噪声与偏差随机游走。

    IMU_STD 描述每个离散惯性测量单元（IMU）样本上的独立噪声，而
    BIAS_RW_STD 描述按每 sqrt(秒) 计的连续偏差随机游走。
    五状态滤波器假定偏差为零，且没有偏差随机游走。
    """
    f, g = motion_jacobians(state, imu, dt)
    process_noise = g @ np.diag(IMU_STD**2) @ g.T
    if len(state) == 8:
        process_noise[5:8, 5:8] += np.diag(BIAS_RW_STD**2) * dt
    predicted_covariance = f @ covariance @ f.T + process_noise
    return motion_model(state, imu, dt), (
        predicted_covariance + predicted_covariance.T) / 2.0


def update_gps(state, covariance, position):
    """当有新的二维 GPS 位置时，校正预测状态。

    GPS 只观测位置。速度、航向与偏差通过惯性测量单元（IMU）
    预测过程中积累的互协方差得到校正。
    """
    h = np.zeros((2, len(state)))
    h[:, :2] = np.eye(2)
    r = np.eye(2) * GPS_STD**2
    innovation_covariance = h @ covariance @ h.T + r
    gain = np.linalg.solve(innovation_covariance, h @ covariance).T
    updated = state + gain @ (position - h @ state)
    updated[4] = wrap_angle(updated[4])
    # Joseph 形式保持协方差的对称性与半正定性。
    residual = np.eye(len(state)) - gain @ h
    updated_covariance = residual @ covariance @ residual.T + gain @ r @ gain.T
    return updated, (updated_covariance + updated_covariance.T) / 2.0


def reference_motion(time):
    """解析的八字形真值，以及理想的机体坐标系惯性测量单元（IMU）量测。"""
    position = np.array([20.0 * np.sin(0.1 * time), 10.0 * np.sin(0.2 * time)])
    velocity = np.array([2.0 * np.cos(0.1 * time), 2.0 * np.cos(0.2 * time)])
    acceleration = np.array([-0.2 * np.sin(0.1 * time), -0.4 * np.sin(0.2 * time)])
    yaw = np.arctan2(velocity[1], velocity[0])
    yaw_rate = (velocity[0] * acceleration[1] - velocity[1] * acceleration[0]) / (
        velocity @ velocity)
    state = np.concatenate((position, velocity, [yaw], TRUE_BIAS))
    imu = np.concatenate((rotation(yaw).T @ acceleration, [yaw_rate]))
    return state, imu


def simulate(duration=SIM_TIME, seed=0, gps_outage=(20.0, 30.0)):
    """用多速率传感器运行一次确定性仿真。

    GPS 中断覆盖区间 [start, end)；传入 None 表示 GPS 不中断。真值以解析方式
    计算，独立于滤波器的离散积分器。三种估计共用同一组带偏差、带噪声的量测，
    并从已知的初始位姿/速度出发。八状态扩展卡尔曼滤波从零估计偏差开始；
    五状态扩展卡尔曼滤波与航位推算假定偏差为零。
    """
    rng = np.random.default_rng(seed)
    times = np.arange(int(round(duration / DT)) + 1) * DT
    truth = np.array([reference_motion(t)[0] for t in times])
    estimates = np.zeros_like(truth)
    no_bias_estimates = np.zeros((len(times), 5))
    dead_reckoning = np.zeros_like(truth)
    covariances = np.zeros((len(times), 8, 8))
    no_bias_covariances = np.zeros((len(times), 5, 5))
    gps = np.full((len(times), 2), np.nan)
    state = truth[0].copy()
    state[5:] = 0.0
    dead_state = state.copy()
    covariance = np.diag([1.0, 1.0, 0.2, 0.2, np.deg2rad(5.0),
                          0.1, 0.1, np.deg2rad(1.0)])**2
    no_bias_state = state[:5].copy()
    no_bias_covariance = covariance[:5, :5].copy()
    estimates[0], dead_reckoning[0], covariances[0] = state, dead_state, covariance
    no_bias_estimates[0], no_bias_covariances[0] = no_bias_state, no_bias_covariance
    for i in range(1, len(times)):
        _, ideal_imu = reference_motion(times[i - 1])
        imu = ideal_imu + TRUE_BIAS + rng.normal(size=3) * IMU_STD
        state, covariance = predict(state, covariance, imu)
        no_bias_state, no_bias_covariance = predict(no_bias_state, no_bias_covariance, imu)
        dead_state = motion_model(dead_state, imu, DT)
        # 即使在中断期间也抽取每一次预定定位，以便在相同随机种子下
        # 比较不同中断时段时传感器噪声保持一致。
        if i % GPS_INTERVAL == 0:
            measurement = truth[i, :2] + rng.normal(size=2) * GPS_STD
            if gps_outage is None or not gps_outage[0] <= times[i] < gps_outage[1]:
                gps[i] = measurement
                state, covariance = update_gps(state, covariance, measurement)
                no_bias_state, no_bias_covariance = update_gps(
                    no_bias_state, no_bias_covariance, measurement)
        estimates[i], dead_reckoning[i], covariances[i] = state, dead_state, covariance
        no_bias_estimates[i], no_bias_covariances[i] = no_bias_state, no_bias_covariance
    return {"time": times, "truth": truth, "estimate": estimates,
            "no_bias_estimate": no_bias_estimates, "no_bias_covariance": no_bias_covariances,
            "dead_reckoning": dead_reckoning, "covariance": covariances,
            "gps": gps, "gps_outage": gps_outage}


def position_covariance_ellipse(position, covariance):
    """返回半轴为三倍标准差的二维位置椭圆。"""
    eigenvalues, eigenvectors = np.linalg.eigh(covariance[:2, :2])
    angles = np.linspace(0.0, 2.0 * np.pi, 61)
    circle = np.array([np.cos(angles), np.sin(angles)])
    # 对半正定协方差，将舍入误差截断到零。
    radii = 3.0 * np.sqrt(np.maximum(eigenvalues, 0.0))
    return position[:2, None] + eigenvectors @ (radii[:, None] * circle)


def create_animation(history):  # pragma: no cover
    """相对真值动画显示路径、位置误差与偏差估计。"""
    fig = plt.figure(figsize=(12, 8))
    grid = fig.add_gridspec(2, 6, height_ratios=[1.3, 1])
    path_ax = fig.add_subplot(grid[0, :3])
    error_ax = fig.add_subplot(grid[0, 3:])
    bias_axes = [fig.add_subplot(grid[1, 2 * i:2 * i + 2], sharex=error_ax)
                 for i in range(3)]
    colors = {"truth": "black", "estimate": "tab:blue",
              "no_bias_estimate": "tab:red", "dead_reckoning": "tab:orange"}
    labels = {"truth": "Ground truth", "estimate": "EKF (bias estimation)",
              "no_bias_estimate": "EKF (no bias estimation)",
              "dead_reckoning": "IMU only"}
    styles = {"truth": "-", "estimate": "-", "no_bias_estimate": "--", "dead_reckoning": ":"}
    lines = {key: path_ax.plot([], [], color=color, linestyle=styles[key], label=labels[key])[0]
             for key, color in colors.items()}
    current_points = {key: path_ax.plot([], [], marker="x" if key == "truth" else "o",
                                       color=colors[key], markersize=6, linestyle="none")[0]
                      for key in ["truth", "estimate", "no_bias_estimate"]}
    gps_line, = path_ax.plot([], [], "+", color="tab:green", label="GPS fixes", alpha=0.7)
    covariance_keys = {"estimate": "covariance", "no_bias_estimate": "no_bias_covariance"}
    ellipse_lines = {}
    positions = [history[key][:, :2] for key in colors]
    for key, covariance_key in covariance_keys.items():
        ellipse_lines[key], = path_ax.plot([], [], color=colors[key], linestyle=styles[key],
                                           linewidth=1.5, label=labels[key] + " 3σ")
        position_width = 3.0 * np.sqrt(np.maximum(
            np.diagonal(history[covariance_key][:, :2, :2], axis1=1, axis2=2), 0.0))
        positions.extend([history[key][:, :2] - position_width,
                          history[key][:, :2] + position_width])
    positions = np.vstack(positions)
    path_ax.set(xlim=(positions[:, 0].min() - 3, positions[:, 0].max() + 3),
                ylim=(positions[:, 1].min() - 3, positions[:, 1].max() + 3),
                xlabel="x [m]", ylabel="y [m]")
    path_ax.set_aspect("equal", adjustable="box")
    path_ax.legend(loc="best", fontsize=7)
    errors = {key: np.linalg.norm(history[key][:, :2] - history["truth"][:, :2], axis=1)
              for key in ["estimate", "no_bias_estimate", "dead_reckoning"]}
    error_lines = {key: error_ax.plot([], [], color=colors[key], linestyle=styles[key], label=labels[key])[0]
                   for key in errors}
    # 这是椭圆的外接半径，不是一维误差标准差。
    position_radius = 3.0 * np.sqrt(np.maximum(
        np.linalg.eigvalsh(history["covariance"][:, :2, :2])[:, -1], 0.0))
    radius_line, = error_ax.plot([], [], "--", color="tab:blue", label="EKF 3σ major radius")
    error_ax.set(xlim=(0, history["time"][-1]),
                 ylim=(0, max(position_radius.max(),
                              max(values.max() for values in errors.values())) * 1.1 + 0.1),
                 xlabel="Time [s]", ylabel="Position error [m]")
    outage = history["gps_outage"]
    if outage is not None:
        error_ax.axvspan(*outage, color="gray", alpha=0.2, label="GPS outage")
    error_ax.legend(loc="upper left", fontsize=8)
    path_ax.grid(True)
    error_ax.grid(True)

    # 加速度计偏差保持 m/s^2；陀螺仪偏差以 deg/s 显示。
    bias_scale = np.array([1.0, 1.0, 180.0 / np.pi])
    bias_estimates = history["estimate"][:, 5:8] * bias_scale
    bias_truth = history["truth"][:, 5:8] * bias_scale
    bias_width = 3.0 * np.sqrt(np.maximum(
        np.diagonal(history["covariance"], axis1=1, axis2=2)[:, 5:8], 0.0)) * bias_scale
    bias_lower, bias_upper = bias_estimates - bias_width, bias_estimates + bias_width
    bias_titles = ["Accelerometer x bias", "Accelerometer y bias", "Gyroscope bias"]
    bias_units = ["Bias [m/s²]", "Bias [m/s²]", "Bias [deg/s]"]
    bias_lines = []
    bias_bands = []
    for i, ax in enumerate(bias_axes):
        line, = ax.plot([], [], color="tab:blue", label="EKF estimate")
        bias_lines.append(line)
        ax.plot(history["time"], bias_truth[:, i], "k--", label="Ground truth")
        bias_bands.append(ax.fill_between([], [], [], color="tab:blue", alpha=0.2,
                                           label="Estimate ±3σ"))
        values = np.concatenate((bias_lower[:, i], bias_upper[:, i], bias_truth[:, i]))
        padding = max(np.ptp(values) * 0.15, 0.01)
        ax.set(ylim=(values.min() - padding, values.max() + padding),
               xlabel="Time [s]", ylabel=bias_units[i], title=bias_titles[i])
        if outage is not None:
            ax.axvspan(*outage, color="gray", alpha=0.2)
        ax.legend(loc="best", fontsize=8)
        ax.grid(True)

    animation_title = "GPS/IMU Fusion Localization with Bias Estimation"
    title = fig.suptitle(animation_title + "\n", fontsize=12)
    fig.tight_layout()

    def update(index):
        for key, line in lines.items():
            line.set_data(history[key][:index + 1, 0], history[key][:index + 1, 1])
        for key, point in current_points.items():
            point.set_data([history[key][index, 0]], [history[key][index, 1]])
        for key, line in ellipse_lines.items():
            ellipse = position_covariance_ellipse(
                history[key][index], history[covariance_keys[key]][index])
            line.set_data(ellipse[0], ellipse[1])
        gps_line.set_data(history["gps"][:index + 1, 0], history["gps"][:index + 1, 1])
        for key, line in error_lines.items():
            line.set_data(history["time"][:index + 1], errors[key][:index + 1])
        radius_line.set_data(history["time"][:index + 1], position_radius[:index + 1])
        for i, line in enumerate(bias_lines):
            line.set_data(history["time"][:index + 1], bias_estimates[:index + 1, i])
            bias_bands[i].set_data(history["time"][:index + 1],
                                   bias_lower[:index + 1, i], bias_upper[:index + 1, i])
        time = history["time"][index]
        status = "GPS unavailable" if outage is not None and outage[0] <= time < outage[1] else "GPS available (1 Hz)"
        title.set_text(f"{animation_title}\n{time:.1f} s — {status}")
        return [*lines.values(), *current_points.values(), *ellipse_lines.values(),
                gps_line, *error_lines.values(), radius_line, *bias_lines, *bias_bands, title]

    frames = list(range(0, len(history["time"]), 5))
    if frames[-1] != len(history["time"]) - 1:
        frames.append(len(history["time"]) - 1)
    return FuncAnimation(fig, update, frames=frames, interval=50, repeat=False)


def main():
    history = simulate(duration=SIM_TIME)
    for key in ["estimate", "no_bias_estimate", "dead_reckoning"]:
        error = history[key][:, :2] - history["truth"][:, :2]
        print(f"{key} position RMSE: {np.sqrt(np.mean(np.sum(error**2, axis=1))):.2f} m")
    animation = create_animation(history) if show_animation else None
    if animation is not None:
        plt.show()
    return history


if __name__ == "__main__":
    main()
