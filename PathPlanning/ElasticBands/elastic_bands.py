"""
弹性带

作者: Wang Zheng (@Aglargil)

参考文献:

- [Elastic Bands: Connecting Path Planning and Control]
(http://www8.cs.umu.se/research/ifor/dl/Control/elastic%20bands.pdf)
"""

import numpy as np
import sys
import pathlib
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

from Mapping.DistanceMap.distance_map import compute_sdf_scipy

# 弹性带参数
MAX_BUBBLE_RADIUS = 100
MIN_BUBBLE_RADIUS = 10
RHO0 = 20.0  # 施加斥力的最大距离
KC = 0.05  # 收缩力增益
KR = -0.1  # 斥力增益
LAMBDA = 0.7  # 重叠约束因子
STEP_SIZE = 3.0  # 计算梯度的步长

# 可视化参数
ENABLE_PLOT = True
# ENABLE_INTERACTIVE 为 True 时，用户可左键添加障碍物，
# 右键添加路径点，中键开始规划
ENABLE_INTERACTIVE = False
# ENABLE_SAVE_DATA 为 True 时，可将交互模式下用户添加的
# 路径与障碍物保存到文件
ENABLE_SAVE_DATA = False
MAX_ITER = 50


class Bubble:
    def __init__(self, position, radius):
        self.pos = np.array(position)  # 气泡中心坐标 [x, y]
        self.radius = radius  # 安全距离半径 ρ(b)
        if self.radius > MAX_BUBBLE_RADIUS:
            self.radius = MAX_BUBBLE_RADIUS
        if self.radius < MIN_BUBBLE_RADIUS:
            self.radius = MIN_BUBBLE_RADIUS


class ElasticBands:
    def __init__(
        self,
        initial_path,
        obstacles,
        rho0=RHO0,
        kc=KC,
        kr=KR,
        lambda_=LAMBDA,
        step_size=STEP_SIZE,
    ):
        self.distance_map = compute_sdf_scipy(obstacles)
        self.bubbles = [
            Bubble(p, self.compute_rho(p)) for p in initial_path
        ]  # 初始化气泡链
        self.kc = kc  # 收缩力增益
        self.kr = kr  # 斥力增益
        self.rho0 = rho0  # 施加斥力的最大距离
        self.lambda_ = lambda_  # 重叠约束因子
        self.step_size = step_size  # 计算梯度的步长
        self._maintain_overlap()

    def compute_rho(self, position):
        """计算该位置的距离场值"""
        return self.distance_map[int(position[0]), int(position[1])]

    def contraction_force(self, i):
        """计算第 i 个气泡的内部收缩力"""
        if i == 0 or i == len(self.bubbles) - 1:
            return np.zeros(2)

        prev = self.bubbles[i - 1].pos
        next_ = self.bubbles[i + 1].pos
        current = self.bubbles[i].pos

        # f_c = kc * ( (prev-current)/|prev-current| + (next-current)/|next-current| )
        dir_prev = (prev - current) / (np.linalg.norm(prev - current) + 1e-6)
        dir_next = (next_ - current) / (np.linalg.norm(next_ - current) + 1e-6)
        return self.kc * (dir_prev + dir_next)

    def repulsive_force(self, i):
        """计算第 i 个气泡的外部斥力"""
        h = self.step_size  # 步长
        b = self.bubbles[i].pos
        rho = self.bubbles[i].radius

        if rho >= self.rho0:
            return np.zeros(2)

        # 梯度 ∂ρ/∂b 的有限差分近似
        dx = np.array([h, 0])
        dy = np.array([0, h])
        grad_x = (self.compute_rho(b - dx) - self.compute_rho(b + dx)) / (2 * h)
        grad_y = (self.compute_rho(b - dy) - self.compute_rho(b + dy)) / (2 * h)
        grad = np.array([grad_x, grad_y])

        return self.kr * (self.rho0 - rho) * grad

    def update_bubbles(self):
        """更新气泡位置"""
        new_bubbles = []
        for i in range(len(self.bubbles)):
            if i == 0 or i == len(self.bubbles) - 1:
                new_bubbles.append(self.bubbles[i])  # 固定起点与终点
                continue

            f_total = self.contraction_force(i) + self.repulsive_force(i)
            v = self.bubbles[i - 1].pos - self.bubbles[i + 1].pos

            # 去除切向分量
            f_star = f_total - f_total * v * v / (np.linalg.norm(v) ** 2 + 1e-6)

            alpha = self.bubbles[i].radius  # 自适应步长
            new_pos = self.bubbles[i].pos + alpha * f_star
            new_pos = np.clip(new_pos, 0, 499)
            new_radius = self.compute_rho(new_pos)

            # 更新气泡并维持重叠约束
            new_bubble = Bubble(new_pos, new_radius)
            new_bubbles.append(new_bubble)

        self.bubbles = new_bubbles
        self._maintain_overlap()

    def _maintain_overlap(self):
        """维持气泡链连续性（简化的插入/删除机制）"""
        # 插入气泡
        i = 0
        while i < len(self.bubbles) - 1:
            bi, bj = self.bubbles[i], self.bubbles[i + 1]
            dist = np.linalg.norm(bi.pos - bj.pos)
            if dist > self.lambda_ * (bi.radius + bj.radius):
                new_pos = (bi.pos + bj.pos) / 2
                rho = self.compute_rho(
                    new_pos
                )  # 使用环境模型计算新半径
                self.bubbles.insert(i + 1, Bubble(new_pos, rho))
                i += 2  # 跳过已处理区域
            else:
                i += 1

        # 删除冗余气泡
        i = 1
        while i < len(self.bubbles) - 1:
            prev = self.bubbles[i - 1]
            next_ = self.bubbles[i + 1]
            dist = np.linalg.norm(prev.pos - next_.pos)
            if dist <= self.lambda_ * (prev.radius + next_.radius):
                del self.bubbles[i]  # 若冗余则删除
            else:
                i += 1


class ElasticBandsVisualizer:
    def __init__(self):
        self.obstacles = np.zeros((500, 500))
        self.obstacles_points = []
        self.path_points = []
        self.elastic_band = None
        self.running = True

        if ENABLE_PLOT:
            self.fig, self.ax = plt.subplots(figsize=(8, 8))
            self.fig.canvas.mpl_connect("close_event", self.on_close)
            self.ax.set_xlim(0, 500)
            self.ax.set_ylim(0, 500)

        if ENABLE_INTERACTIVE:
            self.path_points = []  # 添加用于存储路径点的列表
            # 连接鼠标事件
            self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        else:
            self.path_points = np.load(pathlib.Path(__file__).parent / "path.npy")
            self.obstacles_points = np.load(
                pathlib.Path(__file__).parent / "obstacles.npy"
            )
            for x, y in self.obstacles_points:
                self.add_obstacle(x, y)
            self.plan_path()

        self.plot_background()

    def on_close(self, event):
        """处理窗口关闭事件"""
        self.running = False
        plt.close("all")  # 关闭所有图形窗口

    def plot_background(self):
        """绘制背景栅格"""
        if not ENABLE_PLOT or not self.running:
            return

        self.ax.cla()
        self.ax.set_xlim(0, 500)
        self.ax.set_ylim(0, 500)
        self.ax.grid(True)

        if ENABLE_INTERACTIVE:
            self.ax.set_title(
                "Elastic Bands Path Planning\n"
                "Left click: Add obstacles\n"
                "Right click: Add path points\n"
                "Middle click: Start planning",
                pad=20,
            )
        else:
            self.ax.set_title("Elastic Bands Path Planning", pad=20)

        if self.path_points:
            self.ax.plot(
                [p[0] for p in self.path_points],
                [p[1] for p in self.path_points],
                "yo",
                markersize=8,
            )

        self.ax.imshow(self.obstacles.T, origin="lower", cmap="binary", alpha=0.8)
        self.ax.plot([], [], color="black", label="obstacles")
        if self.elastic_band is not None:
            path = [b.pos.tolist() for b in self.elastic_band.bubbles]
            path = np.array(path)
            self.ax.plot(path[:, 0], path[:, 1], "b-", linewidth=2, label="path")

            for bubble in self.elastic_band.bubbles:
                circle = Circle(
                    bubble.pos, bubble.radius, fill=False, color="g", alpha=0.3
                )
                self.ax.add_patch(circle)
                self.ax.plot(bubble.pos[0], bubble.pos[1], "bo", markersize=10)
            self.ax.plot([], [], color="green", label="bubbles")

        self.ax.legend(loc="upper right")
        plt.draw()
        plt.pause(0.01)

    def add_obstacle(self, x, y):
        """在给定坐标处添加障碍物"""
        size = 30  # 正方形边长
        half_size = size // 2
        x_start = max(0, x - half_size)
        x_end = min(self.obstacles.shape[0], x + half_size)
        y_start = max(0, y - half_size)
        y_end = min(self.obstacles.shape[1], y + half_size)
        self.obstacles[x_start:x_end, y_start:y_end] = 1

    def on_click(self, event):
        """处理鼠标点击事件"""
        if event.inaxes != self.ax:
            return

        x, y = int(event.xdata), int(event.ydata)

        if event.button == 1:  # 左键添加障碍物
            self.add_obstacle(x, y)
            self.obstacles_points.append([x, y])

        elif event.button == 3:  # 右键添加路径点
            self.path_points.append([x, y])

        elif event.button == 2:  # 中键结束路径输入并开始规划
            if len(self.path_points) >= 2:
                if ENABLE_SAVE_DATA:
                    np.save(
                        pathlib.Path(__file__).parent / "path.npy", self.path_points
                    )
                    np.save(
                        pathlib.Path(__file__).parent / "obstacles.npy",
                        self.obstacles_points,
                    )
                self.plan_path()

        self.plot_background()

    def plan_path(self):
        """规划路径"""

        initial_path = self.path_points
        # 创建弹性带对象并优化
        self.elastic_band = ElasticBands(initial_path, self.obstacles)
        for _ in range(MAX_ITER):
            self.elastic_band.update_bubbles()
            self.path_points = [b.pos for b in self.elastic_band.bubbles]
            self.plot_background()


if __name__ == "__main__":
    _ = ElasticBandsVisualizer()
    if ENABLE_PLOT:
        plt.show(block=True)
