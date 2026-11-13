"""
粒子群优化（PSO）路径规划

作者：Anish (@anishk85)

参见维基百科条目 (https://en.wikipedia.org/wiki/Particle_swarm_optimization)

References:
    - Kennedy, J.; Eberhart, R. (1995). "粒子群优化"
    - Shi, Y.; Eberhart, R. (1998). "一种改进的粒子群优化器"
    - https://machinelearningmastery.com/a-gentle-introduction-to-particle-swarm-optimization/

本实现用 PSO 寻找无碰撞路径，将
路径规划视为优化问题：粒子探索
搜索空间，在避开障碍物的同时最小化到目标的距离。
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import signal
import sys
# 添加 show_animation 标志，以便与其他规划器保持一致
show_animation = True

def signal_handler(sig, frame):
    print("\nExiting...")
    plt.close("all")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class Particle:
    """表示 PSO 粒子群中的单个粒子。
    每个粒子维护当前位置、速度，以及搜索过程中发现的个体最优
    位置。粒子根据个体经验（认知分量）
    和群体知识（社会分量）更新速度，从而探索搜索空间。
    Attributes:
        search_bounds: 定义搜索空间的元组列表 [(x_min, x_max), (y_min, y_max)]
        max_velocity: 每个维度允许的最大速度（搜索空间范围的 5%）
        position: 搜索空间中的当前二维位置 [x, y]
        velocity: 当前速度向量 [vx, vy]
        personal_best_position: 迄今找到的个体最优位置
        personal_best_value: 个体最优位置处的适应度
        path: 该粒子访问过的全部位置列表
    """

    def __init__(self, search_bounds, spawn_bounds):
        self.search_bounds = search_bounds
        self.max_velocity = np.array([(b[1] - b[0]) * 0.05 for b in search_bounds])
        self.position = np.array([np.random.uniform(b[0], b[1]) for b in spawn_bounds])
        self.velocity = np.random.randn(2) * 0.1
        self.personal_best_position = self.position.copy()
        self.personal_best_value = np.inf
        self.path = [self.position.copy()]

    def update_velocity(self, gbest_pos, w, c1, c2):
        """使用 PSO 方程更新粒子速度：
        v = w*v + c1*r1*(personal_best - x) + c2*r2*(gbest - x)
        """
        r1 = np.random.rand(2)
        r2 = np.random.rand(2)
        cognitive = c1 * r1 * (self.personal_best_position - self.position)
        social = c2 * r2 * (gbest_pos - self.position)
        self.velocity = w * self.velocity + cognitive + social
        self.velocity = np.clip(self.velocity, -self.max_velocity, self.max_velocity)

    def update_position(self):
        self.position = self.position + self.velocity
        # 保持在边界内
        for i in range(2):
            self.position[i] = np.clip(
                self.position[i], self.search_bounds[i][0], self.search_bounds[i][1]
            )
        self.path.append(self.position.copy())

class PSOSwarm:

    def __init__(
        self, n_particles, max_iter, target, search_bounds, spawn_bounds, obstacles
    ):
        self.n_particles = n_particles
        self.max_iter = max_iter
        self.target = np.array(target)
        self.obstacles = obstacles
        self.search_bounds = search_bounds
        # PSO 参数
        self.w_start = 0.9  # 初始惯性权重
        self.w_end = 0.4  # 最终惯性权重
        self.c1 = 1.5  # 认知系数
        self.c2 = 1.5  # 社会系数
        # 初始化粒子
        self.particles = [
            Particle(search_bounds, spawn_bounds) for _ in range(n_particles)
        ]
        self.gbest_position = None
        self.gbest_value = np.inf
        self.gbest_path = []
        self.iteration = 0

    def fitness(self, pos):
        """计算适应度：到目标的距离 + 障碍物惩罚"""
        dist = np.linalg.norm(pos - self.target)
        # 障碍物惩罚
        penalty = 0
        for ox, oy, r in self.obstacles:
            obs_dist = np.linalg.norm(pos - np.array([ox, oy]))
            if obs_dist < r:
                penalty += 1000  # 在障碍物内部
            elif obs_dist < r + 5:
                penalty += 50 / (obs_dist - r + 0.1)  # 过于接近
        return dist + penalty

    def check_collision(self, start, end, obstacle):
        """使用直线与圆的相交检测，判断从 start 到 end 的路径是否碰到障碍物
            Args:
            start: 起始位置（numpy 数组）
            end: 终止位置（numpy 数组）
            obstacle: 元组 (ox, oy, r)，表示障碍物圆心与半径
        Returns:
            bool：检测到碰撞时为 True，否则为 False
        """
        ox, oy, r = obstacle
        center = np.array([ox, oy])
        # 直线与圆相交的向量计算
        d = end - start
        f = start - center
        a = np.dot(d, d)
        # 防止零长度步长导致 ZeroDivisionError
        if a < 1e-10:  # 近乎零长度的步长
            # 检查起点是否在障碍物内
            return np.linalg.norm(f) <= r
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - r * r
        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return False
        # 检查交点是否在线段上
        sqrt_discriminant = np.sqrt(discriminant)
        t1 = (-b - sqrt_discriminant) / (2 * a)
        t2 = (-b + sqrt_discriminant) / (2 * a)
        return (0 <= t1 <= 1) or (0 <= t2 <= 1)

    def step(self):
        """执行一次 PSO 迭代
        Returns:
            bool：算法应继续时为 True，已完成时为 False
        """
        if self.iteration >= self.max_iter:
            return False
        # 更新惯性权重（线性衰减）
        w = self.w_start - (self.w_start - self.w_end) * (
            self.iteration / self.max_iter
        )
        # 评估所有粒子
        for particle in self.particles:
            value = self.fitness(particle.position)
            # 更新个体最优
            if value < particle.personal_best_value:
                particle.personal_best_value = value
                particle.personal_best_position = particle.position.copy()
            # 更新全局最优
            if value < self.gbest_value:
                self.gbest_value = value
                self.gbest_position = particle.position.copy()
        if self.gbest_position is not None:
            self.gbest_path.append(self.gbest_position.copy())
        # 更新粒子
        for particle in self.particles:
            particle.update_velocity(self.gbest_position, w, self.c1, self.c2)
            # 预测下一位置
            next_pos = particle.position + particle.velocity
            # 检查碰撞
            collision = False
            for obs in self.obstacles:
                if self.check_collision(particle.position, next_pos, obs):
                    collision = True
                    break
            if collision:
                # 若检测到碰撞则减小速度
                particle.velocity *= 0.2
            particle.update_position()
        self.iteration += 1
        if show_animation and self.iteration % 20 == 0:
            print(
                f"Iteration {self.iteration}/{self.max_iter}, Best: {self.gbest_value:.2f}"
            )
        return True

def main():
    """运行 PSO 路径规划算法演示。
    本函数以下列设置演示基于 PSO 的路径规划：
    - 15 个粒子在 (-50,50) x (-50,50) 的搜索空间中探索
    - 起始区域：(-45,-45) 到 (-35,-35)
    - 目标：(40, 35)
    - 4 个圆形障碍物，并带碰撞避免
    - 实时可视化粒子收敛过程（当 show_animation=True）
    - 支持无界面模式以便测试（当 show_animation=False）
    算法最多运行 150 次迭代，显示粒子运动、
    个体/全局最优位置，以及不断演化的最优路径。
    """
    print(__file__ + " start!!")
    # 为无界面环境设置 matplotlib 后端
    if not show_animation:
        plt.switch_backend("Agg")  # 测试时使用非图形界面后端
    # 设置参数
    N_PARTICLES = 15
    MAX_ITER = 150
    SEARCH_BOUNDS = [(-50, 50), (-50, 50)]
    TARGET = [40, 35]
    SPAWN_AREA = [(-45, -35), (-45, -35)]
    OBSTACLES = [(10, 15, 8), (-20, 0, 12), (20, -25, 10), (-5, -30, 7)]
    swarm = PSOSwarm(
        n_particles=N_PARTICLES,
        max_iter=MAX_ITER,
        target=TARGET,
        search_bounds=SEARCH_BOUNDS,
        spawn_bounds=SPAWN_AREA,
        obstacles=OBSTACLES,
    )
    # pragma: no cover
    if show_animation:
        # 可视化设置
        signal.signal(signal.SIGINT, signal_handler)
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_xlim(SEARCH_BOUNDS[0])
        ax.set_ylim(SEARCH_BOUNDS[1])
        ax.set_title("PSO Path Planning with Collision Avoidance", fontsize=14)
        ax.grid(True, alpha=0.3)
        # 绘制障碍物
        for ox, oy, r in OBSTACLES:
            circle = patches.Circle((ox, oy), r, color="gray", alpha=0.7)
            ax.add_patch(circle)
        # 绘制起始区域
        spawn_rect = patches.Rectangle(
            (SPAWN_AREA[0][0], SPAWN_AREA[1][0]),
            SPAWN_AREA[0][1] - SPAWN_AREA[0][0],
            SPAWN_AREA[1][1] - SPAWN_AREA[1][0],
            linewidth=2,
            edgecolor="green",
            facecolor="green",
            alpha=0.2,
            label="Start Zone",
        )
        ax.add_patch(spawn_rect)
        # 绘制目标
        ax.plot(TARGET[0], TARGET[1], "r*", markersize=20, label="Target")
        # 初始化绘图元素
        particles_scatter = ax.scatter(
            [], [], c="blue", s=50, alpha=0.6, label="Particles"
        )
        gbest_scatter = ax.plot([], [], "yo", markersize=12, label="Best Position")[0]
        particle_paths = [
            ax.plot([], [], "b-", lw=0.5, alpha=0.2)[0] for _ in range(N_PARTICLES)
        ]
        gbest_path_line = ax.plot([], [], "y--", lw=2, alpha=0.8, label="Best Path")[0]
        iteration_text = ax.text(
            0.02,
            0.95,
            "",
            transform=ax.transAxes,
            fontsize=12,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
        )
        ax.legend(loc="upper right")
        def animate(frame):
            """matplotlib FuncAnimation 的动画函数"""
            if not swarm.step():
                return (
                    particles_scatter,
                    gbest_scatter,
                    gbest_path_line,
                    iteration_text,
                    *particle_paths,
                )
            # 更新粒子位置
            positions = np.array([p.position for p in swarm.particles])
            particles_scatter.set_offsets(positions)
            # 更新粒子路径
            for i, particle in enumerate(swarm.particles):
                if len(particle.path) > 1:
                    path = np.array(particle.path)
                    particle_paths[i].set_data(path[:, 0], path[:, 1])
            # 更新全局最优
            if swarm.gbest_position is not None:
                gbest_scatter.set_data(
                    [swarm.gbest_position[0]], [swarm.gbest_position[1]]
                )
                if len(swarm.gbest_path) > 1:
                    gbest = np.array(swarm.gbest_path)
                    gbest_path_line.set_data(gbest[:, 0], gbest[:, 1])
            # 更新文本
            iteration_text.set_text(
                f"Iteration: {swarm.iteration}/{MAX_ITER}\n"
                f"Best Fitness: {swarm.gbest_value:.2f}"
            )
            return (
                particles_scatter,
                gbest_scatter,
                gbest_path_line,
                iteration_text,
                *particle_paths,
            )
        # 创建动画并保存引用，防止被垃圾回收
        animation_ref = animation.FuncAnimation(
            fig, animate, frames=MAX_ITER, interval=100, blit=True, repeat=False
        )
        plt.tight_layout()
        plt.show()
        # 保留引用，防止被垃圾回收
        return animation_ref
    else:
        # 测试时不播放动画
        print("Running PSO algorithm without animation...")
        iteration_count = 0
        while swarm.step():
            iteration_count += 1
            if iteration_count >= MAX_ITER:
                break
        print(f"PSO completed after {iteration_count} iterations")
        print(f"Best fitness: {swarm.gbest_value:.2f}")
        if swarm.gbest_position is not None:
            print(
                f"Best position: [{swarm.gbest_position[0]:.2f}, {swarm.gbest_position[1]:.2f}]"
            )
        return None
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        plt.close("all")
        sys.exit(0)