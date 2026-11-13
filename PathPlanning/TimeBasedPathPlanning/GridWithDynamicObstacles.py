"""
本文件实现带三维预留矩阵的栅格，维度为 x、y 与时间。其中
还包含用于生成在栅格中运动的动态障碍物的基础设施。障碍物的路径
在创建时存入预留矩阵。
"""
import numpy as np
import matplotlib.pyplot as plt
from enum import Enum
from dataclasses import dataclass
from PathPlanning.TimeBasedPathPlanning.Node import NodePath, Position

@dataclass
class Interval:
    start_time: int
    end_time: int

class ObstacleArrangement(Enum):
    # 随机障碍物位置与运动
    RANDOM = 0
    # 障碍物在栅格中心沿 y 排成一行，并沿 x 左右往复运动
    ARRANGEMENT1 = 1
    # 静态障碍物布置
    NARROW_CORRIDOR = 2

"""
Generates a 2d numpy array with lists for elements.
"""
def empty_2d_array_of_lists(x: int, y: int) -> np.ndarray:
    arr = np.empty((x, y), dtype=object)
    # 逐个元素赋值——np.full 会让所有元素引用同一个列表
    arr[:] = [[[] for _ in range(y)] for _ in range(x)]
    return arr

class Grid:
    # 在构造函数中设置
    grid_size: np.ndarray
    reservation_matrix: np.ndarray
    obstacle_paths: list[list[Position]] = []
    # 障碍物永远不会占据这些点。用于避免无解情形
    obstacle_avoid_points: list[Position] = []

    # 仿真中的时间步数
    time_limit: int

    # 日志控制
    verbose = False

    def __init__(
        self,
        grid_size: np.ndarray,
        num_obstacles: int = 40,
        obstacle_avoid_points: list[Position] = [],
        obstacle_arrangement: ObstacleArrangement = ObstacleArrangement.RANDOM,
        time_limit: int = 100,
    ):
        self.obstacle_avoid_points = obstacle_avoid_points
        self.time_limit = time_limit
        self.grid_size = grid_size
        self.reservation_matrix = np.zeros((grid_size[0], grid_size[1], self.time_limit))

        if num_obstacles > self.grid_size[0] * self.grid_size[1]:
            raise Exception("Number of obstacles is greater than grid size!")

        if obstacle_arrangement == ObstacleArrangement.RANDOM:
            self.obstacle_paths = self.generate_dynamic_obstacles(num_obstacles)
        elif obstacle_arrangement == ObstacleArrangement.ARRANGEMENT1:
            self.obstacle_paths = self.obstacle_arrangement_1(num_obstacles)
        elif obstacle_arrangement == ObstacleArrangement.NARROW_CORRIDOR:
            self.obstacle_paths = self.generate_narrow_corridor_obstacles(num_obstacles)

        for i, path in enumerate(self.obstacle_paths):
            obs_idx = i + 1  # 避免使用 0——0 表示栅格中的空闲空间
            for t, position in enumerate(path):
                # 在该时间步预留旧位置与新位置
                if t > 0:
                    self.reservation_matrix[path[t - 1].x, path[t - 1].y, t] = obs_idx
                self.reservation_matrix[position.x, position.y, t] = obs_idx

    """
    Generate dynamic obstacles that move around the grid. Initial positions and movements are random
    """
    def generate_dynamic_obstacles(self, obs_count: int) -> list[list[Position]]:
        obstacle_paths = []
        for _ in range(0, obs_count):
            # 采样直到找到空闲的起始空间
            initial_position = self.sample_random_position()
            while not self.valid_obstacle_position(initial_position, 0):
                initial_position = self.sample_random_position()

            positions = [initial_position]
            if self.verbose:
                print("Obstacle initial position: ", initial_position)

            # 使障碍物大多停留原地——运动过多会导致规划场景混乱
            # 不便于观察
            weights = [0.05, 0.05, 0.05, 0.05, 0.8]
            diffs = [
                Position(0, 1),
                Position(0, -1),
                Position(1, 0),
                Position(-1, 0),
                Position(0, 0),
            ]

            for t in range(1, self.time_limit - 1):
                sampled_indices = np.random.choice(
                    len(diffs), size=5, replace=False, p=weights
                )
                rand_diffs = [diffs[i] for i in sampled_indices]

                valid_position = None
                for diff in rand_diffs:
                    new_position = positions[-1] + diff

                    if not self.valid_obstacle_position(new_position, t):
                        continue

                    valid_position = new_position
                    break

                # 障碍物陷入无法移动的情形——留在原地
                #   -> 若其他障碍物的路径挡住了该障碍物，则可能发生这种情况
                if valid_position is None:
                    valid_position = positions[-1]

                positions.append(valid_position)

            obstacle_paths.append(positions)

        return obstacle_paths

    """
    Generate a line of obstacles in y at the center of the grid that move side-to-side in x
    Bottom half start moving right, top half start moving left. If `obs_count` is less than the length of
    the grid, only the first `obs_count` obstacles will be generated.
    """
    def obstacle_arrangement_1(self, obs_count: int) -> list[list[Position]]:
        obstacle_paths = []
        half_grid_x = self.grid_size[0] // 2
        half_grid_y = self.grid_size[1] // 2

        for y_idx in range(0, min(obs_count, self.grid_size[1])):
            moving_right = y_idx < half_grid_y
            position = Position(half_grid_x, y_idx)
            path = [position]

            for t in range(1, self.time_limit - 1):
                # 每隔一个时间步停留原地
                if t % 2 == 0:
                    path.append(position)
                    continue

                # 首先检查是否应改变方向（位于栅格边缘时）
                if (moving_right and position.x == self.grid_size[0] - 1) or (
                    not moving_right and position.x == 0
                ):
                    moving_right = not moving_right
                # 沿该方向前进一步
                position = Position(
                    position.x + (1 if moving_right else -1), position.y
                )
                path.append(position)

            obstacle_paths.append(path)

        return obstacle_paths
    
    def generate_narrow_corridor_obstacles(self, obs_count: int) -> list[list[Position]]:
        obstacle_paths = []

        for y in range(0, self.grid_size[1]):
            if y > obs_count:
                break

            if y == self.grid_size[1] // 2:
                # 跳过中间一行
                continue

            obstacle_path = []
            x = self.grid_size[0] // 2 # 栅格中部
            for t in range(0, self.time_limit - 1):
                obstacle_path.append(Position(x, y))

            obstacle_paths.append(obstacle_path)

        return obstacle_paths

    """
    Check if the given position is valid at time t

    input:
        position (Position): (x, y) position
        t (int): time step

    output:
        bool: True if position/time combination is valid, False otherwise
    """
    def valid_position(self, position: Position, t: int) -> bool:
        # 检查位置是否在栅格内
        if not self.inside_grid_bounds(position):
            return False

        # 检查位置在时刻 t 是否未被占用
        return self.reservation_matrix[position.x, position.y, t] == 0

    """
    Returns True if the given position is valid at time t and is not in the set of obstacle_avoid_points
    """
    def valid_obstacle_position(self, position: Position, t: int) -> bool:
        return (
            self.valid_position(position, t)
            and position not in self.obstacle_avoid_points
        )

    """
    Returns True if the given position is within the grid's boundaries
    """
    def inside_grid_bounds(self, position: Position) -> bool:
        return (
            position.x >= 0
            and position.x < self.grid_size[0]
            and position.y >= 0
            and position.y < self.grid_size[1]
        )

    """
    Sample a random position that is within the grid's boundaries

    output:
        Position: (x, y) position
    """
    def sample_random_position(self) -> Position:
        return Position(
            np.random.randint(0, self.grid_size[0]),
            np.random.randint(0, self.grid_size[1]),
        )

    """
    Returns a tuple of (x_positions, y_positions) of the obstacles at time t
    """
    def get_obstacle_positions_at_time(self, t: int) -> tuple[list[int], list[int]]:
        x_positions = []
        y_positions = []
        for obs_path in self.obstacle_paths:
            x_positions.append(obs_path[t].x)
            y_positions.append(obs_path[t].y)
        return (x_positions, y_positions)

    """
    Returns safe intervals for each cell.
    """
    def get_safe_intervals(self) -> np.ndarray:
        intervals = empty_2d_array_of_lists(self.grid_size[0], self.grid_size[1])
        for x in range(intervals.shape[0]):
            for y in range(intervals.shape[1]):
                intervals[x, y] = self.get_safe_intervals_at_cell(Position(x, y))

        return intervals

    """
    Generate the safe intervals for a given cell. The intervals will be in order of start time.
    ex: Interval (2, 3) will be before Interval (4, 5)
    """
    def get_safe_intervals_at_cell(self, cell: Position) -> list[Interval]:
        vals = self.reservation_matrix[cell.x, cell.y, :]
        # 找出数组中为零的位置
        zero_mask = (vals == 0)

        # 识别零元素与非零元素之间的跳变
        diff = np.diff(zero_mask.astype(int))

        # 起始索引：零段开始处（非零之后的下一位）
        start_indices = np.where(diff == 1)[0] + 1

        # 结束索引：零段结束处（非零之前的一位）
        end_indices = np.where(diff == -1)[0]

        # 处理数组以零开始或结束的边界情形
        if zero_mask[0]:  # 若第一个元素为零，将索引 0 加入 start_indices
            start_indices = np.insert(start_indices, 0, 0)
        if zero_mask[-1]:  # 若最后一个元素为零，将最后一个索引加入 end_indices
            end_indices = np.append(end_indices, len(vals) - 1)

        # 构造（第一个零，最后一个零）配对
        intervals = [Interval(int(start), int(end)) for start, end in zip(start_indices, end_indices)]

        # 去掉仅在一个时间步内空闲的区间。这类区间没有足够时间，
        # 因为进入和离开栅格各需要 1 个时间步，并且在
        # 进入该栅格的时间步以及离开该栅格的时间步，该栅格都被视为占用。
        intervals = [interval for interval in intervals if interval.start_time != interval.end_time]
        return intervals
    
    """
    Reserve an agent's path in the grid. Raises an exception if the agent's index is 0, or if a position is
    already reserved by a different agent.
    """
    def reserve_path(self, node_path: NodePath, agent_index: int):
        if agent_index == 0:
            raise Exception("Agent index cannot be 0")
        
        for i, node in enumerate(node_path.path):
            reservation_finish_time = node.time + 1
            if i < len(node_path.path) - 1:
                reservation_finish_time = node_path.path[i + 1].time

            self.reserve_position(node.position, agent_index, Interval(node.time, reservation_finish_time))

    """
    Reserve a position for the provided agent during the provided time interval.
    Raises an exception if the agent's index is 0, or if the position is already reserved by a different agent during the interval.
    """
    def reserve_position(self, position: Position, agent_index: int, interval: Interval):
        if agent_index == 0:
            raise Exception("Agent index cannot be 0")

        for t in range(interval.start_time, interval.end_time + 1):
            current_reserver = self.reservation_matrix[position.x, position.y, t]
            if current_reserver not in [0, agent_index]:
                raise Exception(
                    f"Agent {agent_index} tried to reserve a position already reserved by another agent: {position} at time {t}, reserved by {current_reserver}"
                )
            self.reservation_matrix[position.x, position.y, t] = agent_index

    """
    Clears the initial reservation for an agent by clearing reservations at its start position with its index for
    from time 0 to the time limit.
    """
    def clear_initial_reservation(self, position: Position, agent_index: int):
        for t in range(self.time_limit):
            if self.reservation_matrix[position.x, position.y, t] == agent_index:
                self.reservation_matrix[position.x, position.y, t] = 0

show_animation = True

def main():
    grid = Grid(
        np.array([11, 11]),
        num_obstacles=10,
        obstacle_arrangement=ObstacleArrangement.ARRANGEMENT1,
    )

    if not show_animation:
        return

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(
        autoscale_on=False,
        xlim=(0, grid.grid_size[0] - 1),
        ylim=(0, grid.grid_size[1] - 1),
    )
    ax.set_aspect("equal")
    ax.grid()
    ax.set_xticks(np.arange(0, 11, 1))
    ax.set_yticks(np.arange(0, 11, 1))
    (obs_points,) = ax.plot([], [], "ro", ms=15)

    # 按 esc 键停止仿真。
    plt.gcf().canvas.mpl_connect(
        "key_release_event", lambda event: [exit(0) if event.key == "escape" else None]
    )

    for i in range(0, grid.time_limit - 1):
        obs_positions = grid.get_obstacle_positions_at_time(i)
        obs_points.set_data(obs_positions[0], obs_positions[1])
        plt.pause(0.2)
    plt.show()


if __name__ == "__main__":
    main()
