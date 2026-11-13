"""
安全时间区间路径规划器
    本脚本实现带动态障碍物的二维栅格安全时间区间路径规划器。它比
    时空 A* 更快，因为通过预先计算每个位置上由相邻
    安全时间步组成的区域（“安全时间区间”），减少了冗余节点扩展。这使得算法可以跳过扩展
    那些位于先前已经访问过的区间内的节点。

    Reference: https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf
"""

import numpy as np
from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    Interval,
    ObstacleArrangement,
    Position,
    empty_2d_array_of_lists,
)
from PathPlanning.TimeBasedPathPlanning.BaseClasses import SingleAgentPlanner
from PathPlanning.TimeBasedPathPlanning.Node import Node, NodePath
from PathPlanning.TimeBasedPathPlanning.Plotting import PlotNodePath

import heapq
from dataclasses import dataclass
from functools import total_ordering
import time

@dataclass()
# 注：使用 total_ordering，而不是在 @dataclass 装饰器中加入 `order=True`，因为
#     该类需要重写 __lt__ 与 __eq__，以忽略 parent_index。父节点
#     索引和区间成员变量仅用于记录算法找到的路径，
#     不影响节点的优劣。
@total_ordering
class SIPPNode(Node):
    interval: Interval

@dataclass
class EntryTimeAndInterval:
    entry_time: int
    interval: Interval

class SafeIntervalPathPlanner(SingleAgentPlanner):

    """
    根据已加载的问题描述生成规划。若找不到路径则抛出异常。
    Arguments:
        verbose (bool): 设为 True 时打印调试信息
    """
    @staticmethod
    def plan(grid: Grid, start: Position, goal: Position, verbose: bool = False) -> NodePath:

        safe_intervals = grid.get_safe_intervals()

        open_set: list[SIPPNode] = []
        first_node_interval = safe_intervals[start.x, start.y][0]
        heapq.heappush(
            open_set, SIPPNode(start, 0, SafeIntervalPathPlanner.calculate_heuristic(start, goal), -1, first_node_interval)
        )

        expanded_list: list[SIPPNode] = []
        visited_intervals = empty_2d_array_of_lists(grid.grid_size[0], grid.grid_size[1])
        while open_set:
            expanded_node: SIPPNode = heapq.heappop(open_set)
            if verbose:
                print("Expanded node:", expanded_node)

            if expanded_node.time + 1 >= grid.time_limit:
                if verbose:
                    print(f"\tSkipping node that is past time limit: {expanded_node}")
                continue

            if expanded_node.position == goal:
                if verbose:
                    print(f"Found path to goal after {len(expanded_list)} expansions")

                path = []
                path_walker: SIPPNode = expanded_node
                while True:
                    path.append(path_walker)
                    if path_walker.parent_index == -1:
                        break
                    path_walker = expanded_list[path_walker.parent_index]

                # 反转路径，使其从起点指向目标点
                path.reverse()
                return NodePath(path, len(expanded_list))

            expanded_idx = len(expanded_list)
            expanded_list.append(expanded_node)
            entry_time_and_node = EntryTimeAndInterval(expanded_node.time, expanded_node.interval)
            add_entry_to_visited_intervals_array(entry_time_and_node, visited_intervals, expanded_node)

            for child in SafeIntervalPathPlanner.generate_successors(grid, goal, expanded_node, expanded_idx, safe_intervals, visited_intervals):
                heapq.heappush(open_set, child)

        raise Exception("No path found")

    """
    Generate list of possible successors of the provided `parent_node` that are worth expanding
    """
    @staticmethod
    def generate_successors(
        grid: Grid, goal: Position, parent_node: SIPPNode, parent_node_idx: int, intervals: np.ndarray, visited_intervals: np.ndarray
    ) -> list[SIPPNode]:
        new_nodes = []
        diffs = [
            Position(0, 0),
            Position(1, 0),
            Position(-1, 0),
            Position(0, 1),
            Position(0, -1),
        ]
        for diff in diffs:
            new_pos = parent_node.position + diff
            if not grid.inside_grid_bounds(new_pos):
                continue

            current_interval = parent_node.interval

            new_cell_intervals: list[Interval] = intervals[new_pos.x, new_pos.y]
            for interval in new_cell_intervals:
                # 若该区间在当前区间结束之后才开始，则中断
                # 假设：区间按开始时间排序，因此之后的所有区间也会满足该条件
                if interval.start_time > current_interval.end_time:
                    break

                # 若该区间在当前区间开始之前就结束，则跳过
                if interval.end_time < current_interval.start_time:
                    continue

                # 若已在该区间内扩展过进入时间小于等于当前时间的节点，则跳过
                better_node_expanded = False
                for visited in visited_intervals[new_pos.x, new_pos.y]:
                    if interval == visited.interval and visited.entry_time <= parent_node.time + 1:
                        better_node_expanded = True
                        break
                if better_node_expanded:
                    continue

                # 此时已知存在值得扩展的节点。在能够进入
                # 新区间的最早时刻生成后继
                for possible_t in range(max(parent_node.time + 1, interval.start_time), min(current_interval.end_time, interval.end_time)):
                    if grid.valid_position(new_pos, possible_t):
                        new_nodes.append(SIPPNode(
                            new_pos,
                            # 进入时刻取区间开始时间与父节点时间 + 1 的较大值（尽快到达）
                            max(interval.start_time, parent_node.time + 1),
                            SafeIntervalPathPlanner.calculate_heuristic(new_pos, goal),
                            parent_node_idx,
                            interval,
                        ))
                        # 中断，因为此后的所有 t 都会产生代价更高、启发式相同且位于同一区间的节点
                        break

        return new_nodes

    """
    Calculate the heuristic for a given position - Manhattan distance to the goal
    """
    @staticmethod
    def calculate_heuristic(position: Position, goal: Position) -> int:
        diff = goal - position
        return abs(diff.x) + abs(diff.y)


"""
Adds a new entry to the visited intervals array. If the entry is already present, the entry time is updated if the new
entry time is better. Otherwise, the entry is added to `visited_intervals` at the position of `expanded_node`.
"""
def add_entry_to_visited_intervals_array(entry_time_and_interval: EntryTimeAndInterval, visited_intervals: np.ndarray, expanded_node: SIPPNode):
    # 若记录已存在，且新进入时间更优，则更新进入时间
    for existing_entry_and_interval in visited_intervals[expanded_node.position.x, expanded_node.position.y]:
        if existing_entry_and_interval.interval == entry_time_and_interval.interval:
            existing_entry_and_interval.entry_time = min(existing_entry_and_interval.entry_time, entry_time_and_interval.entry_time)

    # 否则追加
    visited_intervals[expanded_node.position.x, expanded_node.position.y].append(entry_time_and_interval)


show_animation = True
verbose = False

def main():
    start = Position(1, 18)
    goal = Position(19, 19)
    grid_side_length = 21

    start_time = time.time()

    grid = Grid(
        np.array([grid_side_length, grid_side_length]),
        num_obstacles=250,
        obstacle_avoid_points=[start, goal],
        obstacle_arrangement=ObstacleArrangement.ARRANGEMENT1,
        # obstacle_arrangement=ObstacleArrangement.RANDOM,
    )

    path = SafeIntervalPathPlanner.plan(grid, start, goal, verbose)
    runtime = time.time() - start_time
    print(f"Planning took: {runtime:.5f} seconds")

    if verbose:
        print(f"Path: {path}")

    if not show_animation:
        return

    PlotNodePath(grid, start, goal, path)


if __name__ == "__main__":
    main()
