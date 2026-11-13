"""
用于多智能体路径规划的基于优先级的规划器。
规划器生成一个规划顺序，并按该顺序为各机器人生成规划。每条已规划
路径都会在栅格中预留，此后的所有规划都必须避开该路径。

算法概述见本文第 III 节: https://pure.tudelft.nl/ws/portalfiles/portal/67074672/07138650.pdf
"""

import numpy as np
from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    Interval,
    ObstacleArrangement,
    Position,
)
from PathPlanning.TimeBasedPathPlanning.BaseClasses import MultiAgentPlanner, StartAndGoal
from PathPlanning.TimeBasedPathPlanning.Node import NodePath
from PathPlanning.TimeBasedPathPlanning.BaseClasses import SingleAgentPlanner
from PathPlanning.TimeBasedPathPlanning.SafeInterval import SafeIntervalPathPlanner
from PathPlanning.TimeBasedPathPlanning.Plotting import PlotNodePaths
import time

class PriorityBasedPlanner(MultiAgentPlanner):

    @staticmethod
    def plan(grid: Grid, start_and_goals: list[StartAndGoal], single_agent_planner_class: SingleAgentPlanner, verbose: bool = False) -> tuple[list[StartAndGoal], list[NodePath]]:
        """
        为 `start_and_goals` 列表中的每个智能体生成从起点到目标点的路径。
        返回重排后的 StartAndGoal 组合，以及路径规划列表。规划的顺序
        与 `start_and_goals` 列表的顺序相对应。
        """
        print(f"Using single-agent planner: {single_agent_planner_class}")

        # 预留初始位置
        for start_and_goal in start_and_goals:
            grid.reserve_position(start_and_goal.start, start_and_goal.index, Interval(0, 10))

        # 按起点到目标点距离的降序进行规划
        start_and_goals = sorted(start_and_goals,
                    key=lambda item: item.distance_start_to_goal(),
                    reverse=True)

        paths = []
        for start_and_goal in start_and_goals:
            if verbose:
                print(f"\nPlanning for agent:  {start_and_goal}" )

            grid.clear_initial_reservation(start_and_goal.start, start_and_goal.index)
            path = single_agent_planner_class.plan(grid, start_and_goal.start, start_and_goal.goal, verbose)

            if path is None:
                print(f"Failed to find path for {start_and_goal}")
                return []

            agent_index = start_and_goal.index
            grid.reserve_path(path, agent_index)
            paths.append(path)

        return (start_and_goals, paths)

verbose = False
show_animation = True
def main():
    grid_side_length = 21

    start_and_goals = [StartAndGoal(i, Position(1, i), Position(19, 19-i)) for i in range(1, 16)]
    obstacle_avoid_points = [pos for item in start_and_goals for pos in (item.start, item.goal)]

    grid = Grid(
        np.array([grid_side_length, grid_side_length]),
        num_obstacles=250,
        obstacle_avoid_points=obstacle_avoid_points,
        # obstacle_arrangement=ObstacleArrangement.NARROW_CORRIDOR,
        obstacle_arrangement=ObstacleArrangement.ARRANGEMENT1,
        # obstacle_arrangement=ObstacleArrangement.RANDOM,
    )

    start_time = time.time()
    start_and_goals, paths = PriorityBasedPlanner.plan(grid, start_and_goals, SafeIntervalPathPlanner, verbose)

    runtime = time.time() - start_time
    print(f"\nPlanning took: {runtime:.5f} seconds")

    if verbose:
        print(f"Paths:")
        for path in paths:
            print(f"{path}\n")

    if not show_animation:
        return

    PlotNodePaths(grid, start_and_goals, paths)

if __name__ == "__main__":
    main()