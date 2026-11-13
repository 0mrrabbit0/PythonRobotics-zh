from abc import ABC, abstractmethod
from dataclasses import dataclass
from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    Position,
)
from PathPlanning.TimeBasedPathPlanning.Node import NodePath
import random
import numpy.random as numpy_random

# 固定随机种子以保证可复现
RANDOM_SEED = 50
random.seed(RANDOM_SEED)
numpy_random.seed(RANDOM_SEED)

class SingleAgentPlanner(ABC):
    """
    单智能体规划器基类
    """
    
    @staticmethod
    @abstractmethod
    def plan(grid: Grid, start: Position, goal: Position, verbose: bool = False) -> NodePath:
        pass

@dataclass
class StartAndGoal:
    # 该智能体的索引
    index: int
    # 机器人的起点位置
    start: Position
    # 机器人的目标点位置
    goal: Position

    def distance_start_to_goal(self) -> float:
        return pow(self.goal.x - self.start.x, 2) + pow(self.goal.y - self.start.y, 2)

class MultiAgentPlanner(ABC):
    """
    多智能体规划器基类
    """       
    
    @staticmethod
    @abstractmethod
    def plan(grid: Grid, start_and_goal_positions: list[StartAndGoal], single_agent_planner_class: SingleAgentPlanner, verbose: bool = False) -> tuple[list[StartAndGoal], list[NodePath]]:
        """
        为所有智能体规划。返回的路径顺序与返回的 `StartAndGoal` 对象列表相对应
        """
        pass