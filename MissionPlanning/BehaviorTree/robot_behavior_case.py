"""
机器人行为树案例

本文件演示如何用行为树控制机器人行为。
"""

from behavior_tree import (
    BehaviorTreeFactory,
    Status,
    ActionNode,
)
import time
import random
import os


class CheckBatteryNode(ActionNode):
    """
    检查机器人电池电量的节点

    若电量低于阈值则返回 FAILURE，否则返回 SUCCESS
    """

    def __init__(self, name, threshold=20):
        super().__init__(name)
        self.threshold = threshold
        self.battery_level = 100  # 初始电量为 100%

    def tick(self):
        # 模拟电量下降
        self.battery_level -= random.randint(1, 5)
        print(f"Current battery level: {self.battery_level}%")

        if self.battery_level <= self.threshold:
            return Status.FAILURE
        return Status.SUCCESS


class ChargeBatteryNode(ActionNode):
    """
    为机器人电池充电的节点
    """

    def __init__(self, name, charge_rate=10):
        super().__init__(name)
        self.charge_rate = charge_rate
        self.charging_time = 0

    def tick(self):
        # 模拟充电过程
        if self.charging_time == 0:
            print("Starting to charge...")

        self.charging_time += 1
        charge_amount = self.charge_rate * self.charging_time

        if charge_amount >= 100:
            print("Charging complete! Battery level: 100%")
            self.charging_time = 0
            return Status.SUCCESS
        else:
            print(f"Charging in progress... Battery level: {min(charge_amount, 100)}%")
            return Status.RUNNING


class MoveToPositionNode(ActionNode):
    """
    移动到指定位置的节点
    """

    def __init__(self, name, position, move_duration=2):
        super().__init__(name)
        self.position = position
        self.move_duration = move_duration
        self.start_time = None

    def tick(self):
        if self.start_time is None:
            self.start_time = time.time()
            print(f"Starting movement to position {self.position}")

        elapsed_time = time.time() - self.start_time

        if elapsed_time >= self.move_duration:
            print(f"Arrived at position {self.position}")
            self.start_time = None
            return Status.SUCCESS
        else:
            print(
                f"Moving to position {self.position}... {int(elapsed_time / self.move_duration * 100)}% complete"
            )
            return Status.RUNNING


class DetectObstacleNode(ActionNode):
    """
    检测障碍物的节点
    """

    def __init__(self, name, obstacle_probability=0.3):
        super().__init__(name)
        self.obstacle_probability = obstacle_probability

    def tick(self):
        # 用随机概率模拟障碍物检测
        if random.random() < self.obstacle_probability:
            print("Obstacle detected!")
            return Status.SUCCESS
        else:
            print("No obstacle detected")
            return Status.FAILURE


class AvoidObstacleNode(ActionNode):
    """
    避开障碍物的节点
    """

    def __init__(self, name, avoid_duration=1.5):
        super().__init__(name)
        self.avoid_duration = avoid_duration
        self.start_time = None

    def tick(self):
        if self.start_time is None:
            self.start_time = time.time()
            print("Starting obstacle avoidance...")

        elapsed_time = time.time() - self.start_time

        if elapsed_time >= self.avoid_duration:
            print("Obstacle avoidance complete")
            self.start_time = None
            return Status.SUCCESS
        else:
            print("Avoiding obstacle...")
            return Status.RUNNING


class PerformTaskNode(ActionNode):
    """
    执行特定任务的节点
    """

    def __init__(self, name, task_name, task_duration=3):
        super().__init__(name)
        self.task_name = task_name
        self.task_duration = task_duration
        self.start_time = None

    def tick(self):
        if self.start_time is None:
            self.start_time = time.time()
            print(f"Starting task: {self.task_name}")

        elapsed_time = time.time() - self.start_time

        if elapsed_time >= self.task_duration:
            print(f"Task complete: {self.task_name}")
            self.start_time = None
            return Status.SUCCESS
        else:
            print(
                f"Performing task: {self.task_name}... {int(elapsed_time / self.task_duration * 100)}% complete"
            )
            return Status.RUNNING


def create_robot_behavior_tree():
    """
    创建机器人行为树
    """

    factory = BehaviorTreeFactory()

    # 注册自定义节点
    factory.register_node_builder(
        "CheckBattery",
        lambda node: CheckBatteryNode(
            node.attrib.get("name", "CheckBattery"),
            int(node.attrib.get("threshold", "20")),
        ),
    )

    factory.register_node_builder(
        "ChargeBattery",
        lambda node: ChargeBatteryNode(
            node.attrib.get("name", "ChargeBattery"),
            int(node.attrib.get("charge_rate", "10")),
        ),
    )

    factory.register_node_builder(
        "MoveToPosition",
        lambda node: MoveToPositionNode(
            node.attrib.get("name", "MoveToPosition"),
            node.attrib.get("position", "Unknown Position"),
            float(node.attrib.get("move_duration", "2")),
        ),
    )

    factory.register_node_builder(
        "DetectObstacle",
        lambda node: DetectObstacleNode(
            node.attrib.get("name", "DetectObstacle"),
            float(node.attrib.get("obstacle_probability", "0.3")),
        ),
    )

    factory.register_node_builder(
        "AvoidObstacle",
        lambda node: AvoidObstacleNode(
            node.attrib.get("name", "AvoidObstacle"),
            float(node.attrib.get("avoid_duration", "1.5")),
        ),
    )

    factory.register_node_builder(
        "PerformTask",
        lambda node: PerformTaskNode(
            node.attrib.get("name", "PerformTask"),
            node.attrib.get("task_name", "Unknown Task"),
            float(node.attrib.get("task_duration", "3")),
        ),
    )
    # 从文件读取 XML
    xml_path = os.path.join(os.path.dirname(__file__), "robot_behavior_tree.xml")
    return factory.build_tree_from_file(xml_path)


def main():
    """
    主函数：创建并运行机器人行为树
    """
    print("Creating robot behavior tree...")
    tree = create_robot_behavior_tree()

    print("\nStarting robot behavior tree execution...\n")
    # 运行一段时间或直到完成

    tree.tick_while_running(interval=0.01)

    print("\nBehavior tree execution complete!")


if __name__ == "__main__":
    main()
