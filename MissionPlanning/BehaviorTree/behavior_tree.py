"""
行为树

作者: Wang Zheng (@Aglargil)

参考文献:

- [Behavior Tree](https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control))
"""

import time
import xml.etree.ElementTree as ET
from enum import Enum


class Status(Enum):
    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"


class NodeType(Enum):
    CONTROL_NODE = "ControlNode"
    ACTION_NODE = "ActionNode"
    DECORATOR_NODE = "DecoratorNode"


class Node:
    """
    行为树中所有节点的基类。
    """

    def __init__(self, name):
        self.name = name
        self.status = None

    def tick(self) -> Status:
        """
        对节点执行单次执行（tick）。

        Returns:
            Status: 节点的状态。
        """
        raise ValueError("Node is not implemented")

    def tick_and_set_status(self) -> Status:
        """
        对节点执行单次执行（tick）并设置状态。

        Returns:
            Status: 节点的状态。
        """
        self.status = self.tick()
        return self.status

    def reset(self):
        """
        重置该节点。
        """
        self.status = None

    def reset_children(self):
        """
        重置该节点的子节点。
        """
        pass


class ControlNode(Node):
    """
    行为树中所有控制节点的基类。

    控制节点按特定规则管理其子节点的执行流程。
    它们通常拥有多个子节点，并决定执行哪些子节点以及执行顺序。
    """

    def __init__(self, name):
        super().__init__(name)
        self.children = []
        self.type = NodeType.CONTROL_NODE

    def not_set_children_raise_error(self):
        if len(self.children) == 0:
            raise ValueError("Children are not set")

    def reset_children(self):
        for child in self.children:
            child.reset()


class SequenceNode(ControlNode):
    """
    顺序节点：按顺序执行子节点，直到某一子节点失败或全部成功。

    Returns:
        - 若任一子节点返回 FAILURE，则返回 FAILURE
        - 当全部子节点都成功时返回 SUCCESS
        - 当某个子节点仍在运行，或即将转到下一个子节点时，返回 RUNNING

    Example:
        .. code-block:: xml

            <Sequence>
                <Action1 />
                <Action2 />
            </Sequence>
    """

    def __init__(self, name):
        super().__init__(name)
        self.current_child_index = 0

    def tick(self) -> Status:
        self.not_set_children_raise_error()

        if self.current_child_index >= len(self.children):
            self.reset_children()
            return Status.SUCCESS
        status = self.children[self.current_child_index].tick_and_set_status()
        if status == Status.FAILURE:
            self.reset_children()
            return Status.FAILURE
        elif status == Status.SUCCESS:
            self.current_child_index += 1
            return Status.RUNNING
        elif status == Status.RUNNING:
            return Status.RUNNING
        else:
            raise ValueError("Unknown status")


class SelectorNode(ControlNode):
    """
    选择节点：按顺序执行子节点，直到某一子节点成功或全部失败。

    Returns:
        - 若任一子节点返回 SUCCESS，则返回 SUCCESS
        - 当全部子节点都失败时返回 FAILURE
        - 当某个子节点仍在运行，或即将转到下一个子节点时，返回 RUNNING

    Examples:
        .. code-block:: xml

            <Selector>
                <Action1 />
                <Action2 />
            </Selector>
    """

    def __init__(self, name):
        super().__init__(name)
        self.current_child_index = 0

    def tick(self) -> Status:
        self.not_set_children_raise_error()

        if self.current_child_index >= len(self.children):
            self.reset_children()
            return Status.FAILURE
        status = self.children[self.current_child_index].tick_and_set_status()
        if status == Status.SUCCESS:
            self.reset_children()
            return Status.SUCCESS
        elif status == Status.FAILURE:
            self.current_child_index += 1
            return Status.RUNNING
        elif status == Status.RUNNING:
            return Status.RUNNING
        else:
            raise ValueError("Unknown status")


class WhileDoElseNode(ControlNode):
    """
    条件执行节点，包含三部分：条件、执行，以及可选的否则分支。

    Returns:
        首先执行条件节点（child[0]）
        若条件成功，则执行 do 节点（child[1]）并返回 RUNNING
        若条件失败，则在存在 else 节点（child[2]）时执行它，并返回 else 节点的结果
        若条件失败且没有 else 节点，则返回 SUCCESS

    Example:
        .. code-block:: xml

            <WhileDoElse>
                <Condition />
                <Do />
                <Else />
            </WhileDoElse>
    """

    def __init__(self, name):
        super().__init__(name)

    def tick(self) -> Status:
        if len(self.children) != 3 and len(self.children) != 2:
            raise ValueError("WhileDoElseNode must have exactly 3 or 2 children")

        condition_node = self.children[0]
        do_node = self.children[1]
        else_node = self.children[2] if len(self.children) == 3 else None

        condition_status = condition_node.tick_and_set_status()
        if condition_status == Status.SUCCESS:
            do_node.tick_and_set_status()
            return Status.RUNNING
        elif condition_status == Status.FAILURE:
            if else_node is not None:
                else_status = else_node.tick_and_set_status()
                if else_status == Status.SUCCESS:
                    self.reset_children()
                    return Status.SUCCESS
                elif else_status == Status.FAILURE:
                    self.reset_children()
                    return Status.FAILURE
                elif else_status == Status.RUNNING:
                    return Status.RUNNING
                else:
                    raise ValueError("Unknown status")
            else:
                self.reset_children()
                return Status.SUCCESS
        else:
            raise ValueError("Unknown status")


class ActionNode(Node):
    """
    行为树中所有动作节点的基类。

    动作节点负责执行具体任务或动作。
    它们没有子节点，通常用于执行逻辑或操作。
    """

    def __init__(self, name):
        super().__init__(name)
        self.type = NodeType.ACTION_NODE


class SleepNode(ActionNode):
    """
    休眠指定时长的休眠节点。

    Returns:
        指定时长过去后返回 SUCCESS
        若时长尚未过去则返回 RUNNING

    Example:
        .. code-block:: xml

            <Sleep sec="1.5" />
    """

    def __init__(self, name, duration):
        super().__init__(name)
        self.duration = duration
        self.start_time = None

    def tick(self) -> Status:
        if self.start_time is None:
            self.start_time = time.time()
        if time.time() - self.start_time > self.duration:
            return Status.SUCCESS
        return Status.RUNNING


class EchoNode(ActionNode):
    """
    向控制台打印一条消息的回显节点。

    Returns:
        消息打印后返回 SUCCESS

    Example:
        .. code-block:: xml

            <Echo message="Hello, World!" />
    """

    def __init__(self, name, message):
        super().__init__(name)
        self.message = message

    def tick(self) -> Status:
        print(self.name, self.message)
        return Status.SUCCESS


class DecoratorNode(Node):
    """
    行为树中所有装饰节点的基类。

    装饰节点修改其子节点的行为。
    它们必须只有一个子节点，并且可以改变子节点的状态。
    """

    def __init__(self, name):
        super().__init__(name)
        self.type = NodeType.DECORATOR_NODE
        self.child = None

    def not_set_child_raise_error(self):
        if self.child is None:
            raise ValueError("Child is not set")

    def reset_children(self):
        self.child.reset()


class InverterNode(DecoratorNode):
    """
    反转子节点状态的反相节点。

    Returns:
        - 若子节点返回 FAILURE，则返回 SUCCESS
        - 若子节点返回 SUCCESS，则返回 FAILURE
        - 若子节点返回 RUNNING，则返回 RUNNING

    Examples:
        .. code-block:: xml

            <Inverter>
                <Action />
            </Inverter>
    """

    def __init__(self, name):
        super().__init__(name)

    def tick(self) -> Status:
        self.not_set_child_raise_error()
        status = self.child.tick_and_set_status()
        return Status.SUCCESS if status == Status.FAILURE else Status.FAILURE


class TimeoutNode(DecoratorNode):
    """
    若子节点执行时间过长则失败的超时节点

    Returns:
        - FAILURE：若已超过超时时长
        - 子节点的状态：否则透传子节点的状态

    Example:
        .. code-block:: xml

            <Timeout sec="1.5">
                <Action />
            </Timeout>
    """

    def __init__(self, name, timeout):
        super().__init__(name)
        self.timeout = timeout
        self.start_time = None

    def tick(self) -> Status:
        self.not_set_child_raise_error()
        if self.start_time is None:
            self.start_time = time.time()
        if time.time() - self.start_time > self.timeout:
            return Status.FAILURE
        print(f"{self.name} is running")
        return self.child.tick_and_set_status()


class DelayNode(DecoratorNode):
    """
    将子节点的执行延迟指定时长的延迟节点。

    Returns:
        - 若时长尚未过去，则返回 RUNNING
        - 时长过去后返回子节点的状态

    Example:
        .. code-block:: xml

            <Delay sec="1.5">
                <Action />
            </Delay>
    """

    def __init__(self, name, delay):
        super().__init__(name)
        self.delay = delay
        self.start_time = None

    def tick(self) -> Status:
        self.not_set_child_raise_error()
        if self.start_time is None:
            self.start_time = time.time()
        if time.time() - self.start_time > self.delay:
            return self.child.tick_and_set_status()
        return Status.RUNNING


class ForceSuccessNode(DecoratorNode):
    """
    始终返回 SUCCESS 的 ForceSuccess 节点。

    Returns:
        - 若子节点返回 RUNNING，则返回 RUNNING
        - 若子节点返回 SUCCESS 或 FAILURE，则返回 SUCCESS
    """

    def __init__(self, name):
        super().__init__(name)

    def tick(self) -> Status:
        self.not_set_child_raise_error()
        status = self.child.tick_and_set_status()
        if status == Status.FAILURE:
            return Status.SUCCESS
        return status


class ForceFailureNode(DecoratorNode):
    """
    始终返回 FAILURE 的 ForceFailure 节点。

    Returns:
        - 若子节点返回 RUNNING，则返回 RUNNING
        - 若子节点返回 SUCCESS 或 FAILURE，则返回 FAILURE
    """

    def __init__(self, name):
        super().__init__(name)

    def tick(self) -> Status:
        self.not_set_child_raise_error()
        status = self.child.tick_and_set_status()
        if status == Status.SUCCESS:
            return Status.FAILURE
        return status


class BehaviorTree:
    """
    管理行为树执行的行为树类。
    """

    def __init__(self, root):
        self.root = root

    def tick(self):
        """
        对行为树执行一次单次执行（tick）。
        """
        self.root.tick_and_set_status()

    def reset(self):
        """
        重置行为树。
        """
        self.root.reset()

    def tick_while_running(self, interval=None, enable_print=True):
        """
        在行为树处于运行状态时持续执行单次执行（tick）。

        Args:
            interval (float, optional): 两次 tick 之间的间隔。默认值为 None。
            enable_print (bool, optional): 是否打印行为树。默认值为 True。
        """
        while self.root.tick_and_set_status() == Status.RUNNING:
            if enable_print:
                self.print_tree()
            if interval is not None:
                time.sleep(interval)
        if enable_print:
            self.print_tree()

    def to_text(self, root, indent=0):
        """
        递归地将行为树转换为文本表示。
        """
        current_text = ""
        if root.status == Status.RUNNING:
            # 黄色
            current_text = "\033[93m" + root.name + "\033[0m"
        elif root.status == Status.SUCCESS:
            # 绿色
            current_text = "\033[92m" + root.name + "\033[0m"
        elif root.status == Status.FAILURE:
            # 红色
            current_text = "\033[91m" + root.name + "\033[0m"
        else:
            current_text = root.name
        if root.type == NodeType.CONTROL_NODE:
            current_text = "  " * indent + "[" + current_text + "]\n"
            for child in root.children:
                current_text += self.to_text(child, indent + 2)
        elif root.type == NodeType.DECORATOR_NODE:
            current_text = "  " * indent + "(" + current_text + ")\n"
            current_text += self.to_text(root.child, indent + 2)
        elif root.type == NodeType.ACTION_NODE:
            current_text = "  " * indent + "<" + current_text + ">\n"
        return current_text

    def print_tree(self):
        """
        打印行为树。

        节点打印格式:
            动作: <Action>
            装饰: (Decorator)
            控制: [Control]

        节点状态颜色:
            黄色: RUNNING
            绿色: SUCCESS
            红色: FAILURE
        """
        text = self.to_text(self.root)
        text = text.strip()
        print("\033[94m" + "Behavior Tree" + "\033[0m")
        print(text)
        print("\033[94m" + "Behavior Tree" + "\033[0m")


class BehaviorTreeFactory:
    """
    用于从 XML 字符串创建行为树的工厂类。
    """

    def __init__(self):
        self.node_builders = {}
        # 控制节点
        self.register_node_builder(
            "Sequence",
            lambda node: SequenceNode(node.attrib.get("name", SequenceNode.__name__)),
        )
        self.register_node_builder(
            "Selector",
            lambda node: SelectorNode(node.attrib.get("name", SelectorNode.__name__)),
        )
        self.register_node_builder(
            "WhileDoElse",
            lambda node: WhileDoElseNode(
                node.attrib.get("name", WhileDoElseNode.__name__)
            ),
        )
        # 装饰节点
        self.register_node_builder(
            "Inverter",
            lambda node: InverterNode(node.attrib.get("name", InverterNode.__name__)),
        )
        self.register_node_builder(
            "Timeout",
            lambda node: TimeoutNode(
                node.attrib.get("name", SelectorNode.__name__),
                float(node.attrib["sec"]),
            ),
        )
        self.register_node_builder(
            "Delay",
            lambda node: DelayNode(
                node.attrib.get("name", DelayNode.__name__),
                float(node.attrib["sec"]),
            ),
        )
        self.register_node_builder(
            "ForceSuccess",
            lambda node: ForceSuccessNode(
                node.attrib.get("name", ForceSuccessNode.__name__)
            ),
        )
        self.register_node_builder(
            "ForceFailure",
            lambda node: ForceFailureNode(
                node.attrib.get("name", ForceFailureNode.__name__)
            ),
        )
        # 动作节点
        self.register_node_builder(
            "Sleep",
            lambda node: SleepNode(
                node.attrib.get("name", SleepNode.__name__),
                float(node.attrib["sec"]),
            ),
        )
        self.register_node_builder(
            "Echo",
            lambda node: EchoNode(
                node.attrib.get("name", EchoNode.__name__),
                node.attrib["message"],
            ),
        )

    def register_node_builder(self, node_name, builder):
        """
        为节点注册构建器

        Args:
            node_name (str): 节点名称。
            builder (function): 构建函数。

        Example:
            .. code-block:: python

                factory = BehaviorTreeFactory()
                factory.register_node_builder(
                    "MyNode",
                    lambda node: MyNode(
                        node.attrib.get("name", MyNode.__name__),
                        node.attrib["my_param"],
                    ),
                )
        """
        self.node_builders[node_name] = builder

    def build_node(self, node):
        """
        由 XML 元素构建节点。

        Args:
            node (Element): 用于构建节点的 XML 元素。

        Returns:
            BehaviorTree Node: 构建得到的节点
        """
        if node.tag in self.node_builders:
            root = self.node_builders[node.tag](node)
            if root.type == NodeType.CONTROL_NODE:
                if len(node) <= 0:
                    raise ValueError(f"{root.name} Control node must have children")
                for child in node:
                    root.children.append(self.build_node(child))
            elif root.type == NodeType.DECORATOR_NODE:
                if len(node) != 1:
                    raise ValueError(
                        f"{root.name} Decorator node must have exactly one child"
                    )
                root.child = self.build_node(node[0])
            elif root.type == NodeType.ACTION_NODE:
                if len(node) != 0:
                    raise ValueError(f"{root.name} Action node must have no children")
            return root
        else:
            raise ValueError(f"Unknown node type: {node.tag}")

    def build_tree(self, xml_string):
        """
        由 XML 字符串构建行为树。

        Args:
            xml_string (str): 包含行为树的 XML 字符串。

        Returns:
            BehaviorTree: 行为树。
        """
        xml_tree = ET.fromstring(xml_string)
        root = self.build_node(xml_tree)
        return BehaviorTree(root)

    def build_tree_from_file(self, file_path):
        """
        从文件构建行为树。

        Args:
            file_path (str): 包含行为树的文件路径。

        Returns:
            BehaviorTree: 行为树。
        """
        with open(file_path) as file:
            xml_string = file.read()
        return self.build_tree(xml_string)


xml_string = """
    <Sequence name="Sequence">
        <Echo name="Echo0" message="Hello, World0!" />
        <Delay name="Delay" sec="1.5">
            <Echo name="Echo1" message="Hello, World1!" />
        </Delay>
        <Echo name="Echo2" message="Hello, World2!" />
    </Sequence>
   """


def main():
    factory = BehaviorTreeFactory()
    tree = factory.build_tree(xml_string)
    tree.tick_while_running()


if __name__ == "__main__":
    main()
