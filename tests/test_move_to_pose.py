import itertools
import numpy as np
import conftest  # 将根路径加入 sys.path
from PathTracking.move_to_pose import move_to_pose as m


def test_random():
    m.show_animation = False
    m.main()


def test_stability():
    """
    本单元测试检查 move_to_pose.py 程序的稳定性
    """
    m.show_animation = False
    x_start = 5
    y_start = 5
    theta_start = 0
    x_goal = 1
    y_goal = 4
    theta_goal = 0
    _, _, v_traj, w_traj = m.move_to_pose(
        x_start, y_start, theta_start, x_goal, y_goal, theta_goal
    )

    def v_is_change(current, previous):
        return abs(current - previous) > m.MAX_LINEAR_SPEED

    def w_is_change(current, previous):
        return abs(current - previous) > m.MAX_ANGULAR_SPEED

    # 检查速度是否变化过大
    window_size = 10
    count_threshold = 4
    v_change = [v_is_change(v_traj[i], v_traj[i - 1]) for i in range(1, len(v_traj))]
    w_change = [w_is_change(w_traj[i], w_traj[i - 1]) for i in range(1, len(w_traj))]
    for i in range(len(v_change) - window_size + 1):
        v_window = v_change[i : i + window_size]
        w_window = w_change[i : i + window_size]

        v_unstable = sum(v_window) > count_threshold
        w_unstable = sum(w_window) > count_threshold

        assert not v_unstable, (
            f"v_unstable in window [{i}, {i + window_size}], unstable count: {sum(v_window)}"
        )
        assert not w_unstable, (
            f"w_unstable in window [{i}, {i + window_size}], unstable count: {sum(w_window)}"
        )


def test_reach_goal():
    """
    本单元测试检查 move_to_pose.py 程序是否到达目标
    """
    m.show_animation = False
    x_start = 5
    y_start = 5
    theta_start_list = [0, np.pi / 2, np.pi, 3 * np.pi / 2]
    x_goal_list = [0, 5, 10]
    y_goal_list = [0, 5, 10]
    theta_goal = 0
    for theta_start, x_goal, y_goal in itertools.product(
        theta_start_list, x_goal_list, y_goal_list
    ):
        x_traj, y_traj, _, _ = m.move_to_pose(
            x_start, y_start, theta_start, x_goal, y_goal, theta_goal
        )
        x_diff = x_goal - x_traj[-1]
        y_diff = y_goal - y_traj[-1]
        rho = np.hypot(x_diff, y_diff)
        assert rho < 0.001, (
            f"start:[{x_start}, {y_start}, {theta_start}], goal:[{x_goal}, {y_goal}, {theta_goal}], rho: {rho} is too large"
        )


def test_max_speed():
    """
    本单元测试检查 move_to_pose.py 程序的 MAX_LINEAR_SPEED 与
    MAX_ANGULAR_SPEED
    """
    m.show_animation = False
    m.MAX_LINEAR_SPEED = 11
    m.MAX_ANGULAR_SPEED = 5
    m.main()


if __name__ == "__main__":
    conftest.run_this_test(__file__)
