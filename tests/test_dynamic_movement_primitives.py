import conftest
import numpy as np
from PathPlanning.DynamicMovementPrimitives import \
            dynamic_movement_primitives


def test_1():
    # 测试能否从用户传入的数据学习轨迹
    T = 5
    t = np.arange(0, T, 0.01)
    sin_t = np.sin(t)
    train_data = np.array([t, sin_t]).T

    DMP_controller = dynamic_movement_primitives.DMP(train_data, T)
    DMP_controller.recreate_trajectory(train_data[0], train_data[-1], 4)


def test_2():
    # 测试轨迹长度是否等于期望的时间步数
    T = 5
    t = np.arange(0, T, 0.01)
    sin_t = np.sin(t)
    train_data = np.array([t, sin_t]).T

    DMP_controller = dynamic_movement_primitives.DMP(train_data, T)
    t, path = DMP_controller.recreate_trajectory(train_data[0],
                                                 train_data[-1], 4)

    assert(path.shape[0] == DMP_controller.timesteps)


def test_3():
    # 检查学习到的轨迹是否接近初始轨迹
    T = 3*np.pi/2
    A_noise = 0.02
    t = np.arange(0, T, 0.01)
    noisy_sin_t = np.sin(t) + A_noise*np.random.rand(len(t))
    train_data = np.array([t, noisy_sin_t]).T

    DMP_controller = dynamic_movement_primitives.DMP(train_data, T)
    t, pos = DMP_controller.recreate_trajectory(train_data[0],
                                                train_data[-1], T)

    diff = abs(pos[:, 1] - noisy_sin_t)
    assert(max(diff) < 5*A_noise)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
