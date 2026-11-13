"""
三维 n 连杆机械臂的正运动学
作者: Takayuki Murooka (takayuki5168)
"""
import math
from NLinkArm3d import NLinkArm
import random


def random_val(min_val, max_val):
    return min_val + random.random() * (max_val - min_val)


def main():
    print("Start solving Forward Kinematics 10 times")
    # 用 PR2 的 Denavit-Hartenberg 参数初始化 NLinkArm
    n_link_arm = NLinkArm([[0., -math.pi / 2, .1, 0.],
                           [math.pi / 2, math.pi / 2, 0., 0.],
                           [0., -math.pi / 2, 0., .4],
                           [0., math.pi / 2, 0., 0.],
                           [0., -math.pi / 2, 0., .321],
                           [0., math.pi / 2, 0., 0.],
                           [0., 0., 0., 0.]])
    # 执行正运动学 10 次
    for _ in range(10):
        n_link_arm.set_joint_angles(
            [random_val(-1, 1) for _ in range(len(n_link_arm.link_list))])

        n_link_arm.forward_kinematics(plot=True)


if __name__ == "__main__":
    main()
