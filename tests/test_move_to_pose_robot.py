import conftest  # 将根路径加入 sys.path
from PathTracking.move_to_pose import move_to_pose as m


def test_1():
    """
    本单元测试检查 move_to_pose_robot.py 程序
    """
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
