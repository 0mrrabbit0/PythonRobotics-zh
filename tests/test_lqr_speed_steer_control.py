import conftest  # 将根路径加入 sys.path
from PathTracking.lqr_speed_steer_control import lqr_speed_steer_control as m


def test_1():
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
