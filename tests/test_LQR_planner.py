import conftest  # 将根路径加入 sys.path
from PathPlanning.LQRPlanner import lqr_planner as m


def test_1():
    m.SHOW_ANIMATION = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
