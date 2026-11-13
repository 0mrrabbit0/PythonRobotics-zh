import conftest  # 将根路径加入 sys.path
from PathTracking.stanley_control import stanley_control as m


def test1():
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
