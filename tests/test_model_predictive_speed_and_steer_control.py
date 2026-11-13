import conftest  # 将根路径加入 sys.path

from PathTracking.model_predictive_speed_and_steer_control \
    import model_predictive_speed_and_steer_control as m


def test_1():
    m.show_animation = False
    m.main()


def test_2():
    m.show_animation = False
    m.main2()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
