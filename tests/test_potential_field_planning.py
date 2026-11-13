import conftest  # 将根路径加入 sys.path
from PathPlanning.PotentialFieldPlanning import potential_field_planning as m


def test1():
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
