import conftest  # 将根路径加入 sys.path
from PathPlanning.ParticleSwarmOptimization import particle_swarm_optimization as m


def test_1():
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)