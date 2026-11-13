import conftest  # noqa
import numpy as np
from Mapping.DistanceMap import distance_map as m


def test_compute_sdf():
    """测试有符号距离场（SDF）的计算"""
    # 创建用于测试的简单障碍物地图
    obstacles = np.array([[0, 0, 0], [0, 1, 0], [0, 0, 0]])

    sdf = m.compute_sdf(obstacles)

    # 验证 SDF 的基本性质
    assert sdf.shape == obstacles.shape, "SDF should have the same shape as input map"
    assert np.all(np.isfinite(sdf)), "SDF should not contain infinite values"

    # 验证障碍物位置处的 SDF 值为负
    assert sdf[1, 1] < 0, "SDF value should be negative at obstacle position"

    # 验证自由空间中的 SDF 值为正
    assert sdf[0, 0] > 0, "SDF value should be positive in free space"


def test_compute_udf():
    """测试无符号距离场（UDF）的计算"""
    # 创建用于测试的障碍物地图
    obstacles = np.array([[0, 0, 0], [0, 1, 0], [0, 0, 0]])

    udf = m.compute_udf(obstacles)

    # 验证 UDF 的基本性质
    assert udf.shape == obstacles.shape, "UDF should have the same shape as input map"
    assert np.all(np.isfinite(udf)), "UDF should not contain infinite values"
    assert np.all(udf >= 0), "All UDF values should be non-negative"

    # 验证障碍物位置处的 UDF 值为 0
    assert np.abs(udf[1, 1]) < 1e-10, "UDF value should be 0 at obstacle position"

    # 验证相邻栅格的 UDF 值为 1
    assert np.abs(udf[0, 1] - 1.0) < 1e-10, (
        "UDF value should be 1 for cells adjacent to obstacle"
    )
    assert np.abs(udf[1, 0] - 1.0) < 1e-10, (
        "UDF value should be 1 for cells adjacent to obstacle"
    )
    assert np.abs(udf[1, 2] - 1.0) < 1e-10, (
        "UDF value should be 1 for cells adjacent to obstacle"
    )
    assert np.abs(udf[2, 1] - 1.0) < 1e-10, (
        "UDF value should be 1 for cells adjacent to obstacle"
    )


def test_dt():
    """测试一维距离变换的计算"""
    # 创建测试数据
    d = np.array([m.INF, 0, m.INF])
    m.dt(d)

    # 验证距离变换结果
    assert np.all(np.isfinite(d)), (
        "Distance transform result should not contain infinite values"
    )
    assert d[1] == 0, "Distance at obstacle position should be 0"
    assert d[0] == 1, "Distance at adjacent position should be 1"
    assert d[2] == 1, "Distance at adjacent position should be 1"


def test_compute_sdf_empty():
    """测试空地图上的 SDF 计算"""
    # 用空地图（无障碍物）测试
    empty_map = np.zeros((5, 5))
    sdf = m.compute_sdf(empty_map)

    assert np.all(sdf > 0), "All SDF values should be positive for empty map"
    assert sdf.shape == empty_map.shape, "Output shape should match input shape"


def test_compute_sdf_full():
    """测试完全占据地图上的 SDF 计算"""
    # 用完全占据的地图测试
    full_map = np.ones((5, 5))
    sdf = m.compute_sdf(full_map)

    assert np.all(sdf < 0), "All SDF values should be negative for fully occupied map"
    assert sdf.shape == full_map.shape, "Output shape should match input shape"


def test_compute_udf_invalid_input():
    """测试无效输入值上的 UDF 计算"""
    # 用无效值（不是 0 或 1）测试
    invalid_map = np.array([[0, 2, 0], [0, -1, 0], [0, 0.5, 0]])

    try:
        m.compute_udf(invalid_map)
        assert False, "Should raise ValueError for invalid input values"
    except ValueError:
        pass


def test_compute_udf_empty():
    """测试空地图上的 UDF 计算"""
    # 用空地图测试
    empty_map = np.zeros((5, 5))
    udf = m.compute_udf(empty_map)

    assert np.all(udf > 0), "All UDF values should be positive for empty map"
    assert np.all(np.isfinite(udf)), "UDF should not contain infinite values"


def test_main():
    """测试 main 函数的执行"""
    m.ENABLE_PLOT = False
    m.main()


if __name__ == "__main__":
    conftest.run_this_test(__file__)
