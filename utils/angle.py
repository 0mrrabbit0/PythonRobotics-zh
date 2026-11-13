import numpy as np
from scipy.spatial.transform import Rotation as Rot


def rot_mat_2d(angle):
    """
    由角度创建二维旋转矩阵

    Parameters
    ----------
    angle :

    Returns
    -------
    二维旋转矩阵

    Examples
    --------
    >>> angle_mod(-4.0)


    """
    return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]


def angle_mod(x, zero_2_2pi=False, degree=False):
    """
    角度取模运算
    默认的角度取模区间为 [-pi, pi)

    Parameters
    ----------
    x : float 或 array_like
        一个角度或角度数组。计算时会将该数组展平。
        若传入单个角度，则返回浮点角度。
    zero_2_2pi : bool，可选
        将角度取模区间改为 [0, 2pi)
        默认值为 False。
    degree : bool，可选
        若为 True，则给定角度按度数解释。
        默认值为 False。

    Returns
    -------
    ret : float 或 ndarray
        一个角度或取模后的角度数组。

    Examples
    --------
    >>> angle_mod(-4.0)
    2.28318531

    >>> angle_mod([-4.0])
    np.array(2.28318531)

    >>> angle_mod([-150.0, 190.0, 350], degree=True)
    array([-150., -170.,  -10.])

    >>> angle_mod(-60.0, zero_2_2pi=True, degree=True)
    array([300.])

    """
    if isinstance(x, float):
        is_float = True
    else:
        is_float = False

    x = np.asarray(x).flatten()
    if degree:
        x = np.deg2rad(x)

    if zero_2_2pi:
        mod_angle = x % (2 * np.pi)
    else:
        mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

    if degree:
        mod_angle = np.rad2deg(mod_angle)

    if is_float:
        return mod_angle.item()
    else:
        return mod_angle
