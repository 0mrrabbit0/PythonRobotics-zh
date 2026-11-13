"""

使用纯追踪转向与 PID 速率控制的路径跟踪仿真。

作者：Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)

"""
import numpy as np
import math
import matplotlib.pyplot as plt

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
from utils.angle import angle_mod

# 参数
k = 0.1  # 前视增益
Lfc = 2.0  # [m] 前视距离
Kp = 1.0  # 速率比例增益
dt = 0.1  # [s] 时间步长
WB = 2.9  # [m] 车辆轴距


# 车辆参数
LENGTH = WB + 1.0  # 车辆长度
WIDTH = 2.0  # 车辆宽度
WHEEL_LEN = 0.6  # 车轮长度
WHEEL_WIDTH = 0.2  # 车轮宽度
MAX_STEER = math.pi / 4  # 最大转向角 [rad]


show_animation = True
pause_simulation = False  # 暂停仿真标志
is_reverse_mode = False   # 倒车模式标志

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, is_reverse=False):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direction = -1 if is_reverse else 1  # 由倒车标志决定的方向
        self.rear_x = self.x - self.direction * ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - self.direction * ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.direction * self.v / WB * math.tan(delta) * dt
        self.yaw = angle_mod(self.yaw)
        self.v += a * dt
        self.rear_x = self.x - self.direction * ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - self.direction * ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.direction = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.direction.append(state.direction)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)

    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # 为加快最近点搜索，仅在首次执行。
        if self.old_nearest_point_index is None:
            # 搜索最近点索引
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # 更新前视距离

        # 搜索前视目标点索引
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # 不超过目标点
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    # 倒车时反转转向角
    delta = state.direction * math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    # 将转向角限制在最大值以内
    delta = np.clip(delta, -MAX_STEER, MAX_STEER)

    return delta, ind


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    绘制箭头
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

def plot_vehicle(x, y, yaw, steer=0.0, color='blue', is_reverse=False):
    """
    绘制带四个车轮的车辆模型
    Args:
        x, y: 车辆中心位置
        yaw: 车辆航向角
        steer: 转向角
        color: 车辆颜色
        is_reverse: 倒车模式标志
    """
    # 在倒车模式下调整航向角
    if is_reverse:
        yaw = angle_mod(yaw + math.pi)  # 将航向旋转 180 度
        steer = -steer  # 反转转向方向

    def plot_wheel(x, y, yaw, steer=0.0, color=color):
        """绘制单个车轮"""
        wheel = np.array([
            [-WHEEL_LEN/2, WHEEL_WIDTH/2],
            [WHEEL_LEN/2, WHEEL_WIDTH/2],
            [WHEEL_LEN/2, -WHEEL_WIDTH/2],
            [-WHEEL_LEN/2, -WHEEL_WIDTH/2],
            [-WHEEL_LEN/2, WHEEL_WIDTH/2]
        ])

        # 若存在转向则旋转车轮
        if steer != 0:
            c, s = np.cos(steer), np.sin(steer)
            rot_steer = np.array([[c, -s], [s, c]])
            wheel = wheel @ rot_steer.T

        # 施加车辆航向旋转
        c, s = np.cos(yaw), np.sin(yaw)
        rot_yaw = np.array([[c, -s], [s, c]])
        wheel = wheel @ rot_yaw.T

        # 平移到指定位置
        wheel[:, 0] += x
        wheel[:, 1] += y

        # 用指定颜色绘制车轮
        plt.plot(wheel[:, 0], wheel[:, 1], color=color)

    # 计算车身角点
    corners = np.array([
        [-LENGTH/2, WIDTH/2],
        [LENGTH/2, WIDTH/2],
        [LENGTH/2, -WIDTH/2],
        [-LENGTH/2, -WIDTH/2],
        [-LENGTH/2, WIDTH/2]
    ])

    # 旋转矩阵
    c, s = np.cos(yaw), np.sin(yaw)
    Rot = np.array([[c, -s], [s, c]])

    # 旋转并平移车身
    rotated = corners @ Rot.T
    rotated[:, 0] += x
    rotated[:, 1] += y

    # 绘制车身
    plt.plot(rotated[:, 0], rotated[:, 1], color=color)

    # 绘制车轮（前轮使用更深的颜色）
    front_color = 'darkblue'
    rear_color = color
    
    # 绘制四个车轮
    # 左前轮
    plot_wheel(x + LENGTH/4 * c - WIDTH/2 * s,
              y + LENGTH/4 * s + WIDTH/2 * c,
              yaw, steer, front_color)
    # 右前轮 
    plot_wheel(x + LENGTH/4 * c + WIDTH/2 * s,
              y + LENGTH/4 * s - WIDTH/2 * c,
              yaw, steer, front_color)
    # 左后轮
    plot_wheel(x - LENGTH/4 * c - WIDTH/2 * s,
              y - LENGTH/4 * s + WIDTH/2 * c,
              yaw, color=rear_color)
    # 右后轮
    plot_wheel(x - LENGTH/4 * c + WIDTH/2 * s,
              y - LENGTH/4 * s - WIDTH/2 * c,
              yaw, color=rear_color)

    # 添加方向箭头
    arrow_length = LENGTH/3
    plt.arrow(x, y,
             -arrow_length * math.cos(yaw) if is_reverse else arrow_length * math.cos(yaw),
             -arrow_length * math.sin(yaw) if is_reverse else arrow_length * math.sin(yaw),
             head_width=WIDTH/4, head_length=WIDTH/4,
             fc='r', ec='r', alpha=0.5)

# 键盘事件处理函数
def on_key(event):
    global pause_simulation
    if event.key == ' ':  # 空格键
        pause_simulation = not pause_simulation
    elif event.key == 'escape':
        exit(0)

def main():
    #  目标路径
    cx = -1 * np.arange(0, 50, 0.5) if is_reverse_mode else np.arange(0, 50, 0.5)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    target_speed = 10.0 / 3.6  # [m/s]

    T = 100.0  # 最大仿真时间

    # 初始状态
    state = State(x=-0.0, y=-3.0, yaw=math.pi if is_reverse_mode else 0.0, v=0.0, is_reverse=is_reverse_mode)

    lastIndex = len(cx) - 1
    time = 0.0
    states = States()
    states.append(time, state)
    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)

    while T >= time and lastIndex > target_ind:

        # 计算控制输入
        ai = proportional_control(target_speed, state.v)
        di, target_ind = pure_pursuit_steer_control(
            state, target_course, target_ind)

        state.update(ai, di)  # 控制车辆

        time += dt
        states.append(time, state)
        if show_animation:  # pragma: no cover
            plt.cla()
            # 按 esc 键停止仿真。
            plt.gcf().canvas.mpl_connect('key_release_event', on_key)
            # 传入 is_reverse 参数
            plot_vehicle(state.x, state.y, state.yaw, di, is_reverse=is_reverse_mode)
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.legend()  # 添加图例显示

            # 添加暂停状态显示
            if pause_simulation:
                plt.text(0.02, 0.95, 'PAUSED', transform=plt.gca().transAxes,
                        bbox=dict(facecolor='red', alpha=0.5))

            plt.pause(0.001)

            # 处理暂停状态
            while pause_simulation:
                plt.pause(0.1)  # 降低 CPU 占用
                if not plt.get_fignums():  # 检查窗口是否已关闭
                    exit(0)

    # 测试
    assert lastIndex >= target_ind, "Cannot goal"

    if show_animation:  # pragma: no cover
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
