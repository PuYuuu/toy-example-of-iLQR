import math
import sys
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D
import numpy as np
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent))

from utils import cubic_spline


# LQR parameter
lqr_Q = np.eye(5)
lqr_R = np.eye(2)
dt = 0.1    # time tick[s]
max_steer = np.deg2rad(40.0)  # maximum steering angle[rad]
max_acc = 5

# vehicle parameter [m]
LENGTH = 4.5
WIDTH = 2.2
BACKTOWHEEL = 1.0
WHEEL_LEN = 0.35
WHEEL_WIDTH = 0.2
TREAD = 0.7
WB = 2.5

show_animation = True
steering_wheel_path = str(pathlib.Path(__file__).parent.parent / "images" / "steering_wheel.png")
steering_wheel = plt.imread(steering_wheel_path, format = "png")

def plot_arrow(x: float, y: float, theta: float, L: float, c: str) -> None:
    angle = np.deg2rad(30)
    d = 0.3 * L
    w = 2

    x_start = x
    y_start = y
    x_end = x + L * np.cos(theta)
    y_end = y + L * np.sin(theta)

    theta_hat_L = theta + np.pi - angle
    theta_hat_R = theta + np.pi + angle

    x_hat_start = x_end
    x_hat_end_L = x_hat_start + d * np.cos(theta_hat_L)
    x_hat_end_R = x_hat_start + d * np.cos(theta_hat_R)

    y_hat_start = y_end
    y_hat_end_L = y_hat_start + d * np.sin(theta_hat_L)
    y_hat_end_R = y_hat_start + d * np.sin(theta_hat_R)

    plt.plot([x_start, x_end], [y_start, y_end], color=c, linewidth=w)
    plt.plot([x_hat_start, x_hat_end_L], [y_hat_start, y_hat_end_L], color=c, linewidth=w)
    plt.plot([x_hat_start, x_hat_end_R], [y_hat_start, y_hat_end_R], color=c, linewidth=w)


def plot_car(x: float, y: float, yaw: float, steer: float = 0.0, color = "k") -> None:
    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)
    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),  np.array(outline[1, :]).flatten(), color)
    plt.plot(np.array(fr_wheel[0, :]).flatten(), np.array(fr_wheel[1, :]).flatten(), color)
    plt.plot(np.array(rr_wheel[0, :]).flatten(), np.array(rr_wheel[1, :]).flatten(), color)
    plt.plot(np.array(fl_wheel[0, :]).flatten(), np.array(fl_wheel[1, :]).flatten(), color)
    plt.plot(np.array(rl_wheel[0, :]).flatten(), np.array(rl_wheel[1, :]).flatten(), color)
    plot_arrow(x, y, yaw, 0.8 * WB, color)


def plot_console(steer: float, acc: float) -> None:
    if acc >= 0:
        brake = 0
    else:
        brake = -acc
        acc = 0

    transform_data = Affine2D().rotate_deg_around(-5, 15, steer / np.pi * 180)
    transform_data += plt.gca().transData

    image_extent = [-8, -2, 12, 18]
    plt.imshow(steering_wheel, transform = transform_data,
               extent = image_extent, zorder = 10.0, clip_on = True)

    plt.fill([-1, 0, 0, -1], [12, 12, 12 + 6 * (brake / max_acc), 12 + 6 * (brake / max_acc)], 'c')
    plt.fill([1, 2, 2, 1], [12, 12, 12 + 6 * (acc / max_acc), 12 + 6 * (acc / max_acc)], 'o')


class VehicleState:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0) -> None:
        self.x: float = x
        self.y: float = y
        self.yaw: float = yaw
        self.v: float = v


def update(state: VehicleState, a: float, delta: float) -> VehicleState:
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / WB * math.tan(delta) * dt
    state.v = state.v + a * dt

    return state


def pi_2_pi(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def solve_dare(A: np.matrix, B: np.matrix,
               Q: np.matrix, R: np.matrix, max_iter: int = 150) -> np.matrix:
    # solve a discrete time_Algebraic Riccati equation (DARE)
    x = Q
    x_next = Q
    eps = 0.01

    for i in range(max_iter):
        x_next = A.T @ x @ A - A.T @ x @ B @ np.linalg.inv(R + B.T @ x @ B) @ B.T @ x @ A + Q
        if (abs(x_next - x)).max() < eps:
            break
        x = x_next

    return x_next


def lqr_speed_steering_control(state: VehicleState, cx: list, cy: list, cyaw: list, ck: list,
                               pe: float, pth_e: float, sp: list, Q: np.matrix, R: np.matrix) -> tuple:
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    tv = sp[ind]

    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    # A = [1.0, dt, 0.0, 0.0, 0.0
    #      0.0, 0.0, v, 0.0, 0.0]
    #      0.0, 0.0, 1.0, dt, 0.0]
    #      0.0, 0.0, 0.0, 0.0, 0.0]
    #      0.0, 0.0, 0.0, 0.0, 1.0]
    A = np.zeros((5, 5))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt
    A[4, 4] = 1.0

    # B = [0.0, 0.0
    #      0.0, 0.0
    #      0.0, 0.0
    #      v/L, 0.0
    #      0.0, dt]
    B = np.zeros((5, 2))
    B[3, 0] = v / WB
    B[4, 1] = dt

    Pstar = solve_dare(A, B, Q, R)
    Kstar = np.linalg.inv(B.T @ Pstar @ B + R) @ (B.T @ Pstar @ A)

    # state vector
    # x = [e, dot_e, th_e, dot_th_e, delta_v]
    # e: lateral distance to the path
    # dot_e: derivative of e
    # th_e: angle difference to the path
    # dot_th_e: derivative of th_e
    # delta_v: difference between current speed and target speed
    x = np.zeros((5, 1))
    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt
    x[4, 0] = v - tv

    # input vector
    # u = [delta, accel]
    # delta: steering angle
    # accel: acceleration
    ustar = -Kstar @ x

    # calc steering input
    ff = math.atan2(WB * k, 1)  # feedforward steering angle
    fb = pi_2_pi(ustar[0, 0])   # feedback steering angle
    delta = ff + fb

    # calc accel input
    accel = ustar[1, 0]

    return delta, ind, e, th_e, accel


def calc_nearest_index(state: VehicleState, cx: list, cy: list, cyaw: list) -> tuple:
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)
    ind = d.index(mind)
    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def do_simulation(cx: list, cy: list, cyaw: list,
                  ck: list, speed_profile: list, goal: list) -> None:
    MAX_T = 500.0  # max simulation time
    goal_dis = 0.3
    stop_speed = 0.05

    state = VehicleState(x=0.0, y=0.0, yaw=0.0, v=0.0)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]

    e, e_th = 0.0, 0.0
    last_dl = 0.0
    while MAX_T >= time:
        dl, target_ind, e, e_th, acc = lqr_speed_steering_control(
            state, cx, cy, cyaw, ck, e, e_th, speed_profile, lqr_Q, lqr_R)

        dl = max(-max_steer, min(dl, max_steer))
        acc = max(-max_acc, min(acc, max_acc))
        # steering low-pass filter
        steering = dl * 0.75 + last_dl * 0.25
        last_dl = steering

        state = update(state, acc, steering)

        if abs(state.v) <= stop_speed:
            target_ind += 1

        time = time + dt

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.hypot(dx, dy) <= goal_dis:
            print("Goal")
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)

        if show_animation:
            plt.cla()
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plot_car(state.x, state.y, state.yaw, steering)
            plot_console(steering, acc)
            plt.axis("equal")
            plt.grid(True)
            plt.xlim(-10, 38)
            plt.ylim(-15, 20)
            plt.legend(loc='lower left')
            plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2))
                      + ", target index:" + str(target_ind))
            plt.pause(0.01)

    plt.show()


def calc_speed_profile(cyaw: list, target_speed: float) -> list:
    speed_profile = [target_speed] * len(cyaw)
    direction = 1.0

    # Set stop point
    for i in range(len(cyaw) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

        if switch:
            speed_profile[i] = 0.0

    # slow down
    for i in range(50):
        speed_profile[-i] = target_speed / (60 - i)
        if speed_profile[-i] <= 1.0 / 3.6:
            speed_profile[-i] = 1.0 / 3.6

    return speed_profile


def main():
    ax = [0.0, 15.0, 31.25, 25.0, 18.75, 7.5, -2.5]
    ay = [0.0, -7.5, -12.25, 16.25, 7.5, 12.5, -5.0]
    goal = [ax[-1], ay[-1]]

    cx, cy, cyaw, ck, s = cubic_spline.calc_spline_course(ax, ay, ds=0.1)
    target_speed = 20.0 / 3.6  # simulation parameter km/h -> m/s
    sp = calc_speed_profile(cyaw, target_speed)

    do_simulation(cx, cy, cyaw, ck, sp, goal)


if __name__ == '__main__':
    main()
