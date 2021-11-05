from casadi import *
from model import TruckModel
import matplotlib.pyplot as plt


def func(x, u):
    _x = 0.5 * cos(x[2])
    _y = 0.5 * sin(x[2])
    _theta = 0.5 * tan(u[0]) / 2.5
    return vertcat(_x, _y, _theta)


def mpc(init_x, init_y, init_theta, last_u, N):
    opti = Opti()  # Optimization problem

    # ---- decision variables ---------
    X = opti.variable(3, N + 1)  # state trajectory
    x = X[0, :]
    y = X[1, :]
    theta = X[2, :]

    U = opti.variable(1, N)  # control trajectory (throttle)

    opti.minimize(sumsqr(y) + sumsqr(theta))

    dt = 0.1  # length of a control interval

    for k in range(N):
        opti.subject_to(X[:, k + 1] == X[:, k] + dt * func(X[:, k], U[:, k]))

    # ---- path constraints -----------
    opti.subject_to(opti.bounded(-100, y, 100))  # control is limited
    opti.subject_to(opti.bounded(-pi, theta, pi))  # control is limited
    opti.subject_to(opti.bounded(-pi / 6, U, pi / 6))  # control is limited

    # ---- boundary conditions --------
    opti.subject_to(x[0] == init_x)  # start at position 0 ...
    opti.subject_to(y[0] == init_y)  # ... from stand-still
    opti.subject_to(theta[0] == init_theta)  # ... from stand-still

    # ---- initial values for solver ---
    opti.set_initial(U, last_u)

    # ---- solve NLP              ------
    p_opts = dict(print_time=False, verbose=False)
    s_opts = dict(print_level=0)
    opti.solver("ipopt", p_opts, s_opts)  # set numerical backend
    sol = opti.solve()  # actual solve

    return sol.value(U)[0]


def get_mpc_trajectory(init_state):
    init_x, init_y, init_theta = init_state
    model = TruckModel(init_x, init_y, init_theta)
    # model = TruckModel(20, 40, 0)
    # model = TruckModel(10, -30, 90)
    # model = TruckModel(40, 30, 220)
    # model = TruckModel(50, 10, -10)

    N = 10

    last_u = 0
    finish = False

    hist_x = []
    hist_y = []
    hist_theta = []
    hist_u = []

    step = 0

    while True:
        step += 1

        # observe
        x, y, theta, finish = model.observe()
        hist_x.append(x)
        hist_y.append(y)
        hist_theta.append(theta)

        if finish:
            break

        # set input
        last_u = mpc(x, y, theta / 180 * pi, last_u, N)
        hist_u.append(last_u * 180 / pi)
        # control
        model.control(last_u * 180 / pi)
    return hist_x, hist_y, hist_theta, hist_u


if __name__ == "__main__":
    model = TruckModel(20, 40, 0)
    # model = TruckModel(10, -30, 90)
    # model = TruckModel(40, 30, 220)
    # model = TruckModel(50, 10, -10)

    last_u = 0
    finish = False

    hist_x = []
    hist_y = []

    step = 0

    while True:
        step += 1
        print(f"-----step {step}------")

        # observe
        x, y, theta, finish = model.observe()
        hist_x.append(x)
        hist_y.append(y)
        print(f"pose: ({x}, {y}, {theta})")

        if finish:
            break

        # set input
        last_u = mpc(x, y, theta / 180 * pi, last_u)

        # control
        model.control(last_u * 180 / pi)
        print(f"control: {last_u * 180 / pi}")

    print(f"Final converge time: {step * model.dt}")
    plt.plot(hist_x, hist_y, "red")
    plt.show()
