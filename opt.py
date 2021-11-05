from casadi import *
from model import TruckModel
import matplotlib.pyplot as plt


def func(x, u):
    _x = 0.5 * cos(x[2])
    _y = 0.5 * sin(x[2])
    _theta = 0.5 * tan(u[0]) / 2.5
    return vertcat(_x, _y, _theta)


def get_opt_trajectory(init_state):
    init_x, init_y, init_theta = init_state
    while init_theta > 180:
        init_theta -= 360
    while init_theta < -180:
        init_theta += 360

    N = 1000  # number of control intervals

    opti = Opti()  # Optimization problem

    # ---- decision variables ---------
    X = opti.variable(3, N + 1)  # state trajectory
    x = X[0, :]
    y = X[1, :]
    theta = X[2, :]

    U = opti.variable(1, N)  # control trajectory (throttle)

    T = opti.variable()

    opti.minimize(T)

    dt = T / N  # length of a control interval

    for k in range(N):
        k1 = func(X[:, k], U[:, k])
        k2 = func(X[:, k] + dt / 2 * k1, U[:, k])
        k3 = func(X[:, k] + dt / 2 * k2, U[:, k])
        k4 = func(X[:, k] + dt * k3, U[:, k])
        x_next = X[:, k] + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        opti.subject_to(X[:, k + 1] == x_next)
        # opti.subject_to(X[:, k + 1] == X[:, k] + dt * func(X[:, k], U[:, k]))

    # ---- path constraints -----------
    opti.subject_to(opti.bounded(0, x, 300))  # control is limited
    opti.subject_to(opti.bounded(0, T, 2000))  # control is limited
    opti.subject_to(opti.bounded(-100, y, 100))  # control is limited
    opti.subject_to(opti.bounded(-pi, theta, pi))  # control is limited
    opti.subject_to(opti.bounded(-pi / 6, U, pi / 6))  # control is limited

    # ---- boundary conditions --------
    opti.subject_to(x[0] == init_x)  # start at position 0 ...
    opti.subject_to(y[0] == init_y)  # ... from stand-still
    opti.subject_to(theta[0] == init_theta * pi / 180.0)  # ... from stand-still
    opti.subject_to(y[-1] * y[-1] <= 0.01 * 0.01)  # ... from stand-still
    opti.subject_to(theta[-1] * theta[-1] <= 0.01 * 0.01)  # ... from stand-still

    # ---- initial values for solver ---
    opti.set_initial(U, 0)

    # ---- solve NLP              ------
    p_opts = dict(print_time=False, verbose=False)
    s_opts = dict(print_level=0)
    opti.solver("ipopt", p_opts, s_opts)  # set numerical backend
    sol = opti.solve()  # actual solve

    # plt.plot(sol.value(x), sol.value(y), 'red')
    # plt.show()

    return sol.value(x), sol.value(y), [i * 180 / pi for i in sol.value(theta)], [i * 180 / pi for i in
                                                                                  sol.value(U)], sol.value(T)


if __name__ == "__main__":
    get_opt_trajectory((10, -30, 90))
    # model = mpc(10, -30, 90)
    # model = mpc(40, 30, 220)
    # model = mpc(50, 10, -10)
