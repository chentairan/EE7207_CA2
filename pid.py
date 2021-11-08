import matplotlib.pyplot as plt

from model import TruckModel
import numpy as np
import math


def get_pid_trajectory(init_state):
    init_x, init_y, init_theta = init_state
    model = TruckModel(init_x, init_y, init_theta)

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

        # control
        u = -0.2 * y - 0.16 * theta
        if u > 30:
            u = 30
        elif u < -30:
            u = 30
        model.control(u)
        hist_u.append(u)
    return hist_x, hist_y, hist_theta, hist_u


if __name__ == "__main__":
    hist_x, hist_y, hist_theta, hist_u = get_pid_trajectory((20, 40, 0))
    plt.plot(hist_x, hist_y)
    plt.show()
