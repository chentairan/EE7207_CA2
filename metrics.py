from fuzzy import get_fuzzy_trajectory, set_fuzzy_controller
from mpc import get_mpc_trajectory
from opt import get_opt_trajectory
from pid import get_pid_trajectory
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from tqdm import tqdm


def plot_control_space():
    # if [True] run code or [False] just load pre-calculated data to plot figure
    run_code = False
    if run_code:
        sim = set_fuzzy_controller()
        # We can simulate at higher resolution with full accuracy
        upsampled_y = np.linspace(-100, 100, 251)
        upsampled_theta = np.linspace(-180, 180, 451)
        y, theta = np.meshgrid(upsampled_y, upsampled_theta)
        u = np.zeros_like(y)

        # Loop through the system 901*501 times to collect the control surface
        for i in tqdm(range(451)):
            for j in range(251):
                sim.input['y'] = y[i, j]
                sim.input['theta'] = theta[i, j]
                sim.compute()
                u[i, j] = sim.output['u']

        np.savez("data/control_space.npz", y=y, theta=theta, u=u)
    else:
        control_space = np.load("data/control_space.npz")
        y = control_space["y"]
        theta = control_space["theta"]
        u = control_space["u"]

    fig = plt.figure(figsize=(8, 8), dpi=326)
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(y, theta, u, rstride=8, cstride=8, cmap='viridis',
                           linewidth=32, antialiased=True)

    cset = ax.contourf(y, theta, u, zdir='z', offset=-30, cmap='viridis', alpha=0.5)
    cset = ax.contourf(y, theta, u, zdir='y', offset=-200, cmap='viridis', alpha=0.5)
    cset = ax.contourf(y, theta, u, zdir='x', offset=-120, cmap='viridis', alpha=0.5)

    ax.view_init(30, 20)
    ax.set_xlabel('$y/m$')
    ax.set_ylabel('$\\theta/degree$')
    ax.set_zlabel('$u/degree$')

    plt.savefig('./image/control_space.png')
    fig.delaxes(ax)
    plt.figure(figsize=(12.8, 9.6), dpi=326)
    plt.cla()


def plot_xt(fuzzy, pid, mpc, opt, T, title_name, axis_name):
    dt = np.linspace(0, (len(fuzzy) - 1) * 0.1, len(fuzzy))
    plt.plot(dt, fuzzy, 'blue', label='Fuzzy Control')
    dt = np.linspace(0, (len(pid) - 1) * 0.1, len(pid))
    plt.plot(dt, pid, 'orange', label='P Control')
    dt = np.linspace(0, (len(mpc) - 1) * 0.1, len(mpc))
    plt.plot(dt, mpc, 'red', label='NMPC')
    dt = np.linspace(0, T, len(opt))
    plt.plot(dt, opt, 'green', label='Optimal Control')

    plt.grid()
    plt.title(title_name)
    plt.xlabel('t/s')
    plt.ylabel(axis_name)
    plt.legend(loc='best')


def plot_xy(fuzzy_x, fuzzy_y, pid_x, pid_y, mpc_x, mpc_y, opt_x, opt_y):
    plt.plot(fuzzy_x, fuzzy_y, 'blue', label='Fuzzy Control')
    plt.plot(pid_x, pid_y, 'orange', label='P Control')
    plt.plot(mpc_x, mpc_y, 'red', label='NMPC')
    plt.plot(opt_x, opt_y, 'green', label='Optimal Control')

    plt.grid()
    plt.title("$x-y$")
    plt.xlabel('x/m')
    plt.ylabel('y/m')
    plt.legend(loc='best')


if __name__ == "__main__":
    # if [True] run code or [False] just load pre-calculated data to plot figure
    run_code = False
    init_states = [(20, 40, 0), (10, -30, 90), (40, 30, 220), (50, 10, -10)]
    for i, init_state in enumerate(init_states):
        if (run_code):
            # run code
            fuzzy_x, fuzzy_y, fuzzy_theta, fuzzy_u = get_fuzzy_trajectory(init_state)
            pid_x, pid_y, pid_theta, pid_u = get_pid_trajectory(init_state)
            mpc_x, mpc_y, mpc_theta, mpc_u = get_mpc_trajectory(init_state)
            opt_x, opt_y, opt_theta, opt_u, T = get_opt_trajectory(init_state)

            # save data
            np.savez(f"data/fuzzy{i + 1}.npz", fuzzy_x=fuzzy_x, fuzzy_y=fuzzy_y, fuzzy_theta=fuzzy_theta,
                     fuzzy_u=fuzzy_u)
            np.savez(f"data/pid{i + 1}.npz", pid_x=pid_x, pid_y=pid_y, pid_theta=pid_theta, pid_u=pid_u)
            np.savez(f"data/mpc{i + 1}.npz", mpc_x=mpc_x, mpc_y=mpc_y, mpc_theta=mpc_theta, mpc_u=mpc_u)
            np.savez(f"data/opt{i + 1}.npz", opt_x=opt_x, opt_y=opt_y, opt_theta=opt_theta, opt_u=opt_u, T=T)
        else:
            # load data
            fuzzy = np.load(f"data/fuzzy{i + 1}.npz")
            pid = np.load(f"data/pid{i + 1}.npz")
            mpc = np.load(f"data/mpc{i + 1}.npz")
            opt = np.load(f"data/opt{i + 1}.npz")

            # set data
            T = opt["T"]
            fuzzy_x, pid_x, mpc_x, opt_x = fuzzy["fuzzy_x"], pid["pid_x"], mpc["mpc_x"], opt["opt_x"]
            fuzzy_y, pid_y, mpc_y, opt_y = fuzzy["fuzzy_y"], pid["pid_y"], mpc["mpc_y"], opt["opt_y"]
            fuzzy_theta, pid_theta, mpc_theta, opt_theta = fuzzy["fuzzy_theta"], pid["pid_theta"], mpc["mpc_theta"], \
                                                           opt["opt_theta"]
            fuzzy_u, pid_u, mpc_u, opt_u = fuzzy["fuzzy_u"], pid["pid_u"], mpc["mpc_u"], opt["opt_u"]

        # plot results
        fig = plt.figure(figsize=(12.8, 9.6), dpi=326)
        fig.suptitle(f'Initial state: x = {init_state[0]}, y = {init_state[1]}, $\\theta$ = {init_state[2]}')
        plt.subplot(2, 2, 1)
        plot_xy(fuzzy_x, fuzzy_y, pid_x, pid_y, mpc_x, mpc_y, opt_x, opt_y)
        plt.subplot(2, 2, 2)
        plot_xt(fuzzy_y, pid_y, mpc_y, opt_y, T, '$y-t$', 'y/m')
        plt.subplot(2, 2, 3)
        plot_xt(fuzzy_theta, pid_theta, mpc_theta, opt_theta, T, '$\\theta-t$', '$\\theta/degree$')
        plt.subplot(2, 2, 4)
        plot_xt(fuzzy_u, pid_u, mpc_u, opt_u, T, '$u-t$', 'u/degree')
        plt.savefig(f'image/init{i + 1}.png')
        plt.clf()

    # plot control space
    # plot_control_space()
