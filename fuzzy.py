import numpy as np
import math
import skfuzzy as fuzz
import skfuzzy.control as ctrl
from model import TruckModel
import matplotlib.pyplot as plt


def set_fuzzy_controller():
    y_range = np.linspace(-100, 100, 1000)
    theta_range = np.linspace(-180, 180, 1000)
    u_range = np.linspace(-30, 30, 1000)

    # Create the three fuzzy variables - two inputs, one output
    y = ctrl.Antecedent(y_range, 'y')
    theta = ctrl.Antecedent(theta_range, 'theta')
    u = ctrl.Consequent(u_range, 'u')

    # Here we use the convenience `automf` to populate the fuzzy variables with
    # terms. The optional kwarg `names=` lets us specify the names of our Terms.
    y_names = ['ab', 'ac', 'ce', 'bc', 'be']
    theta_names = ['ao', 'ar', 'ah', 'hz', 'bh', 'br', 'bo']
    u_names = ['nb', 'nm', 'ns', 'ze', 'ps', 'pm', 'pb']

    # custom y member function
    y['be'] = fuzz.trapmf(y.universe, [-100, -100, -40, -12.5])
    y['bc'] = fuzz.trimf(y.universe, [-20, -10, 0])
    y['ce'] = fuzz.trimf(y.universe, [-7.5, 0, 7.5])
    y['ac'] = fuzz.trimf(y.universe, [0, 10, 20])
    y['ab'] = fuzz.trapmf(y.universe, [12.5, 40, 100, 100])

    # custom theta member function
    theta['bo'] = fuzz.trapmf(theta.universe, [-180, -180, -125, -87.5])
    theta['br'] = fuzz.trimf(theta.universe, [-100, -68.75, -37.5])
    theta['bh'] = fuzz.trimf(theta.universe, [-50, -25, 0])
    theta['hz'] = fuzz.trimf(theta.universe, [-12.5, 0, 12.5])
    theta['ah'] = fuzz.trimf(theta.universe, [0, 25, 50])
    theta['ar'] = fuzz.trimf(theta.universe, [37.5, 68.75, 100])
    theta['ao'] = fuzz.trapmf(theta.universe, [87.5, 125, 180, 180])

    # custom u member function
    u['nb'] = fuzz.trimf(u.universe, [-30, -30, -17.5])
    u['nm'] = fuzz.trimf(u.universe, [-25, -15, -5])
    u['ns'] = fuzz.trimf(u.universe, [-12.5, -6.25, 0])
    u['ze'] = fuzz.trimf(u.universe, [-5, -0, 5])
    u['ps'] = fuzz.trimf(u.universe, [0, 6.25, 12.5])
    u['pm'] = fuzz.trimf(u.universe, [5, 15, 25])
    u['pb'] = fuzz.trimf(u.universe, [17.5, 30, 30])

    # view member function
    # y.view()
    # theta.view()
    # u.view()

    # set fuzzy rule
    rule1 = ctrl.Rule(theta['bo'] & y['be'], u['pb'])
    rule2 = ctrl.Rule(theta['bo'] & y['bc'], u['pb'])
    rule3 = ctrl.Rule(theta['bo'] & y['ce'], u['pm'])
    rule4 = ctrl.Rule(theta['bo'] & y['ac'], u['pm'])
    rule5 = ctrl.Rule(theta['bo'] & y['ab'], u['ps'])

    rule6 = ctrl.Rule(theta['br'] & y['be'], u['pb'])
    rule7 = ctrl.Rule(theta['br'] & y['bc'], u['pb'])
    rule8 = ctrl.Rule(theta['br'] & y['ce'], u['pm'])
    rule9 = ctrl.Rule(theta['br'] & y['ac'], u['ps'])
    rule10 = ctrl.Rule(theta['br'] & y['ab'], u['ns'])

    rule11 = ctrl.Rule(theta['bh'] & y['be'], u['pb'])
    rule12 = ctrl.Rule(theta['bh'] & y['bc'], u['pm'])
    rule13 = ctrl.Rule(theta['bh'] & y['ce'], u['ps'])
    rule14 = ctrl.Rule(theta['bh'] & y['ac'], u['ns'])
    rule15 = ctrl.Rule(theta['bh'] & y['ab'], u['nm'])

    rule16 = ctrl.Rule(theta['hz'] & y['be'], u['pm'])
    rule17 = ctrl.Rule(theta['hz'] & y['bc'], u['pm'])
    rule18 = ctrl.Rule(theta['hz'] & y['ce'], u['ze'])
    rule19 = ctrl.Rule(theta['hz'] & y['ac'], u['nm'])
    rule20 = ctrl.Rule(theta['hz'] & y['ab'], u['nm'])

    rule21 = ctrl.Rule(theta['ah'] & y['be'], u['pm'])
    rule22 = ctrl.Rule(theta['ah'] & y['bc'], u['ps'])
    rule23 = ctrl.Rule(theta['ah'] & y['ce'], u['ns'])
    rule24 = ctrl.Rule(theta['ah'] & y['ac'], u['nm'])
    rule25 = ctrl.Rule(theta['ah'] & y['ab'], u['nb'])

    rule26 = ctrl.Rule(theta['ar'] & y['be'], u['ps'])
    rule27 = ctrl.Rule(theta['ar'] & y['bc'], u['ns'])
    rule28 = ctrl.Rule(theta['ar'] & y['ce'], u['nm'])
    rule29 = ctrl.Rule(theta['ar'] & y['ac'], u['nb'])
    rule30 = ctrl.Rule(theta['ar'] & y['ab'], u['nb'])

    rule31 = ctrl.Rule(theta['ao'] & y['be'], u['ns'])
    rule32 = ctrl.Rule(theta['ao'] & y['bc'], u['nm'])
    rule33 = ctrl.Rule(theta['ao'] & y['ce'], u['nm'])
    rule34 = ctrl.Rule(theta['ao'] & y['ac'], u['nb'])
    rule35 = ctrl.Rule(theta['ao'] & y['ab'], u['nb'])

    rule = [rule1, rule2, rule3, rule4, rule5,
            rule6, rule7, rule8, rule9, rule10,
            rule11, rule12, rule13, rule14, rule15,
            rule16, rule17, rule18, rule19, rule20,
            rule21, rule22, rule23, rule24, rule25,
            rule26, rule27, rule28, rule29, rule30,
            rule31, rule32, rule33, rule34, rule35]

    steering_ctrl = ctrl.ControlSystem(rule)

    sim = ctrl.ControlSystemSimulation(steering_ctrl)
    return sim


def get_fuzzy_trajectory(init_state):
    init_x, init_y, init_theta = init_state
    model = TruckModel(init_x, init_y, init_theta)
    # model = TruckModel(10, -30, 90)
    # model = TruckModel(40, 30, 220)
    # model = TruckModel(50, 10, -10)

    fuzzy_controller = set_fuzzy_controller()
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
        fuzzy_controller.input['y'] = y
        fuzzy_controller.input['theta'] = theta

        # control
        fuzzy_controller.compute()
        model.control(fuzzy_controller.output['u'])
        hist_u.append(fuzzy_controller.output['u'])
    return hist_x, hist_y, hist_theta, hist_u


if __name__ == "__main__":
    model = TruckModel(20, 40, 0)
    # model = TruckModel(10, -30, 90)
    # model = TruckModel(40, 30, 220)
    # model = TruckModel(50, 10, -10)

    fuzzy_controller = set_fuzzy_controller()
    finish = False

    hist_x = []
    hist_y = []

    step = 0

    while True:
        print(f"-----step {step}------")
        step += 1

        # observe
        x, y, theta, finish = model.observe()
        hist_x.append(x)
        hist_y.append(y)
        print(f"pose: ({x}, {y}, {theta})")

        if finish:
            break

        # set input
        fuzzy_controller.input['y'] = y
        fuzzy_controller.input['theta'] = theta

        # control
        fuzzy_controller.compute()
        model.control(fuzzy_controller.output['u'])
        print(f"control: {fuzzy_controller.output['u']}")

    print(f"Final converge time: {step * model.dt}")
    plt.plot(hist_x, hist_y, "red")
    plt.show()
