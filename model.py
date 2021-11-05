from math import tan, sin, cos, pi


class TruckModel:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta / 180 * pi
        self.dt = 0.1
        self.v = 0.5
        self.L = 2.5

    def set_pos(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def control(self, u):
        rad_u = u / 180 * pi
        self.x = self.x + self.v * self.dt * cos(self.theta)
        self.y = self.y + self.v * self.dt * sin(self.theta)
        self.theta = self.theta + self.v * self.dt * tan(rad_u) / self.L

    def unwrap_theta(self):
        while self.theta >= pi:
            self.theta -= 2*pi
        while self.theta <= -pi:
            self.theta += 2*pi

    def observe(self):
        finish = False
        if abs(self.y) < 0.01 and abs(self.theta) < 0.01:
            finish = True
        self.unwrap_theta()

        return self.x, self.y, self.theta * 180 / pi, finish
