import sympy as sym
from sympy import MatMul, Matrix, simplify, solve
from sympy import cos, sin
from math import pi


class robot:
    def __init__(self) -> None:

        self.theta_1 = sym.Symbol('theta1')
        self.theta_2 = sym.Symbol('theta2')

        self.t1 = simplify(Matrix([[cos(0), -sin(0)*cos(pi/2), sin(0)*sin(pi/2), 2*cos(0)],
                                   [sin(0), cos(0)*cos(pi/2), -
                                    cos(0)*sin(pi/2), 2*sin(0)],
                                   [0, sin(pi/2), cos(pi/2), 0.1],
                                   [0, 0, 0, 1]]))

        self.t2 = simplify(Matrix([[cos(self.theta_1), -sin(self.theta_1)*cos(0), sin(self.theta_1)*sin(0), 1*cos(self.theta_1)],
                                   [sin(self.theta_1), cos(self.theta_1)*cos(0), -
                                    cos(self.theta_1)*sin(0), 1*sin(self.theta_1)],
                                   [0, sin(0), cos(0), 0.1],
                                   [0, 0, 0, 1]]))

        self.t3 = Matrix([[cos(self.theta_2), -sin(self.theta_2)*cos(0), sin(self.theta_2)*sin(0), 1*cos(self.theta_2)],
                          [sin(self.theta_2), cos(self.theta_2)*cos(0), -
                           cos(self.theta_2)*sin(0), 1*sin(self.theta_2)],
                          [0, sin(0), cos(0), 0.1],
                          [0, 0, 0, 1]])

        self.getTMatrix()
        self.getXYZ()
        self.getTheta()

    def getTMatrix(self):
        self.T = MatMul(self.t1*self.t2*self.t3)
        self.T = simplify(self.T)

        return self.T

    def getXYZ(self):
        self.px = self.T[0, 3]
        self.py = self.T[1, 3]
        self.pz = self.T[2, 3]

        return self.px, self.py, self.pz

    def getTheta(self):
        self.theta2 = solve(self.pz, self.theta_2)
        self.theta1 = solve(self.px.subs(
            self.theta_2, self.theta2), self.theta_1)

        return self.theta1, self.theta2


if __name__ == "__main__":
    dh = robot()
    theta1, theta2 = dh.getTheta()
    print(theta1)
    print(theta2)
