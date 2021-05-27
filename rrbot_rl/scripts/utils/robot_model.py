import sympy as sym
from sympy import MatMul, Matrix, simplify, solve
from sympy import cos, sin
from sympy.utilities.lambdify import lambdify
from math import pi


class robot:
    def __init__(self):

        self.theta_1 = sym.Symbol('theta1')
        self.theta_2 = sym.Symbol('theta2')
        self.xEE, self.yEE, self.zEE = sym.Symbol(
            'xEE'), sym.Symbol('yEE'), sym.Symbol('zEE')

        self.t1 = simplify(Matrix([[cos(0), -sin(0)*cos(pi/2), sin(0)*sin(pi/2), 0.1*cos(0)],
                                   [sin(0), cos(0)*cos(pi/2), -
                                    cos(0)*sin(pi/2), 0.1*sin(0)],
                                   [0, sin(pi/2), cos(pi/2), 2],
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
        self.getForwardKinematics()
        self.getInverseKinematics()

    def getTMatrix(self):
        self.T = MatMul(self.t1*self.t2*self.t3)
        self.T = simplify(self.T)

        return self.T

    def getForwardKinematics(self):
        self.px = self.T[0, 3]
        self.py = self.T[1, 3]
        self.pz = self.T[2, 3]

        self.px_function = lambdify([self.theta_1, self.theta_2], self.px)
        self.py_function = lambdify([self.theta_1, self.theta_2], self.py)
        self.pz_function = lambdify([self.theta_1, self.theta_2], self.pz)

        return self.px_function, self.py_function, self.pz_function

    def getInverseKinematics(self):
        self.xEq = self.xEE == self.px
        self.yEq = self.yEE == self.py
        self.zEq = self.zEE == self.pz

        self.IK = solve([self.xEq, self.yEq], [self.theta_1, self.theta_2])
        print(self.IK)
        self.theta1_function = lambdify(
            [self.xEE, self.yEE, self.zEE], self.IK[0])
        self.theta2_function = lambdify(
            [self.xEE, self.yEE, self.zEE], self.IK[1])

        return self.theta1_function, self.theta2_function


if __name__ == "__main__":
    dh = robot()
    theta1, theta2 = dh.getInverseKinematics()
    dh.getForwardKinematics()
