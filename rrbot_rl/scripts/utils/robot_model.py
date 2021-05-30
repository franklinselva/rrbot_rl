import sympy as sym
from sympy import MatMul, Matrix, simplify, solve
from sympy import cos, sin, tan, atan2, acos
from sympy.utilities.lambdify import lambdify
from math import pi


class robot:
    def __init__(self):

        self.theta_1 = sym.Symbol('theta1')
        self.theta_2 = sym.Symbol('theta2')
        self.x, self.y, self.z = sym.Symbol(
            'x'), sym.Symbol('y'), sym.Symbol('z')

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
        self.z_modified = self.z - 2
        self.theta2 = [acos((self.x*self.x + self.z_modified*self.z_modified - 1 - 1)/2),
                       -acos((self.x * self.x + self.z_modified * self.z_modified - 1 - 1)/2)]
        self.theta1 = [atan2(self.x, self.z_modified) - atan2(sin(self.theta_2), 1+cos(self.theta_2)),
                       atan2(self.x, self.z_modified) + atan2(sin(self.theta_2), 1+cos(self.theta_2))]

        self.theta1_function = lambdify(
            [self.x, self.z, self.theta_2], self.theta1)
        self.theta2_function = lambdify(
            [self.x, self.z], self.theta2)
        return self.theta1_function, self.theta2_function


if __name__ == "__main__":
    dh = robot()
    theta1, theta2 = dh.getInverseKinematics()
    dh.getForwardKinematics()
