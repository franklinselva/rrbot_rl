import sympy as sym
from sympy import MatMul, Matrix, simplify
from sympy import cos, sin
from math import pi

theta_1 = sym.Symbol('theta1')
theta_2 = sym.Symbol('theta2')

a = cos(theta_1)

t1 = simplify(Matrix([[cos(0), -sin(0)*cos(pi/2), sin(0)*sin(pi/2), 2*cos(0)],
                      [sin(0), cos(0)*cos(pi/2), -cos(0)*sin(pi/2), 2*sin(0)],
                      [0, sin(pi/2), cos(pi/2), 0.1],
                      [0, 0, 0, 1]]))

t2 = simplify(Matrix([[cos(theta_1), -sin(theta_1)*cos(0), sin(theta_1)*sin(0), 1*cos(theta_1)],
                      [sin(theta_1), cos(theta_1)*cos(0), -
                       cos(theta_1)*sin(0), 1*sin(theta_1)],
                      [0, sin(0), cos(0), 0.1],
                      [0, 0, 0, 1]]))

t3 = Matrix([[cos(theta_2), -sin(theta_2)*cos(0), sin(theta_2)*sin(0), 1*cos(theta_2)],
             [sin(theta_2), cos(theta_2)*cos(0), -
              cos(theta_2)*sin(0), 1*sin(theta_2)],
             [0, sin(0), cos(0), 0.1],
             [0, 0, 0, 1]])

T = MatMul(t1*t2*t3)

print(simplify(T))
