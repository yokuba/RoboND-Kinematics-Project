import math
import numpy as np
from sympy import symbols, cos, sin, pi, simplify
from sympy.matrices import Matrix

d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')  # link offset
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')  # link length
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')  # twist angle
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # joint angle symbols

DH_Table = {alpha0: 0, a0: 0, d1: 0.75, q1: 0,
            alpha1: -pi/2., a1: 0.35, d2: 0, q2: -pi/2. + q2,
            alpha2: 0, a2: 1.25, d3: 0,
            alpha3: -pi/2., a3: -0.054, d4: 1.5,
            alpha4: pi/2., a4: 0, d5: 0,
            alpha5: -pi/2., a5: 0, d6: 0,
            alpha6: 0, a6: 0, d7: 0.303, q7: 0}

  # Define Modified DH Transformation matrix


def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[cos(q),                     -sin(q),           0,           a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d],
                [0,                                 0,          0,          1]])
    return TF

  # Create individual transformation matrices
T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

# Incrementally build homogeneous transforms
T0_2 = simplify(T0_1 * T1_2)
T0_3 = simplify(T0_2 * T2_3)
T0_4 = simplify(T0_3 * T3_4)
T0_5 = simplify(T0_4 * T4_5)
T0_6 = simplify(T0_5 * T5_6)
T0_EE = simplify(T0_6 * T6_EE)

# Orientation correction between DH Table vs the URDF parameters
y = pi
p = - pi/2

ROT_z = Matrix([[cos(y),   -sin(y), 0, 0],
                [sin(y),    cos(y), 0, 0],
                [0,         0,      1, 0],
                [0,   0, 0, 1]])

ROT_y = Matrix([[cos(p),   0, sin(p), 0],
                [0,        1,  0,     0],
                [-sin(p),  0, cos(p), 0],
                [0,         0,  0,  1]])  # PITCH about the y axis

Rot_corr = simplify(ROT_z * ROT_y)


print("T0_1: ", T0_1.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_2: ", T0_2.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_3: ", T0_3.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_4: ", T0_4.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_5: ", T0_5.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_6: ", T0_6.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("T0_EE: ", T0_EE.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))

# Orientation correction applied to homogeneous transform between base link and end effector
T_Total = simplify(T0_EE.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}) * Rot_corr)
print("T_Total: ", T_Total)

r, p, y = symbols('r p y')

ROT_x = Matrix([[1, 0, 0],
        [0, cos(r), -sin(r)],
        [0, sin(r), cos(r)]]) #ROLL

ROT_y = Matrix([[cos(p),   0, sin(p)],
        [0,        1,  0],
        [-sin(p),  0, cos(p)]]) #PITCH


ROT_z = Matrix([[cos(y),   -sin(y), 0],
        [sin(y),    cos(y), 0],
        [0,         0,      1]])  #YAW

ROT_EE = ROT_z * ROT_y * ROT_x
