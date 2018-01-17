import math
import numpy as np

# d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link offset
# a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #link length
# alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #twist angle
# q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #joint angle symbols
q1 = 0.
q2 = 0.
q3 = 0.
q4 = 0.
q5 = 0.
q6 = 0.
q7 = 0.

  # Define Modified DH Transformation matrix
def TF_Matrix(alpha, a, d, q):
  TF = np.matrix([[ math.cos(q), -math.sin(q), 0, a],
    [ math.sin(q)*math.cos(alpha), math.cos(q)*math.cos(alpha), -math.sin(alpha), -math.sin(alpha)*d],
    [ math.sin(q)*math.sin(alpha), math.cos(q)*math.sin(alpha), math.cos(alpha), math.cos(alpha)*d],
    [ 0,      0,      0,  1]])
  return TF

DH_Table = dict( alpha0 = 0, a0 = 0, d1 = 0.75, q1 = q1,
   alpha1 = -math.pi/2., a1 = 0.35, d2 = 0, q2 = -math.pi/2. + q2,
   alpha2 = 0, a2 = 1.25, d3 = 0, q3 = q3,
   alpha3 = -math.pi/2., a3 = -0.054, d4 = 1.5, q4 = q4,
   alpha4 = math.pi/2., a4 = 0, d5 = 0, q5 = q5,
   alpha5 = -math.pi/2., a5 = 0, d6 = 0, q6 = q6,
   alpha6 = 0, a6 = 0, d7 = 0.303, q7 = 0)

  # Create individual transformation matrices
T0_1 = TF_Matrix(DH_Table['alpha0'], DH_Table['a0'], DH_Table['d1'], DH_Table['q1'])
T1_2 = TF_Matrix(DH_Table['alpha1'], DH_Table['a1'], DH_Table['d2'], DH_Table['q2'])
T2_3 = TF_Matrix(DH_Table['alpha2'], DH_Table['a2'], DH_Table['d3'], DH_Table['q3'])
T3_4 = TF_Matrix(DH_Table['alpha3'], DH_Table['a3'], DH_Table['d4'], DH_Table['q4'])
T4_5 = TF_Matrix(DH_Table['alpha4'], DH_Table['a4'], DH_Table['d5'], DH_Table['q5'])
T5_6 = TF_Matrix(DH_Table['alpha5'], DH_Table['a5'], DH_Table['d6'], DH_Table['q6'])
T6_EE = TF_Matrix(DH_Table['alpha6'], DH_Table['a6'], DH_Table['d7'], DH_Table['q7'])

print('The T0_1 joint angle is ', T0_1)
print('The T1_2 joint angle is ', T1_2)
print('The T2_3 joint angle is ', T2_3)
print('The T3_4 joint angle is ', T3_4)
print('The T4_5 joint angle is ', T4_5)
print('The T5_6 joint angle is ', T5_6)
print('The T6_EE joint angle is ', T6_EE)
