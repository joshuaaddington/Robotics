import transforms as tr
import numpy as np
from visualization import VizScene 
import time

# T1_in_0 = np.array([[1,0,0,0],
#                     [0,1,0,1],
#                     [0,0,1,1],
#                     [0,0,0,1]])

# T2_in_0 = np.array([[1,0,0,-.5],
#                     [0,1,0,1.5],
#                     [0,0,1,1],
#                     [0,0,0,1]])

# T3_in_0 = np.array([[0,1,0,-.5],
#                     [1,0,0,1.5],
#                     [0,0,-1,3],
#                     [0,0,0,1]])

# T3_in_2 = np.array([[0,1,0,0],
#                     [1,0,0,0],
#                     [0,0,-1,2],
#                     [0,0,0,1]])

# print("T1_in_0:\n", T1_in_0,"\n")
# print("T2_in_0:\n", T2_in_0,"\n")
# print("T3_in_0:\n", T3_in_0,"\n")
# print("T3_in_2:\n", T3_in_2,"\n")

# Tw_to_frame1 = np.array([[1, 0,  0,  1],
#                          [0, 0, -1,  0],
#                          [0, 1,  0,  0],
#                          [0, 0,  0,  1]])

# viz = VizScene()
# viz.add_frame(np.eye(4), label='world')
# viz.add_frame(T2_in_0, label = "T2_in_0")
# viz.add_frame(T1_in_0, label = "T1_in_0")
# viz.add_frame(T3_in_0, label = "T3_in_0")
# viz.add_frame(T3_in_2, label = "T3_in_2")

# time_to_run = 20
# refresh_rate = 60
# for i in range(refresh_rate * time_to_run):
#     t = time.time()
#     viz.update(As=[np.eye(4), T2_in_0, T1_in_0, T3_in_0, T3_in_2])

# viz.close_viz()

import sympy as sp

# Define the symbols for the joint angles and link lengths
q1, q2, q3, q4, q5, d1, a1, a2 = sp.symbols('q1 q2 q3 q4 q5 d1 a1 a2')

# Define the transformation matrices T01, T02, T03, T04, T05

# T01
T01 = sp.Matrix([
    [sp.cos(q1), 0, -sp.sin(q1), 0],
    [sp.sin(q1), 0, sp.cos(q1), 0],
    [0, -1, 0, d1],
    [0, 0, 0, 1]
])

# T02
T02 = sp.Matrix([
    [sp.cos(q2), -sp.sin(q2), 0, a1 * sp.cos(q2)],
    [sp.sin(q2), sp.cos(q2), 0, a1 * sp.sin(q2)],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

# T03
T03 = sp.Matrix([
    [sp.cos(q3), -sp.sin(q3), 0, a2 * sp.cos(q3)],
    [sp.sin(q3), sp.cos(q3), 0, a2 * sp.sin(q3)],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

# T04
T04 = sp.Matrix([
    [sp.sin(q4), 0, -sp.cos(q4), 0],
    [-sp.cos(q4), 0, -sp.sin(q4), 0],
    [0, -1, 0, 0],
    [0, 0, 0, 1]
])

# T05
T05 = sp.Matrix([
    [sp.cos(q5), -sp.sin(q5), 0, 0],
    [sp.sin(q5), sp.cos(q5), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

# Calculate the overall transformation matrix T05
T_total = T01 * T02 * T03 * T04 * T05
T_total_simplified = sp.simplify(T_total)
sp.pprint(T_total_simplified)