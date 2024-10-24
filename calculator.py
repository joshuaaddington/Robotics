import numpy as np
import sympy as sp

q1 = sp.symbols("q1")
q2 = sp.symbols("q2")
d1 = sp.symbols("d1")
a2 = sp.symbols("a2")
R1z = sp.Matrix([[sp.cos(q1), -sp.sin(q1), 0],
                [sp.sin(q1), sp.cos(q1), 0],
                [0, 0, 1]])
R1x = sp.Matrix([[1, 0, 0],
                [0, sp.cos(sp.pi/2), -sp.sin(sp.pi/2)],
                [0, sp.sin(sp.pi/2), sp.cos(sp.pi/2)]])
R1total = R1z @ R1x
sp.pprint(R1total)
A0_to_1 = sp.Matrix([[sp.cos(q1), 0, sp.sin(q1), 0],
                     [sp.sin(q1), 0, -sp.cos(q1), 0],
                     [0, 1, 0, d1],
                     [0, 0, 0, 1]])
A1_to_2 = sp.Matrix([[sp.cos(q2), -sp.sin(q2), 0, a2*sp.cos(q2)],
                     [sp.sin(q2), sp.cos(q2), 0, a2*sp.sin(q2)],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])
A0_to_2 = A0_to_1 @ A1_to_2
r1_to_2_in_1 = A1_to_2[:3, 3]
r1_to_2_in_0 = R1total @ r1_to_2_in_1
print("r1_to_2_in_0:\n")
sp.pprint(r1_to_2_in_0)
print("A0_to_2:")
sp.pprint(A0_to_2)