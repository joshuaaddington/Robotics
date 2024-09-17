import sympy as sp
from sympy import pprint
import numpy as np
import transforms as tf
import time
from visualization import VizScene

theta = sp.symbols("theta")
phi = sp.symbols("phi")

print("Problem 7")
Final_Rot_7 = tf.rotx(theta) @ tf.roty(phi) @ tf.rotz(sp.pi) @ tf.roty(-phi) @ tf.rotx(-theta)
print(Final_Rot_7)

Final_Rot_8 = tf.rotz(sp.pi/2) @ tf.roty(0) @ tf.rotz(sp.pi/4)
print("\n Problem 8")
print(Final_Rot_8)

# Tw_to_frame1 = np.eye(4)
                         
# viz = VizScene()
# viz.add_frame(np.eye(4), label='frame0')
# viz.add_frame(Tw_to_frame1, label='frame1')

# time_to_run = 10
# refresh_rate = 60
# for i in range(refresh_rate * time_to_run*10):
#     t = time.time()
#     Tw_to_frame1 = np.array([[np.cos(np.pi/2*t), -np.sin(np.pi/2*t), 0, 1],
#                              [np.sin(np.pi/2*t), np.cos(np.pi/2*t), 0, 1],
#                              [0, 0, 1, 0],
#                              [0, 0, 0, 1]])
#     viz.update(As=[np.eye(4), Tw_to_frame1])
#     time.sleep(1.0/refresh_rate)
    
# viz.close_viz()