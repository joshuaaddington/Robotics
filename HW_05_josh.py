import kinematics as kin
from visualization import VizScene
import sympy as sp
import numpy as np
import time
np.set_printoptions(precision=4, suppress=True)

dh = [[0, 0.3, 0, np.pi/2.0],
      [0, 0, 0.3, 0]]

# An example of defining joint types which we may not have done yet.
# The 4th joint, and 4th row of the DH parameters correspond to a prismatic joint.
jt_types = ['r', 'r']

# notice how "jt_types" can be passed as an argument into "SerialArm"
arm = kin.SerialArm(dh, jt=jt_types)

# defining two different sets of joint angles
q_set1 = [0, 0]
q_set2 = [0, np.pi/2]

# calculating two different jacobians for the two different joint configurations.
J1 = arm.jacob(q_set1)
J2 = arm.jacob(q_set2)

print("from first set of q's, J is:")
print(J1)

print("from second set of q's, J is:")
print(J2)

# making a visualization
viz = VizScene()

# adding a SerialArm to the visualization, and telling it to draw the joint frames.
viz.add_arm(arm, draw_frames=True)

# setting the joint angles to draw
viz.update(qs=[q_set1])


time_to_run = 7
refresh_rate = 60

for i in range(refresh_rate * time_to_run):
    viz.update()
    time.sleep(1.0/refresh_rate)

# updating position of the arm in visualization
viz.update(qs=[q_set2])

for i in range(refresh_rate * time_to_run):
    viz.update()
    time.sleep(1.0/refresh_rate)

viz.close_viz()

dh = [[0, 0, 0, -np.pi/2],
      [0, .154, 0, np.pi/2],
      [0, .25, 0, 0],
      [-np.pi/2, 0, 0, -np.pi/2],
      [-np.pi/2, 0, 0, np.pi/2],
      [np.pi/2, .263, 0, 0]]

# An example of defining joint types which we may not have done yet.
# The 4th joint, and 4th row of the DH parameters correspond to a prismatic joint.
jt_types = ['r', 'r', 'p', 'r', 'r', 'r']

# notice how "jt_types" can be passed as an argument into "SerialArm"
arm = kin.SerialArm(dh, jt=jt_types, tip = [[0,0,-1,0],
                                             [0,1,0,0],
                                             [1,0,0,0],
                                             [0,0,0,1]])

# defining two different sets of joint angles
q_set1 = [0, 0, .1, 0, 0, 0]

# calculating two different jacobians for the two different joint configurations.
J1 = arm.jacob(q_set1)

print("from first set of q's, J is:")
print(J1)

# making a visualization
viz = VizScene()

# adding a SerialArm to the visualization, and telling it to draw the joint frames.
viz.add_arm(arm, draw_frames=True)

# setting the joint angles to draw
viz.update(qs=[q_set1])

time_to_run = 20
refresh_rate = 60

for i in range(refresh_rate * time_to_run):
    viz.update()
    time.sleep(1.0/refresh_rate)