import transforms as tf
import kinematics as kn
import numpy as np
import sympy as sp

#Problem 1
# Completed transforms.py I will attach the file to the assignment to show work done including screenshots of visualization

R1 = tf.roty(np.pi/2)
R2 = tf.rotz(np.pi/4)
R_tot = R1 @ R2
R_axis = tf.R2axis(R_tot)
print(R_axis)

R_quat = tf.R2quatuat(R_tot)
print(R_quat)

#######################################################################################################
## problem 2-a result: [1.7177715174584016, 0.357406744336593, 0.862856209461017, 0.357406744336593] ##
## problem 2-b result: [0.65328148, 0.2705980,5 0.65328148, 0.27059805]                              ##
#######################################################################################################


# Problem 3

psi = sp.symbols('psi')
theta = sp.symbols('theta')
phi = sp.symbols('phi')

R = tf.rotz(psi) @ tf.roty(theta) @ tf.rotx(phi)
unit_vector = np.array([0, 0, 1])
Rotated = R @ unit_vector
Rotated = Rotated.reshape(3,1)
sp.pprint(Rotated)
print(R)


############################################################################
## Problem 3a result: [[cos(psi)*cos(theta), sin(phi)*sin(theta)*cos(psi)-sin(psi)*cos(phi),    sin(phi)*sin(psi)+sin(theta)*cos(phi)*cos(psi)]
##                     [sin(psi)*cos(theta), sin(phi)*sin(psi)*sin(theta) + cos(phi)*cos(psi), -sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi)]
##                     [-sin(theta),         sin(phi)*cos(theta),                               cos(phi)*cos(theta)]]
##
##
## Problem 3b result:                                                     
##                   [[sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi)]  
##                   [-sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi)]  
##                   [cos(phi)*cos(theta)]]                               
##
## Problem 3c result:
##                   I'm lost on this one.
##
##
############################################################################