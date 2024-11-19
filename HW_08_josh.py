import numpy as np
import sympy as sp
import kinematics as kin
import dynamics as dyn
from visualization import VizScene 
import transforms as tr
from extractor import translation, rotation, norm


# Problem 1:
# For a rectangular prism with mass m, and origin at the center of the prism:
#
# Ix = 1/12 * m * (b^2 + c^2)
# Iy = 1/12 * m * (a^2 + c^2)
# Iz = 1/12 * m * (a^2 + b^2)
#
# Ixy = Iyz = Ixz = 0
# 
# For a prism with lengths, a, b, and c, and mass m with origin at the corner:
#
# Ixx = 1/3 * m * (b^2 + c^2)
# Iyy = 1/3 * m * (a^2 + c^2)
# Izz = 1/3 * m * (a^2 + b^2)
#
# All off-diagonal terms = m/4 * a * b * c
#
# Problem 2:

dh = [[0, 0, .4, 0],
      [0, 0, .4, 0],
      [0, 0, .4, 0]]

joint_type = ['r', 'r', 'r']

link_masses = [1, 1, 1]

# defining three different centers of mass, one for each link
r_coms = [np.array([-0.2, 0, 0]), np.array([-0.2, 0, 0]), np.array([-0.2, 0, 0])]

link_inertias = [[1/12 * 1 * (.4**2 + .4**2), 0, 0], 
                 [1/12 * 1 * (.4**2 + .4**2), 0, 0], 
                 [1/12 * 1 * (.4**2 + .4**2), 0, 0]]

arm = dyn.SerialArmDyn(dh,
                        jt=joint_type,
                        mass=link_masses,
                        r_com=r_coms,
                        link_inertia=link_inertias)

q = [np.pi/4.0]*3
qd = [.2]*3
qdd = [.1]*3
arm.rne(q, qd, qdd, g = np.array([0, 0, -9.81]))

