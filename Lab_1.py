import numpy as np
import sympy as sp
from visualization import VizScene 
import transforms as tr
import kinematics as kin
np.set_printoptions(precision=4, suppress=True)

# Set DH Paramaters
dh = np.array([[0, .27035, -.069, np.pi/2],
                [np.pi/2, 0, 0, -np.pi/2],
                [0, .36435, -.069, np.pi/2],
                [0, 0, 0, -np.pi/2],
                [0, .37429, -.010, np.pi/2],
                [0, 0, 0, -np.pi/2],
                [0, .229525, 0, 0]])

# Create a visualization scene
viz = VizScene()
