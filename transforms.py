"""
Transforms Module - Contains code for to learn about rotations
and eventually homogenous transforms. 

Empty outline derived from code written by John Morrell, former TA. 
"""

import numpy as np
from numpy import sin, cos, sqrt
from numpy.linalg import norm

## 2D Rotations
def rot2(th):
    """
    R = rot2(theta)
    Parameters
        theta: float or int, angle of rotation
    Returns
        R: 2 x 2 numpy array representing rotation in 2D by theta
    """

    ## TODO - Fill this out
    R = np.array([[cos(th), -sin(th)], [sin(th), cos(th)]]);
    return R

## 3D Transformations
def rotx(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about x-axis by amount theta
    """
    ## TODO - Fill this out
    R = np.array([[1, 0, 0], [0, cos(th), -sin(th)], [0, sin(th), cos(th)]])

    return R

def roty(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about y-axis by amount theta
    """
    ## TODO - Fill this out
    R = np.array([[cos(th), 0, sin(th)], [0, 1, 0], [-sin(th), 0, cos(th)]])

    return R

def rotz(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about z-axis by amount theta
    """

    ## TODO - Fill this out
    R = np.array([[cos(th), -sin(th), 0], [sin(th), cos(th), 0], [0, 0, 1]])

    return R

# inverse of rotation matrix
def rot_inv(R):
    '''
    R = rot_inv(R)
    Parameters
        R: 2x2 or 3x3 numpy array representing a proper rotation matrix
    Returns
        R: 2x2 or 3x3 inverse of the input rotation matrix
    '''
    ## TODO - Fill this out
    return np.transpose(R)
