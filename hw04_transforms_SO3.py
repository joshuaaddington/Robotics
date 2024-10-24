"""
SO(3) conversion code to convert between different SO(3) representations. 

Copy this file into your 'transforms.py' file at the bottom. 
"""

import numpy as np
from numpy import sin, cos, sqrt

def R2rpy(R):
    """
    rpy = R2rpy(R)
    Description:
    Returns the roll-pitch-yaw representation of the SO3 rotation matrix

    Parameters:
    R - 3 x 3 Numpy array for any rotation

    Returns:
    rpy - 1 x 3 Numpy Matrix, containing <roll pitch yaw> coordinates (in radians)
    """
    
    # follow formula in book, use functions like "np.atan2" 
    # for the arctangent and "**2" for squared terms. 
    # TODO - fill out this equation for rpy

    roll = np.atan2(R[2,1], R[1,1])
    pitch = np.atan2(-R[3,1], sqrt(R[3,2]**2 + R[3,3]**2))
    yaw = np.atan2(R[3,2], R[3,3])

    return np.array([roll, pitch, yaw])


def R2axis(R):
    """
    axis_angle = R2axis(R)
    Description:
    Returns an axis angle representation of a SO(3) rotation matrix

    Parameters:
    R

    Returns:
    axis_angle - 1 x 4 numpy matrix, containing  the axis angle representation
    in the form: <angle, rx, ry, rz>
    """

    # see equation (2.27) and (2.28) on pg. 54, using functions like "np.acos," "np.sin," etc. 
    ang = np.arccos((R[1,1]+R[2,2]+R[3,3]-1)/2)
    axis_angle = np.array([ang,
                            1/(2*np.sin(ang)) * (R[3,2]-R[2,3]),
                            1/(2*np.sin(ang)) * (R[1,3]-R[3,1]),
                            1/(2*np.sin(ang)) * (R[2,1]-R[1,2])])

    return axis_angle

def axis2R(ang, v):
    """
    R = axis2R(angle, rx, ry, rz, radians=True)
    Description:
    Returns an SO3 object of the rotation specified by the axis-angle

    Parameters:
    angle - float, the angle to rotate about the axis in radians
    v = [rx, ry, rz] - components of the unit axis about which to rotate as 3x1 numpy array
    
    Returns:
    R - 3x3 numpy array
    """
    # TODO fill this out 
    R = np.array([[v[0]**2 * (1-cos(ang)) + cos(ang), v[0]*v[1]*(1-cos(ang)) - v[2]*sin(ang), v[0]*v[2]*(1-cos(ang)) + v[1]*sin(ang)],
                  [v[0]*v[1]*(1-cos(ang)) + v[2]*sin(ang), v[1]**2 * (1-cos(ang)) + cos(ang), v[1]*v[2]*(1-cos(ang)) + v[0]*sin(ang)],
                  [v[0]*v[2]*(1-cos(ang)) - v[1]*sin(ang), v[1]*v[2]*(1-cos(ang)) + v[0]*sin(ang), v[2]**2 * (1-cos(ang)) + cos(ang)]])
    return R

def R2quatuat(R):
    """
    quaternion = R2quat(R)
    Description:
    Returns a quaternion representation of pose

    Parameters:
    R

    Returns:
    quaternion - 1 x 4 numpy matrix, quaternion representation of pose in the 
    format [nu, ex, ey, ez]
    """
    # TODO, see equation (2.34) and (2.35) on pg. 55, using functions like "sp.sqrt," and "sp.sign"

    return np.array([np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2,
                     (np.sign(R[2,1] - R[1,2]) * np.sqrt(1 + R[0,0] - R[1,1] - R[2,2])) / 2,
                     (np.sign(R[1,0] - R[0,1]) * np.sqrt(1 + R[1,1] - R[2,2] - R[0,0])) / 2,
                     (np.sign(R[1,0] - R[0,1]) * np.sqrt(1 + R[2,2] - R[0,0] - R[1,1])) / 2])
                    
def quat2R(q):
    """
    R = quat2R(q)
    Description:
    Returns a 3x3 rotation matrix

    Parameters:
    q - 4x1 numpy array, [nu, ex, ey, ez ] - defining the quaternion
    
    Returns:
    R - a 3x3 numpy array 
    """
    # TODO, extract the entries of q below, and then calculate R
    nu = q[0]
    ex = q[1]
    ey = q[2]
    ez = q[3]
    R =  np.array([[2*(nu**2 + ex**2) - 1, 2*(ex*ey - nu*ez), 2*(ex*ez + nu*ey)]
                   [2*(ex*ey + nu*ez), 2*(nu**2 + ey**2) - 1, 2*(ey*ez - nu*ex)]
                   [2*(ex*ez - nu*ey), 2*(ey*ez + nu*ex), 2*(nu**2 + ez**2) - 1]])
    return R


def euler2R(th1, th2, th3, order='xyz'):
    """
    R = euler2R(th1, th2, th3, order='xyz')
    Description:
    Returns a 3x3 rotation matrix as specified by the euler angles, we assume in all cases
    that these are defined about the "current axis," which is why there are only 12 versions 
    (instead of the 24 possiblities noted in the course slides). 

    Parameters:
    th1, th2, th3 - float, angles of rotation
    order - string, specifies the euler rotation to use, for example 'xyx', 'zyz', etc.
    
    Returns:
    R - 3x3 numpy matrix
    """

    # TODO - fill out each expression for R based on the condition 
    # (hint: use your rotx, roty, rotz functions)
    if order == 'xyx':
        R = rotx(th1) @ roty(th2) @ rotx(th3)
    elif order == 'xyz':
        R = rotx(th1) @ roty(th2) @ rotz(th3)
    elif order == 'xzx':
        R = rotx(th1) @ rotz(th2) @ rotx(th3)
    elif order == 'xzy':
        R = rotx(th1) @ rotz(th2) @ roty(th3)
    elif order == 'yxy':
        R = roty(th1) @ rotx(th2) @ roty(th3)
    elif order == 'yxz':
        R = roty(th1) @ rotx(th2) @ rotz(th3)
    elif order == 'yzx':
        R = roty(th1) @ rotz(th2) @ rotx(th3)
    elif order == 'yzy':
        R = roty(th1) @ rotz(th2) @ roty(th3)
    elif order == 'zxy':
        R = rotz(th1) @ rotx(th2) @ roty(th3)
    elif order == 'zxz':
        R = rotz(th1) @ rotx(th2) @ rotz(th3)
    elif order == 'zyx':
        R = rotz(th1) @ roty(th2) @ rotx(th3)
    elif order == 'zyz':
        R = rotz(th1) @ roty(th2) @ rotz(th3)
    else:
        print("Invalid Order!")
        return

    return R
