import numpy as np
import sympy as sp

def translation (array: np.array):
    """
    Takes a transform matrix and extracts the translation vector
    """
    return array[:3, 3]

def rotation (array: np.array):
    """
    Takes a transform matrix and extracts the rotation matrix
    """
    return array[:3, :3]

def norm (vector: np.array):
    """
    Normalizes a vector
    """
    return vector / np.linalg.norm(vector)

def z_axis (array: np.array):
    """
    Extracts the z-axis of a rotation matrix
    """
    return array[:3, 2]