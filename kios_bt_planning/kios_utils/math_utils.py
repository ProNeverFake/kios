import numpy as np
from spatialmath import *

# ! NOT IN USE I GUESS


def HT_inverse(HT: np.ndarray) -> np.ndarray:
    """
    Inverse of a homogeneous transformation matrix
    """
    R = HT[0:3, 0:3]
    t = HT[0:3, 3]
    HT_inv = np.zeros((4, 4))
    HT_inv[0:3, 0:3] = R.transpose()
    HT_inv[0:3, 3] = -R.transpose().dot(t)
    HT_inv[3, 3] = 1
    return HT_inv


def R_from_rpy(vec: np.ndarray) -> np.ndarray:
    """
    Rotation matrix from a 3D rpy vector
    """
    R = SE3.RPY(vec, unit="deg", order="xyz")
    return R


def HT_from_xyzrpy(vec: np.ndarray) -> np.ndarray:
    """
    Homogeneous transformation matrix from a 6D xyzrpy vector
    """
    HT = SE3(vec[0:3]) * SE3.RPY(vec[3:6], unit="deg", order="xyz")
    return HT


# not tested
def generate_HT(HT: np.ndarray, xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    """
    Generate a homogeneous transformation matrix from a 6D vector
    """
    HT_new = HT * SE3(xyz) * SE3.RPY(rpy, order="xyz")
    return HT_new
