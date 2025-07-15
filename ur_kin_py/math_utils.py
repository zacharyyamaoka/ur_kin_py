import numpy as np
from transforms3d.euler import euler2mat

def xyzrpy_to_matrix(xyz, rpy):
    mat = np.eye(4)
    mat[:3, :3] = euler2mat(*rpy)  # rotation
    mat[:3, 3] = xyz               # translation
    return mat