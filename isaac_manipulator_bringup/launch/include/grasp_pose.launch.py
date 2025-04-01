import numpy as np
import os

###############################################################################
# 1) base->camera_top transform as a 4x4 matrix
###############################################################################
pos_base_cam = np.array([-0.0570,  0.1052,  0.7462])
quat_base_cam = np.array([0.3981,  0.0859,  0.0224,  0.9130])  # (qx, qy, qz, qw)

def quaternion_to_matrix(q):
    """Convert [qx, qy, qz, qw] to a 3x3 rotation matrix."""
    x, y, z, w = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z

    return np.array([
        [1 - 2*(yy + zz), 2*(xy - wz),     2*(xz + wy)],
        [2*(xy + wz),     1 - 2*(xx + zz), 2*(yz - wx)],
        [2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy)]
    ])

def make_4x4(rot3x3, trans3):
    """Build a 4x4 homogeneous transform from a 3x3 rotation and 3D translation."""
    T = np.eye(4)
    T[:3, :3] = rot3x3
    T[:3, 3] = trans3
    return T

R_base_cam = quaternion_to_matrix(quat_base_cam)
T_base_cam = make_4x4(R_base_cam, pos_base_cam)

###############################################################################
# 2) LOAD your "camera -> grasp" pose from a .npy file (3x3 rotation + 3D translation)
###############################################################################
PATH_TO_GRASP_NPY = '/workspaces/cumotion/isaac_manipulator/isaac_manipulator_bringup/best_grasp.npy'  # Change to your actual file path
data = np.load(PATH_TO_GRASP_NPY, allow_pickle=True)

# Suppose "data" is a dictionary or a structure you saved. 
# If the array is literally just a dict with 'rotation' and 'translation', you might do:
R_cam_grasp = data.item().get('rotation')       # shape (3, 3)
t_cam_grasp = data.item().get('translation')    # shape (3,)
# If the data is stored differently, adjust these lines accordingly.

T_cam_grasp = make_4x4(R_cam_grasp, t_cam_grasp)

###############################################################################
# 3) Multiply to get base->grasp
###############################################################################
T_base_grasp = T_base_cam @ T_cam_grasp

###############################################################################
# 4) Extract final rotation + translation if desired
###############################################################################
R_base_grasp = T_base_grasp[:3, :3]
t_base_grasp = T_base_grasp[:3, 3]

print("T_base_grasp:\n", T_base_grasp)
print("R_base_grasp:\n", R_base_grasp)
print("t_base_grasp:\n", t_base_grasp)
