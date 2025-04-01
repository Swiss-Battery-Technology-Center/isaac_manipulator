#!/usr/bin/env python3

from typing import List
import os
import math
import numpy as np

import isaac_ros_launch_utils as lu
from isaac_ros_launch_utils.all_types import Action, LaunchDescription

##############################################################################
# 1) Hard-coded path to best_grasp_in_base.npy
##############################################################################
BEST_GRASP_NPY_PATH = '/workspaces/cumotion/isaac_manipulator/isaac_manipulator_bringup/grasp/best_grasp_in_base.npy'

##############################################################################
# 2) Robot-only dictionary transforms (no cameras)
##############################################################################
calibrations_dict = {
    'home': {
        'world_to_base_link': {
            'parent_frame': 'world',
            'child_frame': 'base_link',
            'translation': [0.0, 0.0, 0.0],
            'rotation': [0.0, 0.0, 0.0, 1.0],
        },
        'home': {
            'parent_frame': 'world',
            'child_frame': 'home_frame',
            'translation': [0.2799, 0.23077, 0.3283],
            'rotation': [0.8703, 0.43052, 0.049201, 0.23412],
        },
        'camera': {
            'parent_frame': 'end_effector_link',
            'child_frame': 'camera_link',
            'translation': [0.0154, 0.0862, -0.0544],
            'rotation': [0.0210, -0.0461, 0.9979, 0.0399],
        },
    },
    'test': {
        'world_to_base_link': {
            'parent_frame': 'world',
            'child_frame': 'base_link',
            'translation': [0.0, 0.0, 0.0],
            'rotation': [0.0, 0.0, 0.0, 1.0],
        },
        'world_to_target_frame_1': {
            'parent_frame': 'world',
            'child_frame': 'target1_frame',
            'translation': [0.3, 0.3, 0.15],
            'rotation': [1.0, 0.0, 0.0, 0.0],
        },
        'world_to_target_frame_2': {
            'parent_frame': 'world',
            'child_frame': 'target2_frame',
            'translation': [0.3, -0.3, 0.15],
            'rotation': [1.0, 0.0, 0.0, 0.0],
        },
    },
}

##############################################################################
# 3) Utility: create static transform from a dict
##############################################################################
def static_transform_from_dict(transform_dict):
    return lu.static_transform(
        parent=transform_dict['parent_frame'],
        child=transform_dict['child_frame'],
        translation=transform_dict['translation'],
        orientation_quaternion=transform_dict['rotation']
    )

##############################################################################
# 4) Utility to convert 3x3 to quaternion
##############################################################################
def matrix_to_quaternion(R: np.ndarray):
    """Convert a 3x3 rotation matrix to [qx, qy, qz, qw]."""
    qw = 0.5 * math.sqrt(max(0, 1 + R[0,0] + R[1,1] + R[2,2]))
    qx = 0.5 * math.sqrt(max(0, 1 + R[0,0] - R[1,1] - R[2,2]))
    qy = 0.5 * math.sqrt(max(0, 1 - R[0,0] + R[1,1] - R[2,2]))
    qz = 0.5 * math.sqrt(max(0, 1 - R[0,0] - R[1,1] + R[2,2]))

    # Determine sign of qx,qy,qz from off-diagonal elements:
    if (R[2,1] - R[1,2]) < 0: qx = -qx
    if (R[0,2] - R[2,0]) < 0: qy = -qy
    if (R[1,0] - R[0,1]) < 0: qz = -qz

    return [qx, qy, qz, qw]

##############################################################################
# 5) Load standard transforms from dictionary
##############################################################################
def add_robot_transforms() -> List[Action]:
    """
    Always load the 'home' calibration from calibrations_dict.
    (Or 'test', or both. Adjust as you wish.)
    """
    actions: List[Action] = []

    # Let's assume we always load 'home' transforms:
    if 'home' not in calibrations_dict:
        actions.append(lu.log_info(["No 'home' calibration found, skipping."]))
        return actions

    transforms = calibrations_dict['home']

    if 'camera' in transforms:
        actions.append(static_transform_from_dict(transforms['camera']))

    # broadcast world->base_link
    if 'world_to_base_link' in transforms:
        actions.append(static_transform_from_dict(transforms['world_to_base_link']))

    # broadcast home (world->home_frame)
    if 'home' in transforms:
        actions.append(static_transform_from_dict(transforms['home']))

    actions.append(
        lu.log_info(["Loaded the 'home' dictionary transforms (world->base_link, home_frame)."])
    )
    return actions

##############################################################################
# 6) Also load the best_grasp_in_base.npy if it exists, broadcast base->best_grasp_frame
##############################################################################
def add_best_grasp_transform() -> List[Action]:
    actions: List[Action] = []

    if os.path.exists(BEST_GRASP_NPY_PATH):
        T_base_grasp = np.load(BEST_GRASP_NPY_PATH)
        if T_base_grasp.shape == (4,4):
            # Extract translation
            tx, ty, tz = T_base_grasp[0:3, 3]
            # Extract rotation
            R = T_base_grasp[:3, :3]
            q = matrix_to_quaternion(R)

            actions.append(
                lu.static_transform(
                    parent='camera_link',     # or 'world', if you prefer
                    child='grasp_frame',
                    translation=[float(tx), float(ty), float(tz)],
                    orientation_quaternion=[float(q[0]), float(q[1]), float(q[2]), float(q[3])]
                )
            )
            actions.append(
                lu.log_info([
                    f"Loaded best_grasp_in_base from {BEST_GRASP_NPY_PATH} "
                    "and broadcasting base_link->grasp_frame"
                ])
            )
        else:
            actions.append(lu.log_info([
                f"Warning: {BEST_GRASP_NPY_PATH} not a 4x4. shape={T_base_grasp.shape}"
            ]))
    else:
        actions.append(lu.log_info([
            f"No file found at '{BEST_GRASP_NPY_PATH}' => skipping grasp_frame."
        ]))

    return actions

##############################################################################
# 7) Put it all together
##############################################################################
def generate_launch_description() -> LaunchDescription:
    actions: List[Action] = []

    # 1) Load the dictionary transforms for "home"
    actions.extend(add_robot_transforms())

    # 2) Load the best_grasp_in_base.npy if present
    actions.extend(add_best_grasp_transform())

    return LaunchDescription(actions)
