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
            'translation': [0.13808, 0.004505, 0.51433],
            'rotation': [0.70335, 0.70181, 0.089203, 0.069305],
        },
        'camera': {
            'parent_frame': 'end_effector_link',
            'child_frame': 'camera_link',
            'translation': [0.0348, 0.0460, 0.0651],
            'rotation': [-0.0051, 0.0003, 0.9993, -0.0375],
        },
        'post_grasp_lift': {
            'parent_frame': 'base_link', # Define relative to base_link
            'child_frame': 'post_grasp_lift_frame',
             # USE THE FIRST IMAGE'S VALUES (Position, Orientation xyzw)
            'translation': [0.27914, 0.14424, 0.45736],
            'rotation': [0.60482, 0.52133, 0.47815, 0.36576], # x, y, z, w
        },
        'drop_off': {
            'parent_frame': 'base_link', # Define relative to base_link
            'child_frame': 'drop_off_frame',
             # USE THE SECOND IMAGE'S VALUES (Position, Orientation xyzw)
            'translation': [0.32895, 0.52452, 0.41249],
            'rotation': [0.6815, 0.62578, 0.17843, 0.33486], # x, y, z, w
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
    # Check if required keys exist
    if not all(k in transform_dict for k in ['parent_frame', 'child_frame', 'translation', 'rotation']):
        lu.log_warn(f"Skipping transform definition due to missing keys: {transform_dict}")
        return None

    return lu.static_transform(
        parent=transform_dict['parent_frame'],
        child=transform_dict['child_frame'],
        translation=transform_dict['translation'],
        orientation_quaternion=transform_dict['rotation']
    )


##############################################################################
# 5) Load standard transforms from dictionary
##############################################################################
def add_robot_transforms() -> List[Action]:
    """
    Always load the 'home' calibration from calibrations_dict.
    (Or 'test', or both. Adjust as you wish.)
    """
    actions: List[Action] = []
    calibration_name = 'home' # Or make this configurable if needed

    if calibration_name not in calibrations_dict:
        actions.append(lu.log_info([f"No '{calibration_name}' calibration found, skipping."]))
        return actions

    transforms = calibrations_dict[calibration_name]
    loaded_transforms = []

    # List of transforms to load for the 'home' setup
    transform_keys = ['world_to_base_link', 'home', 'post_grasp_lift', 'drop_off', 'camera']
    # Optional: Add 'camera' back if needed: transform_keys.append('camera')


    for key in transform_keys:
        if key in transforms:
            tf_action = static_transform_from_dict(transforms[key])
            if tf_action:
                actions.append(tf_action)
                loaded_transforms.append(transforms[key]['child_frame'])
        else:
            actions.append(lu.log_warn([f"Transform key '{key}' not found in '{calibration_name}' calibration."]))


    actions.append(
        lu.log_info([f"Loaded '{calibration_name}' dictionary transforms for frames: {loaded_transforms}."])
    )
    return actions


##############################################################################
# 7) Put it all together
##############################################################################
def generate_launch_description() -> LaunchDescription:
    actions: List[Action] = []

    # 1) Load the dictionary transforms for "home"
    actions.extend(add_robot_transforms())


    return LaunchDescription(actions)
