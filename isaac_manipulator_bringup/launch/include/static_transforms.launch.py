# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

from typing import List

from isaac_ros_launch_utils.all_types import Action, LaunchDescription
import isaac_ros_launch_utils as lu

from isaac_manipulator_ros_python_utils.types import CameraType, TrackingType

# Dictionary containing the calibration of various camera setups.
calibrations_dict = {
    'test': {
        'world_to_base_link': {
            'parent_frame': 'world',
            'child_frame': 'base_link',
            'translation': [0.0, 0.0, 0.0],
            'rotation': [0.0, 0.0, 0.0, 1.0],  # [qx, qy ,qz, qw]
        },
        'world_pose_realsense_camera_intermediate': {
            'parent_frame': 'world',
            'child_frame': 'camera_intermediate',
            'translation': [0.7723, 0.4918, 0.9027],
            'rotation': [0.3511, 0.8570, -0.3616, -0.1073],  # [qx, qy, qz, qw]
        },
        'world_pose_realsense_camera_intermediate1': {
            'parent_frame': 'camera_intermediate',
            'child_frame': 'camera_intermediate1',
            'translation': [0.0, 0.0, 0.0],
            'rotation': [0.0, 0.0, 0.707168, 0.707168],  # [qx, qy, qz, qw]
        },
        'world_pose_realsense_camera_intermediate2': {
            'parent_frame': 'camera_intermediate1',
            'child_frame': 'camera_intermediate2',
            'translation': [0.0, 0.0, 0.0],
            'rotation': [0.707168, 0.0, 0.707168, 0.0],  # [qx, qy, qz, qw]
        },
        'world_to_realsense_1': {
            'parent_frame': 'camera_intermediate2',
            'child_frame': 'camera_1_link',
            'translation': [0.0, 0.0, 0.0],
            'rotation': [1.0, 0.0, 0.0, 0.0],  # [qx, qy ,qz, qw]
        },
        'world_to_target_frame_1': {
            'parent_frame': 'world',
            'child_frame': 'target1_frame',
            'translation': [0.3, 0.3, 0.15],
            'rotation': [1.0, 0.0, 0.0, 0.0],  # [qx, qy ,qz, qw]
        },
        'world_to_target_frame_2': {
            'parent_frame': 'world',
            'child_frame': 'target2_frame',
            'translation': [0.3, -0.3, 0.15],
            'rotation': [1.0, 0.0, 0.0, 0.0],  # [qx, qy ,qz, qw]
        },
    },
    'home': {
        'world_to_base_link': {
            'parent_frame': 'world',
            'child_frame': 'base_link',
            'translation': [0.0, 0.0, 0.0],
            'rotation': [0.0, 0.0, 0.0, 1.0],  # [qx, qy ,qz, qw]
        },
        'home': {
            'parent_frame': 'world',
            'child_frame': 'home_frame',
            'translation': [0.2799, 0.23077, 0.3283],
            'rotation': [0.8703, 0.43052, 0.049201, 0.23412],  # [qx, qy ,qz, qw]
        },
    },
}


def static_transform_from_dict(transform_dict):
    return lu.static_transform(
        parent=transform_dict['parent_frame'],
        child=transform_dict['child_frame'],
        translation=transform_dict['translation'],
        orientation_quaternion=transform_dict['rotation']
    )


def add_static_transforms(args: lu.ArgumentContainer) -> List[Action]:
    """
    If `camera_type` is empty (''), skip camera transforms entirely.
    Otherwise, if camera_type is hawk or realsense, load the respective transforms.
    """

    # Access the arguments
    camera_type_str = args.camera_type  # might be empty
    tracking_type_str = args.tracking_type
    num_cameras = int(args.num_cameras)
    broadcast_world_base_link = bool(args.broadcast_world_base_link)

    calibration_name = args.calibration_name
    if calibration_name not in calibrations_dict:
        return [
            lu.log_info([
                f"Calibration '{calibration_name}' not found. Not loading static transforms."
            ])
        ]

    transforms = calibrations_dict[calibration_name]
    actions: List[Action] = []

    # If broadcast_world_base_link, publish the base_link transform
    if broadcast_world_base_link and 'world_to_base_link' in transforms:
        actions.append(static_transform_from_dict(transforms['world_to_base_link']))

    # Optional: If you also want to always broadcast other frames (like 'home'), do that:
    if 'home' in transforms:
        actions.append(static_transform_from_dict(transforms['home']))

    # If camera_type is empty or not recognized => skip camera transforms
    if camera_type_str:
        # Convert string to enum
        camera_type_enum = CameraType[camera_type_str]

        # If camera is hawk
        if camera_type_enum is CameraType.hawk:
            # Ensure the transforms are in the dictionary or skip
            if 'world_to_hawk' in transforms:
                actions.append(static_transform_from_dict(transforms['world_to_hawk']))
            else:
                actions.append(lu.log_info(["No hawk transforms found in dict. Skipping."]))

        # If camera is realsense
        elif camera_type_enum is CameraType.realsense:
            # Attempt to load realsense transforms if they exist
            realsense_keys = [
                'world_pose_realsense_camera_intermediate',
                'world_pose_realsense_camera_intermediate1',
                'world_pose_realsense_camera_intermediate2',
                'world_to_realsense_1',
                'world_to_target_frame_1',
                'world_to_target_frame_2'
            ]
            for key in realsense_keys:
                if key in transforms:
                    actions.append(static_transform_from_dict(transforms[key]))
                else:
                    actions.append(lu.log_info([f"Transform '{key}' not found. Skipping."]))

            # If 2 cameras, also load 'world_to_realsense_2' if present
            if num_cameras > 1:
                if 'world_to_realsense_2' in transforms:
                    actions.append(static_transform_from_dict(transforms['world_to_realsense_2']))
                else:
                    actions.append(lu.log_info(["'world_to_realsense_2' not found. Skipping."]))
                assert num_cameras <= 2, 'Running more than 2 cameras not allowed.'
        else:
            # If for some reason there's another camera type we haven't covered
            actions.append(lu.log_info([
                f"CameraType {camera_type_enum} not recognized in static transforms. Skipping."
            ]))

    # Always log success
    actions.append(
        lu.log_info([
            f"Successfully loaded the static transforms for calibration '{calibration_name}'."
        ])
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    # Here, we default camera_type to '' so if user doesn't specify it, no camera transforms are loaded
    args = lu.ArgumentContainer()
    args.add_arg('num_cameras', 1)
    args.add_arg('broadcast_world_base_link', False)
    args.add_arg('camera_type', '')       # Default is empty => no camera transforms
    args.add_arg('tracking_type', 'pose_to_pose')
    args.add_arg('calibration_name', '')

    # Use an opaque function to build the transforms
    args.add_opaque_function(add_static_transforms)
    return LaunchDescription(args.get_launch_actions())
