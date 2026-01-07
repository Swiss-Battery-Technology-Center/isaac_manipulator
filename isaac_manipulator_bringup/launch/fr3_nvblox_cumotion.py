# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

import isaac_ros_launch_utils.all_types as lut
import isaac_ros_launch_utils as lu
import isaac_manipulator_ros_python_utils.constants as constants

def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('log_level', 'info')
    actions = args.get_launch_actions()

    # 1. Static Transforms (Position)
    # This places the camera at your 0.92m / 0.25m coordinates
    actions.append(
        lu.include(
            'isaac_manipulator_bringup',
            'launch/include/static_transforms_fr3.launch.py',
            launch_arguments={
                'calibration_name': 'test',  # Force 'test' setup
                'camera_type': 'realsense' 
            },
        ))

    # 2. RealSense Driver (Hardware)
    # Uses the 'realsense.launch.py' you modified with the HARDCODED Serial Number
    actions.append(
        lu.include(
            'isaac_manipulator_bringup',
            'launch/include/realsense.launch.py',
            launch_arguments={
                'num_cameras': '1',
                'camera_ids_config_name': 'test'
            },
        ))

    # 3. Nvblox (3D Mapping)
    # Uses the 'nvblox.launch.py' you modified with the HARDCODED config
    actions.append(
        lu.include(
            'isaac_manipulator_bringup',
            'launch/include/nvblox.launch.py',
            launch_arguments={
                'camera_type': 'realsense',
                'num_cameras': '1',
                'workspace_bounds_name': 'test'
            },
            delay='2.0' # Wait for camera to warm up
        ))

    # 4. CuMotion (Planner)
    # We launch this manually to ensure it doesn't look for bad configs
    actions.append(
        lu.include(
            'isaac_manipulator_bringup',
            'launch/include/cumotion.launch.py',
            launch_arguments={
                'camera_type': 'realsense',
                'num_cameras': '1',
                'no_robot_mode': 'False', 
                'workspace_bounds_name': 'test',
            },
            delay='5.0' # Wait for Nvblox to be ready
        ))

    # 5. Container (Required)
    actions.append(
        lu.component_container(constants.MANIPULATOR_CONTAINER_NAME, log_level=args.log_level))
    
    return lut.LaunchDescription(actions)