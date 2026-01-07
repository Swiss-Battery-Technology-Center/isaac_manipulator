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

import os
from typing import List, Tuple

import isaac_ros_launch_utils.all_types as lut
import isaac_ros_launch_utils as lu

import isaac_manipulator_ros_python_utils.constants as constants
from isaac_manipulator_ros_python_utils.types import CameraType


def get_realsense_remappings(num_cameras: int, no_robot_mode: bool) -> List[Tuple[str, str]]:
    remappings = []
    for i in range(num_cameras):
        remappings.append((f'/camera_{i}/color/image', f'/camera_{i+1}/color/image_raw'))
        remappings.append((f'/camera_{i}/color/camera_info', f'/camera_{i+1}/color/camera_info'))
        remappings.append((f'/camera_{i}/depth/camera_info',
                           f'/camera_{i+1}/aligned_depth_to_color/camera_info'))
        # If we have no robot, nvblox consumes the depth image directly.
        # Otherwise the depth image published by the robot segmenter is used.
        if no_robot_mode:
            remappings.append(
                (f'/camera_{i}/depth/image', f'/camera_{i+1}/aligned_depth_to_color/image_raw'))
        else:
            remappings.append((f'/camera_{i}/depth/image', f'/cumotion/camera_{i+1}/world_depth'))
    return remappings


def get_hawk_remappings(no_robot_mode: bool) -> List[Tuple[str, str]]:
    remappings = []
    remappings.append(('/camera_0/color/image', '/rgb/image_rect_color'))
    remappings.append(('/camera_0/color/camera_info', '/rgb/camera_info'))
    # If we have no robot, nvblox consumes the depth image directly.
    # Otherwise the depth image published by the robot segmenter is used.
    if no_robot_mode:
        remappings.append(('/camera_0/depth/image', '/depth_image'))
    else:
        remappings.append(('/camera_0/depth/image', '/cumotion/camera_1/world_depth'))
    remappings.append(('/camera_0/depth/camera_info', '/rgb/camera_info'))
    return remappings


def get_sim_remappings() -> List[Tuple[str, str]]:
    remappings = []
    remappings.append(('/camera_0/color/image', '/front_stereo_camera/left/image_raw'))
    remappings.append(('/camera_0/color/camera_info', '/front_stereo_camera/left/camera_info'))
    # We want nvblox to consume the depth map that segments out the robot
    remappings.append(('/camera_0/depth/image', '/cumotion/camera_1/world_depth'))
    remappings.append(('/camera_0/depth/camera_info', '/front_stereo_camera/depth/camera_info'))
    return remappings


def add_nvblox(args: lu.ArgumentContainer) -> List[lut.Action]:
    camera_type = CameraType[args.camera_type]
    num_cameras = int(args.num_cameras)
    no_robot_mode = lu.is_true(args.no_robot_mode)
    workspace_bounds_name = str(args.workspace_bounds_name)
    actions = []

    # Check if the configuration is valid
    if camera_type is CameraType.hawk:
        assert num_cameras == 1, 'Running multiple hawk cameras not allowed.'
    elif camera_type is CameraType.realsense:
        assert num_cameras <= 2, 'Running more than 2 RealSense cameras not allowed.'
    elif camera_type is CameraType.isaac_sim:
        assert num_cameras == 1, 'Running multiple cameras in Isaac Sim not allowed.'

    # --- DEBUGGING START: Explicitly print every path ---
    print("\n" + "="*50)
    print("DEBUG: Starting Nvblox Configuration Load")
    print("="*50)

    # 1. Load Base Config
    base_config = None
    try:
        base_config = lu.get_path('nvblox_examples_bringup', 'config/nvblox/nvblox_base.yaml')
        print(f"DEBUG: Found base_config (examples): {base_config}")
    except:
        print("DEBUG: Could not find nvblox_examples_bringup.")

    if base_config is None:
        try:
            base_config = lu.get_path('isaac_ros_nvblox', 'config/nvblox/nvblox_base.yaml')
            print(f"DEBUG: Found base_config (core): {base_config}")
        except:
            print("DEBUG: Could not find isaac_ros_nvblox either.")

    # 2. Load Manipulator Config
    try:
        manipulator_base_config = lu.get_path('isaac_manipulator_bringup', 'config/nvblox/nvblox_manipulator_base.yaml')
        print(f"DEBUG: manipulator_base_config: {manipulator_base_config}")
    except:
        manipulator_base_config = None
        print("DEBUG: manipulator_base_config FAILED to load")

    # 3. Load Camera Configs
    hawk_config = None
    try:
        hawk_config = lu.get_path('isaac_manipulator_bringup', 'config/nvblox/specializations/nvblox_manipulator_hawk.yaml')
    except: pass
    
    realsense_config = None
    try:
        realsense_config = lu.get_path('isaac_manipulator_bringup', 'config/nvblox/specializations/nvblox_manipulator_realsense.yaml')
    except: pass
    
    isaac_sim_config = None
    try:
        isaac_sim_config = lu.get_path('isaac_manipulator_bringup', 'config/nvblox/specializations/nvblox_manipulator_sim.yaml')
    except: pass

    # 4. Load Workspace Config
    workspace_config = None
    try:
        workspace_config = lu.get_path('isaac_manipulator_bringup', f'config/nvblox/workspace_bounds/{workspace_bounds_name}.yaml')
        print(f"DEBUG: workspace_config: {workspace_config}")
    except:
        print("DEBUG: workspace_config FAILED to load")

    # Select the specific camera config
    camera_config = None
    if camera_type is CameraType.hawk:
        remappings = get_hawk_remappings(no_robot_mode)
        camera_config = hawk_config
    elif camera_type is CameraType.realsense:
        remappings = get_realsense_remappings(num_cameras, no_robot_mode)
        camera_config = realsense_config
    elif camera_type is CameraType.isaac_sim:
        remappings = get_sim_remappings()
        camera_config = isaac_sim_config
    else:
        raise Exception(f'CameraType {camera_type} not implemented.')
    
    print(f"DEBUG: Selected camera_config: {camera_config}")

    # --- CRITICAL CHECK ---
    # If any of these are None, the launch WILL crash with 'NoneType not iterable'
    if base_config is None:
        raise Exception("CRITICAL: base_config is None. Install 'nvblox_examples_bringup' or 'isaac_ros_nvblox'.")
    if manipulator_base_config is None:
        raise Exception("CRITICAL: manipulator_base_config is None.")
    if camera_config is None:
        raise Exception("CRITICAL: camera_config is None.")
    if workspace_config is None:
        raise Exception(f"CRITICAL: workspace_config is None for setup '{workspace_bounds_name}'.")

    # Load the workspace config validation
    if not os.path.exists(workspace_config):
        raise Exception(f'Workspace file path found but file does not exist: {workspace_config}')

    print("DEBUG: All configs loaded successfully.")
    print("="*50 + "\n")

    # Get all parameters with overrides.
    parameters = []
    parameters.append(base_config)
    parameters.append(manipulator_base_config)
    parameters.append(camera_config)
    parameters.append(workspace_config)
    parameters.append({'num_cameras': num_cameras})

    nvblox_node = lut.ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=parameters)

    actions.append(lu.load_composable_nodes(args.container_name, [nvblox_node]))
    actions.append(
        lu.log_info([
            "Enabling nvblox for '",
            str(num_cameras), "' '",
            str(camera_type), "' cameras configured for the '", workspace_bounds_name,
            "' workspace."
        ]))
    return actions


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('camera_type')
    args.add_arg('no_robot_mode', False)
    args.add_arg('num_cameras', 1)
    args.add_arg('workspace_bounds_name', '')
    args.add_arg('container_name', constants.MANIPULATOR_CONTAINER_NAME)

    args.add_opaque_function(add_nvblox)
    return lut.LaunchDescription(args.get_launch_actions())