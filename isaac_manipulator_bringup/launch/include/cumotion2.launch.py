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

from ament_index_python.packages import get_package_share_directory

from isaac_manipulator_ros_python_utils.types import CameraType
import isaac_ros_launch_utils as lu
from isaac_ros_launch_utils.all_types import Action, Node, LaunchDescription


def get_hawk_depth_topics() -> Tuple[str, str]:
    depth_image_topics = '["/depth_image"]'
    depth_camera_infos = '["/rgb/camera_info"]'
    return depth_image_topics, depth_camera_infos


def get_realsense_depth_topics(num_cameras: int) -> Tuple[str, str]:
    depth_image_topics = []
    depth_camera_infos = []
    for i in range(num_cameras):
        depth_image_topics.append(f'/camera_{i+1}/aligned_depth_to_color/image_raw')
        depth_camera_infos.append(f'/camera_{i+1}/aligned_depth_to_color/camera_info')
    return depth_image_topics, depth_camera_infos


def get_isaac_sim_depth_topics() -> Tuple[str, str]:
    depth_image_topics: str = '["/front_stereo_camera/depth/ground_truth"]'
    depth_camera_infos: str = '["/front_stereo_camera/depth/camera_info"]'
    return depth_image_topics, depth_camera_infos


def add_cumotion(args: lu.ArgumentContainer) -> List[Action]:
    camera_type = CameraType[args.camera_type]
    num_cameras = int(args.num_cameras)
    from_bag = lu.is_true(args.from_bag)
    no_robot_mode = lu.is_true(args.no_robot_mode)
    enable_object_attachment = lu.is_true(args.enable_object_attachment)
    workspace_bounds_name = str(args.workspace_bounds_name)
    actions: List[Action] = []

    # depth topics setup omitted for brevity…

    # Get the workspace.
    workspace_file_path = lu.get_path(
        'isaac_manipulator_bringup',
        f'config/nvblox/workspace_bounds/{workspace_bounds_name}.yaml'
    )
    if not os.path.exists(workspace_file_path):
        raise RuntimeError(f"Workspace '{workspace_bounds_name}' not found.")
    actions.append(lu.log_info([
        "Loading the '", workspace_bounds_name,
        "' workspace. Ignoring the grid_center_m / grid_size_m parameters."
    ]))

    # cuMotion launch
    if not from_bag and not no_robot_mode:
        actions.append(
            lu.include(
                'isaac_ros_cumotion',
                'launch/isaac_ros_cumotion.launch.py',
                launch_arguments={
                    'cumotion_planner.robot':                     args.robot_file_name,
                    'cumotion_planner.workspace_file_path':       workspace_file_path,
                    'cumotion_planner.grid_size_m':               '[2.0, 2.0, 2.0]',
                    'cumotion_planner.grid_center_m':             '[0.0, 0.0, 0.0]',
                    'cumotion_planner.time_dilation_factor':      args.time_dilation_factor,
                    'cumotion_planner.tool_frame':                args.tool_frame,
                    'cumotion_planner.base_link':                 args.base_link,
                    'cumotion_planner.read_esdf_world':           args.read_esdf_world,
                    'cumotion_planner.publish_curobo_world_as_voxels':
                                                                    args.publish_curobo_world_as_voxels,
                    'cumotion_planner.override_moveit_scaling_factors': 'True',
                    'cumotion_planner.joint_states_topic':        args.joint_states_topic,
                    'cumotion_planner.voxel_size':                '0.04',
                    'cumotion_planner.publish_voxel_size':        '0.04',
                    'cumotion_planner.update_link_sphere_server':
                                                                    args.update_link_sphere_server_planner,
                    'cumotion_planner.urdf_path':                 args.urdf_file_path,
                },
            )
        )
    else:
        # fallback to ESDF visualizer…
        actions.append(
            Node(
                package='isaac_ros_esdf_visualizer',
                executable='esdf_visualizer',
                name='esdf_visualizer',
                parameters=[{
                    'workspace_file_path': workspace_file_path,
                    'esdf_service_call_period_secs': 0.05,
                }],
                output='screen',
            )
        )

    # robot_segmenter launch
    if not no_robot_mode:
        actions.append(
            lu.include(
                'isaac_ros_cumotion',
                'launch/robot_segmentation.launch.py',
                launch_arguments={
                    'robot_segmenter.robot':                     args.robot_file_name,
                    'robot_segmenter.urdf_path':                 args.urdf_file_path,
                    'robot_segmenter.depth_qos':                 args.qos_setting,
                    'robot_segmenter.depth_info_qos':            args.qos_setting,
                    'robot_segmenter.mask_qos':                  args.qos_setting,
                    'robot_segmenter.world_depth_qos':           args.qos_setting,
                    'robot_segmenter.depth_image_topics':        depth_image_topics,
                    'robot_segmenter.depth_camera_infos':        depth_camera_infos,
                    'robot_segmenter.robot_mask_publish_topics': robot_mask_publish_topics,
                    'robot_segmenter.world_depth_publish_topics':world_depth_publish_topics,
                    'robot_segmenter.filter_speckles_in_mask':   filter_speckles_in_robot_mask,
                    'robot_segmenter.max_filtered_speckles_size':max_filtered_speckles_size,
                    'robot_segmenter.distance_threshold':        args.distance_threshold,
                    'robot_segmenter.time_sync_slop':            args.time_sync_slop,
                    'robot_segmenter.joint_states_topic':        args.joint_states_topic,
                    'robot_segmenter.tool_frame':                args.tool_frame,
                    'robot_segmenter.base_link':                 args.base_link,
                    'robot_segmenter.update_link_sphere_server':
                                                                    args.update_link_sphere_server_segmenter,
                },
            )
        )

    # object‐attachment block unchanged…

    return actions


def generate_launch_description() -> LaunchDescription:
    default_urdf = os.path.join(
        get_package_share_directory('isaac_ros_cumotion'),
        'curobo_core/curobo/src/curobo/content/assets/robot/kinova/kinova_gen3_7dof.urdf'
    )
    default_xrdf = os.path.join(
        get_package_share_directory('isaac_ros_cumotion'),
        'curobo_core/curobo/src/curobo/content/configs/robot/kinova_gen3.yml'
    )

    args = lu.ArgumentContainer()
    args.add_arg('camera_type')
    args.add_arg('no_robot_mode',        False)
    args.add_arg('enable_object_attachment', False)
    args.add_arg('from_bag',             False)
    args.add_arg('num_cameras',          1)
    args.add_arg('workspace_bounds_name','')
    args.add_arg('use_sim_time',         False)
    args.add_arg(
        'urdf_file_path',
        cli=True,
        default=default_urdf,
        description='The URDF for curobo (sim planning)')
    args.add_arg(
        'robot_file_name',
        cli=True,
        default=default_xrdf,
        description='The XRDF robot description for cuMotion')
    args.add_arg(
        'time_dilation_factor',
        cli=True,
        default='0.25',
        description='Speed scaling factor for the planner')
    args.add_arg(
        'distance_threshold',
        cli=True,
        default='0.05',
        description='Mask distance threshold for robot segmenter')
    args.add_arg(
        'time_sync_slop',
        cli=True,
        default='0.1',
        description='Time sync tolerance for segmenter')
    args.add_arg(
        'filter_depth_buffer_time',
        cli=True,
        default='0.1',
        description='Max age for depth buffer in object‐attachment')
    args.add_arg(
        'joint_states_topic',
        cli=True,
        default='/joint_states',
        description='Topic for joint states')
    args.add_arg(
        'tool_frame',
        cli=True,
        default='fr3_hand',
        description='End‐effector link frame name')
    args.add_arg(
        'base_link',
        cli=True,
        default='base',
        description='Root base link frame name')             # <-- new
    args.add_arg(
        'read_esdf_world',
        cli=True,
        default='True',
        description='Enable ESDF lookup')
    # … plus your other existing args …

    args.add_opaque_function(add_cumotion)
    return LaunchDescription(args.get_launch_actions())
