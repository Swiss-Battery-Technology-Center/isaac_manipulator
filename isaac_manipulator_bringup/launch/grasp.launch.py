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

import isaac_ros_launch_utils.all_types as lut
import isaac_ros_launch_utils as lu

import isaac_manipulator_ros_python_utils.constants as constants
from isaac_manipulator_ros_python_utils.types import CameraType, DepthType, TrackingType


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg(
        'setup',
        default='home',
        cli=True,
        description='The name of the setup you are running on (specifying calibration '
                    'workspace bounds and camera ids).'
    )
    args.add_arg(
        'camera_type',
        default='',
        cli=True,
        description='Camera type (empty => no cameras)'
    )
    actions = args.get_launch_actions()

    actions.append(
    lu.include(
        'isaac_manipulator_bringup',
        'launch/include/cumotiongraps.launch.py',
        launch_arguments={
            'workspace_bounds_name': args.setup,
        },
    ))
        # Goal setter
    actions.append(
        lu.include(
            'isaac_manipulator_bringup',
            'launch/include/home.launch.py',
        ))
    actions.append(
    lu.include(
        'isaac_manipulator_bringup',
        'launch/include/home_tf.launch.py',
        launch_arguments={
            'tracking_type': TrackingType.pose_to_pose,
            'calibration_name': args.setup,
        },
    ))
    return lut.LaunchDescription(actions)

