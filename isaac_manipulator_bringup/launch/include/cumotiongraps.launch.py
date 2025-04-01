import os
from typing import List

import isaac_ros_launch_utils as lu
from isaac_ros_launch_utils.all_types import Action, LaunchDescription


def add_cumotion(args: lu.ArgumentContainer) -> List[Action]:
    """
    Minimal version of cuMotion launch with NO nvblox references.
    - No workspace/ESDF references
    - No camera
    - If from_bag or no_robot_mode => do nothing
    - Otherwise => include cuMotion
    """
    from_bag = lu.is_true(args.from_bag)
    no_robot_mode = lu.is_true(args.no_robot_mode)
    actions: List[Action] = []

    # If not from_bag & not no_robot_mode => run cuMotion
    # Otherwise do nothing
    if not from_bag and not no_robot_mode:
        actions.append(
            lu.include(
                'isaac_ros_cumotion',
                'launch/isaac_ros_cumotion.launch.py',
                launch_arguments={
                    'cumotion_planner.robot': args.robot_file_name,
                    # You can still pass other arguments if cuMotion needs them
                    'cumotion_planner.time_dilation_factor': args.time_dilation_factor,
                    'cumotion_planner.tool_frame': args.tool_frame,
                    'cumotion_planner.read_esdf_world': 'False',  # or remove entirely
                    'cumotion_planner.publish_curobo_world_as_voxels': 'False',
                    'cumotion_planner.override_moveit_scaling_factors': 'True',
                    'cumotion_planner.joint_states_topic': args.joint_states_topic,
                    'cumotion_planner.update_link_sphere_server':
                        args.update_link_sphere_server_planner,
                    'cumotion_planner.urdf_path': args.urdf_file_path,
                },
            )
        )
    else:
        actions.append(
            lu.log_info([
                "Not starting cuMotion (because either 'from_bag=True' or 'no_robot_mode=True')."
            ])
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    """
    Launch file that starts cuMotion, with NO references to nvblox or ESDF visualizer.
    """
    # Example defaults; adjust as needed
    default_urdf_file_path = '/workspaces/cumotion/isaac_ros_cumotion/curobo_core/curobo/src/curobo/content/assets/robot/kinova/kinova_gen3_7dof.urdf'
    default_xrdf_file_path = '/workspaces/cumotion/isaac_ros_cumotion/curobo_core/curobo/src/curobo/content/configs/robot/kinova_gen3.yml'

    args = lu.ArgumentContainer()
    args.add_arg('no_robot_mode', False)
    args.add_arg('from_bag', False)
    args.add_arg('use_sim_time', False)

    # We keep these for the cuMotion pipeline
    args.add_arg(
        'urdf_file_path',
        cli=True,
        default=default_urdf_file_path,
        description='The URDF file for cuMotion to ingest for planning'
    )
    args.add_arg(
        'robot_file_name',
        cli=True,
        default=default_xrdf_file_path,
        description='XRDF that describes the robot'
    )
    args.add_arg(
        'time_dilation_factor',
        cli=True,
        default='0.25',
        description='Speed scaling factor for the planner'
    )
    args.add_arg(
        'joint_states_topic',
        cli=True,
        default='/joint_states',
        description='The joint states topic for robot positions'
    )
    args.add_arg(
        'tool_frame',
        cli=True,
        default='end_effector_link',
        description='The tool frame of the robot'
    )
    args.add_arg(
        'update_link_sphere_server_planner',
        cli=True,
        default='planner_attach_object',
        description='Name for updating link sphere server in the planner'
    )

    actions = args.get_launch_actions()

    return LaunchDescription(actions)
