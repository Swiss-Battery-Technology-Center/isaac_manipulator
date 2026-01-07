# home.launch.py

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(
            'world_frame',
            default_value='base_link',
            description='The world frame of the robot (used as planning frame if not overridden)'),
        DeclareLaunchArgument(
            'home_frame',
            default_value='home_frame',
            description='Target frame where robot should go initially'),
        DeclareLaunchArgument(
            'post_grasp_lift_frame', # NEW
            default_value='post_grasp_lift_frame',
            description='Target frame after grasping and closing gripper'),
        DeclareLaunchArgument(
            'drop_off_frame', # NEW
            default_value='drop_off_frame',
            description='Target frame for dropping off the object'),
        DeclareLaunchArgument(
            'plan_timer_period',
            default_value='0.01',
            description='The time in seconds for the goal state machine tick'),
        DeclareLaunchArgument(
            'planner_group_name',
            default_value='manipulator',
            description='The MoveIt group name that the planner should plan for'),
        DeclareLaunchArgument(
            'pipeline_id',
            default_value='isaac_ros_cumotion',
            description='The MoveIt pipeline ID to use'),
        DeclareLaunchArgument(
            'planner_id',
            default_value='cuMotion',
            description='The MoveIt planner ID to use'),
        DeclareLaunchArgument(
            'end_effector_link',
            default_value='tool_frame', # Make sure this matches your robot's EE link
            description='The name of the end effector link for planning'),
         DeclareLaunchArgument( # Optional: Add if you need to override planning frame
            'planning_frame',
            default_value='', # Defaults to world_frame in the node if empty
            description='The MoveIt planning frame override (e.g., base_link)'),
         DeclareLaunchArgument(
            'gripper_action_name',
            default_value='/robotiq_gripper_controller/gripper_cmd', # VERIFY THIS NAME
            description='The action server name for controlling the gripper'),
         DeclareLaunchArgument(
            'gripper_open_position', # NEW
            default_value='0.0', # Example for Robotiq: 0.0 = fully open
            description='Position value for opening the gripper'),
         DeclareLaunchArgument(
            'gripper_close_position', # NEW
            default_value='100.0', # Example for Robotiq: > ~0.7 depends on object, use high value or measured closed value
            description='Position value for closing the gripper'),
         DeclareLaunchArgument(
            'gripper_max_effort', # NEW
            default_value='100.0', # Adjust as needed
            description='Max effort for gripper commands'),
    ]

    world_frame = LaunchConfiguration('world_frame')
    home_frame = LaunchConfiguration('home_frame')
    post_grasp_lift_frame = LaunchConfiguration('post_grasp_lift_frame') # NEW
    drop_off_frame = LaunchConfiguration('drop_off_frame') # NEW
    plan_timer_period = LaunchConfiguration('plan_timer_period')
    planner_group_name = LaunchConfiguration('planner_group_name')
    pipeline_id = LaunchConfiguration('pipeline_id')
    planner_id = LaunchConfiguration('planner_id')
    end_effector_link = LaunchConfiguration('end_effector_link')
    planning_frame_arg = LaunchConfiguration('planning_frame') # NEW
    gripper_action_name = LaunchConfiguration('gripper_action_name') # NEW
    gripper_open_position = LaunchConfiguration('gripper_open_position') # NEW
    gripper_close_position = LaunchConfiguration('gripper_close_position') # NEW
    gripper_max_effort = LaunchConfiguration('gripper_max_effort') # NEW


    home_node = Node(
        package='isaac_ros_moveit_goal_setter', # Make sure package name is correct
        namespace='',
        executable='home_node', # Make sure executable name is correct
        name='home_node',
        parameters=[{
            'world_frame': world_frame,
            'home_frame': home_frame,
            'post_grasp_lift_frame': post_grasp_lift_frame, # NEW
            'drop_off_frame': drop_off_frame, # NEW
            'plan_timer_period': plan_timer_period,
            'planner_group_name': planner_group_name,
            'pipeline_id': pipeline_id,
            'planner_id': planner_id,
            'end_effector_link': end_effector_link,
            'planning_frame': planning_frame_arg, # Pass planning frame override
            'gripper_action_name': gripper_action_name, # Pass gripper action name
            'gripper_open_position': gripper_open_position, # NEW
            'gripper_close_position': gripper_close_position, # NEW
            'gripper_max_effort': gripper_max_effort, # NEW
        }],
        output='screen'
    )

    return launch.LaunchDescription(launch_args + [home_node])