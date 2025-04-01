import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(
            'world_frame',
            default_value='base_link',
            description='The world frame of the robot'),
        DeclareLaunchArgument(
            'home_frame',
            default_value='home_frame',
            description='home where robot should go'),
        DeclareLaunchArgument(
            'grasp_frame',
            default_value='grasp_frame',
            description='home where robot should go'),
        DeclareLaunchArgument(
            'plan_timer_period',
            default_value='0.01',
            description='The time in seconds for which the goal should request a plan'),
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
            default_value='tool_frame',
            description='The name of the end effector link for planning'),
    ]

    world_frame = LaunchConfiguration('world_frame')
    home_frame = LaunchConfiguration('home_frame')
    grasp_frame = LaunchConfiguration('grasp_frame')
    plan_timer_period = LaunchConfiguration('plan_timer_period')
    planner_group_name = LaunchConfiguration('planner_group_name')
    pipeline_id = LaunchConfiguration('pipeline_id')
    planner_id = LaunchConfiguration('planner_id')
    end_effector_link = LaunchConfiguration('end_effector_link')

    home_node = Node(
        package='isaac_ros_moveit_goal_setter',
        namespace='',
        executable='home_node',
        name='home_node',
        parameters=[{
            'world_frame': world_frame,
            'home_frame': home_frame,
            'grasp_frame': grasp_frame,
            'plan_timer_period': plan_timer_period,
            'planner_group_name': planner_group_name,
            'pipeline_id': pipeline_id,
            'planner_id': planner_id,
            'end_effector_link': end_effector_link,
        }],
        output='screen'
    )

    return launch.LaunchDescription(launch_args + [home_node])