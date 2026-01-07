# Isaac Manipulator

This repository contains the integrated Isaac Manipulator launch files.

## Setup and Documentation

Visit [Isaac Manipulator](https://nvidia-isaac-ros.github.io/reference_workflows/isaac_manipulator/index.html) to learn how to use this repository.


ros2 launch isaac_manipulator_bringup cumotion_nvblox_ready.launch.py camera_type:=realsense num_cameras:=1 setup:=test  robot_file_name:=/workspaces/cumotion/isaac_ros_cumotion/franka_description/fr3.xrdf  urdf_file_path:=/workspaces/cumotion/isaac_ros_cumotion/franka_description/robots/fr3/fr3.urdf