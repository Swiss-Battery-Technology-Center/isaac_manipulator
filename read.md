
to launch cumotion with franka fr3
ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args -p robot:=/workspaces/cumotion/isaac_ros_cumotion/franka_description/fr3.xrdf -p urdf_path:=/workspaces/cumotion/isaac_ros_cumotion/franka_description/robots/fr3/fr3.urdf


launch cumotion and nvbloxs fr3

ros2 launch isaac_manipulator_bringup cumotion_nvblox_ready.launch.py  camera_type:=realsense num_cameras:=1 setup:=test  robot_file_name:=/workspaces/cumotion/isaac_ros_cumotion/franka_description/fr3.xrdf  urdf_file_path:=/workspaces/cumotion/isaac_ros_cumotion/franka_description/robots/fr3/fr3.urdf


kinova

ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args -p robot:=/workspaces/cumotion/isaac_ros_cumotion/curobo_core/curobo/src/curobo/content/configs/robot/kinova_gen3.yml -p urdf_path:=/workspaces/cumotion/isaac_ros_cumotion/curobo_core/curobo/src/curobo/content/assets/robot/kinova/kinova_gen3_7dof.urdf