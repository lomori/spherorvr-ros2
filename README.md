# spherorvr-ros2
Minimal implementation of a ROS2 node for the Sphero RVR robot. See hardware description at [ROS2 Robot (Sphero RVR) with Localization, Navigation, AI, and a Thermal Imager](https://youtu.be/RVCkEL206kc).

## Mapping
1. Reset RVR (on/off): this is necessary to reset internal Sphero odometry.
1. run **ros2 run sphero sphero_node**
1. run **ros2 launch rplidar_ros rplidar.launch.py frame_id:=laser** (ROS2 RP LIDAR node)
1. Start rviz2
1. run **ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3'** (to move the rover around during mapping)
1. run **ros2 launch mapping_slam_async_launch.py** (start SLAMToolbox in mapping mode)

## Start Navigation
1. Reset RVR (on/off): this is necessary to reset internal Sphero odometry.
1. run **ros2 run sphero sphero_node**
1. run **ros2 launch rplidar_ros rplidar.launch.py frame_id:=laser** (ROS2 RP LIDAR node)
1. Start rviz2 (otherwise it may lose the initial map)
1. run **ros2 launch navigation_launch.py** (start navigation2)
1. run **ros2 launch localization_slam_async_launch.py** (start SLAMToolbox localization)
1. run **ros2 run sphero image_publisher** (for publishing thermal and visible light images)
