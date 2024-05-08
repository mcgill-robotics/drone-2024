# ROS package responsible for bridging lidar from gazebo ignition topic to ros topic
Also sets up a frame for making visualization with Rviz2 possible.
Publishes lidar scans under `/lidar_scan` topic, with type `sensor_msgs/msg/LaserScan`.
# Required package
## gz-transport12
This should be installed by default with your gazebo garden installation
