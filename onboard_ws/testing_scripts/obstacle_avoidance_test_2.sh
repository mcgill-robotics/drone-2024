#!/bin/bash

source ../install/setup.bash
## build fence
ros2 topic pub --once /boundary_setter geometry_msgs/msg/Polygon "{points: [{x: -20., y: -20.}, {x: 20., y: -20.}, {x: 20., y: 20.}, {x: 40., y: 20.}, {x: 40., y: 40.}, {x: -20., y: 40.}]}"

## goto destination
ros2 topic pub --once /goal_waypoint custom_msgs/msg/Waypoint "{x: 30.0, y: 25., z: -10., max_speed_h: 8.0}"
