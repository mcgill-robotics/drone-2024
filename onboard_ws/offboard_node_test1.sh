#!/bin/bash

source ./install/setup.bash

## Takeoff
ros2 service call /px4_action_queue/append_action custom_msgs/srv/SendAction "{action: 1, x: 0., y: 0., z: 0.}"

## Waypoints
ros2 service call /px4_action_queue/append_action custom_msgs/srv/SendAction "{action: 2, x: 0., y: 10., z: -5.}"

ros2 service call /px4_action_queue/append_action custom_msgs/srv/SendAction "{action: 2, x: 200., y: 10., z: -5.}"

ros2 service call /px4_action_queue/append_action custom_msgs/srv/SendAction "{action: 2, x: 100., y: 10., z: -5.}"

ros2 service call /px4_action_queue/append_action custom_msgs/srv/SendAction "{action: 2, x: 0., y: 0., z: -5.}"

## Land
ros2 service call /px4_action_queue/append_action custom_msgs/srv/SendAction "{action: 3, x: 0., y: 0., z: 0.}"
