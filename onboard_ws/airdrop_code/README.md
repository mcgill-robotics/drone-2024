# Micro ros part of the code.
Arduino code for the Arduino Due can be found here. To flash the arduino, you will Arduino IDE version 1, and to follow the instructions in the [README.md of this github repo](https://github.com/micro-ROS/micro_ros_arduino/tree/humble), also, need to the Patch SAM part as well. 
## Micro ros agent
It seems that there is an issue building micro ros agent, so we will use the following docker command instead 
`sudo docker run -it --rm --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0`
