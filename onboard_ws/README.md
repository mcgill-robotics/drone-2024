# Workspace with packages that are relevant to the onboard operations of the drone.
Some packages, like `ros_gazebo_bridge` are only relevant to the simulation of the drone
## C++ qol tip
If you're using a c++ language server that is compatible with the `compile_commands.json` file, you can generate such a file for your whole workspace by running `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1`, the `compile_commands.json` file will be inside the build directory. We can then symlink it to where it is needed `ln -s ./build/compile_commands.json <path where it is needed>`
## Topic monitoring through network
Make sure to disable the firewall on both devices and either dont have `ROS_DOMAIN_ID` environment variable set or have both devices share the same value for that variable
## Dependencies
Make sure to run `rosdep update && rosdep install --from-paths src --ignore-src -y` to get the necessary ros dependencies for each package. Note however that different packages may have non ros dependencies, refer to their individual README files.
## PS
Drone ssh port is `33556`, currently address `192.168.0.5`, user `drone` if using the TPlink.
Drone ssh port is `33556`, currently address `172.20.10.10`, user `drone` if using Imad's hotspot.
