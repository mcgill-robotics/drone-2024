# Workspace with packages that are relevant to the onboard operations of the drone.
Some packages, like `ros_gazebo_bridge` are only relevant to the simulation of the drone
## C++ qol tip
If you're using a c++ language server that is compatible with the `compile_commands.json` file, you can generate such a file for your whole workspace by running `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1`, the `compile_commands.json` file will be inside the build directory. We can then symlink it to where it is needed `ln -s ./build/compile_commands.json <path where it is needed>`
