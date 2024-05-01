# drone 2024
Use  `git clone <https-link>` to clone this repo.
Drone and various other models can be found under models.
README files can be found under the `onboard_ws` and `off_board_ws` folders which are relevant to their respective workspaces
# Packages structure
Please test abundantly, for each node write an equivalently named
test python script which tests the functionallity of that node and the 
various functions that can found there. Refer to [this tutorial for testing ros 2 code](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html)

    \<pkg_name>
        \<pkg_name>
            <nodes.py>
        \tests
            <node_test.py>
        setup.cfg
        setup.py
        package.xml

# Simulation
## How to simulate custom model 
### Step 1 
Copy the folder `models/iris_lidar` to  `{PX4_HOME}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models`
### Step 2
Overwrite the content of `{PX4_HOME}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/lidar/model.sdf`
 with the content of the file `models/lidar/model.sdf`
### Step 3
Copy the airframe file `airframe/22001_...` under `{PX4_HOME}/ROMFS/px4fmu_common/init.d-posix/airframe` and overwrite the content of `{PX4_HOME}/ROMFS/px4fmu_common/init.d-posix/airframe/CMakeLists.txt` with the content of the file `models/airframe/CMakeLists.txt`.
### Step 4
Add `iris_lidar` to the set(model ... section of the file `{PX4_HOME}/src/modules/simulation/simulator_mavlink/sitl_target_gazebo.cmake` 
(Or just overwrite its content with the content of the file `models/sitl_targets_gazebo-classic.cmake`)
## Finally
Go to `{PX4_HOME}` and run `make px4_sitl gazebo-classic_iris_lidar`
