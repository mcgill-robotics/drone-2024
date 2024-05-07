# Simulation
Due to compatibility reasons with ROS2, we will migrate the simulation from gazebo classic to gazebo ignition. Thankfully, most
of the work is already done for us by PX4 for both the simulation and the transition.
## Migration from classic to ignition
### Step 0
First, remove gazebo. Most likely `sudo apt remove gazebo && sudo apt autoremove` will do the job quite nicely.
### Step 1
You also want to `rm -rf <Path to your PX4 folder>`, so that we can reset our px4 to a fresh version.
### Step 3
run `cd ~ && git clone https://github.com/PX4/PX4-Autopilot.git --recursive`, this will take a while.
### Step 4
run `cd ~ && bash ./PX4-Autopilot/Tools/setup/ubuntu.sh`, this may take a while as well.
### Thats it
if you're on ubuntu 22.04 LTS (as you should be for the MicroXRCE agent to work with ROS2 humble and properly interface with PX4), step 4 will automatically install gazebo ignition 
## Simulation
Copy the content of the file under `iris_lidar/model.sdf` to the file under 
`<Your PX4 Folder>/Tools/simulation/gz/models/x500/model.sdf`
, `cd <Your PX4 Folder> && make px4_sitl gz_x500`.
