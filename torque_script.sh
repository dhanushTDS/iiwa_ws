#!/bin/bash

# Launch torque_recorder.launch in a new terminal
gnome-terminal -- bash -c "catkin build; source devel/setup.bash; cd src/iiwa_package-main/iiwa_cam/launch; roslaunch torque_recorder.launch; exec bash"

# Wait for the first terminal to initialize before launching the second one
sleep 5

# Launch toggler.launch in a new terminal
gnome-terminal -- bash -c "catkin build; source devel/setup.bash; cd src/iiwa_package-main/iiwa_cam/launch; roslaunch toggler.launch; exec bash"

