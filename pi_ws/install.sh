#!/bin/bash
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install  -y ros-noetic-tf-conversions
sudo apt-get install -y ros-noetic-teleop-twist-keyboard
sudo apt-get install -y ros-noetic-diagnostic-updater
sudo apt-get install -y ros-noetic-navigation
sudo apt-get install -y ros-noetic-map-server
sudo apt-get install -y ros-noetic-slam-gmapping
sudo apt-get install -y ros-noetic-dwa-local-planner
sudo apt-get install -y ros-noetic-smach
