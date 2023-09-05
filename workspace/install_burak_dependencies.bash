#!/bin/bash
# These packages are needed exclusively by Burak's packages.
sudo apt-get install -y ros-noetic-imu-pipeline # for imu_transformer
sudo apt-get install -y ros-noetic-imu-tools
sudo apt-get install -y ros-noetic-navigation # for navigation stack
sudo apt-get install -y ros-noetic-spacenav-node
sudo apt-get install -y ros-noetic-tf2-sensor-msgs
sudo apt-get install -y spacenavd
sudo apt-get install -y sshpass