#!/usr/bin/bash
# This script runs the robot simulations and ensures the shell has the correct env variables for Gazebo.

# Get the script's directory.
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
echo "Running simulation from $SCRIPT_DIR"

# Set the environment variables for Gazebo.
export GAZEBO_MODEL_PATH=$SCRIPT_DIR/src/AssistiveRobot-SimulationFiles/lab_gazebo/models
export GAZEBO_RESOURCE_PATH=$SCRIPT_DIR/src/AssistiveRobot-SimulationFiles/lab_gazebo/worlds
export SDF_PATH=$SCRIPT_DIR/src/AssistiveRobot-SimulationFiles/lab_gazebo/models
export IGN_FILE_PATH=$SCRIPT_DIR/src/AssistiveRobot-SimulationFiles/lab_gazebo/worlds

# Source the workspace.
source $SCRIPT_DIR/devel/setup.bash

# Roslaunch all the relevant nodes.
roslaunch arm_utils robot_sim.launch