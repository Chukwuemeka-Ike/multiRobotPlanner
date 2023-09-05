# Workspace

This folder contains the high-level components in a readily-usable form. The README holds instructions for installing, building, and using the high-level portion of the project.


## Status
![Status - Work in Progress](https://img.shields.io/badge/status-Work%20in%20Progress-yellow)


## Installation
The following instructions assume you're running an Ubuntu 20.04 system with ROS Noetic Desktop-Full. If that is not the case, these instructions may not work exactly as intended.
1. [Ubuntu Install Instructions](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
2. [ROS Noetic Install Instructions](https://wiki.ros.org/noetic/Installation/Ubuntu)

### Repositories
Clone this repository into a new folder.
```bash
cd
mkdir multiRobotPlanner
cd multiRobotPlanner
git clone https://github.com/Chukwuemeka-Ike/multiRobotPlanner.git . 
```

Clone the following repositories by Burak into the *src/* subfolder for running the simulations.
```bash
cd workspace/src
git clone https://github.com/burakaksoy/RVizMeshVisualizer.git
git clone https://github.com/burakaksoy/AssistiveRobot-SimulationFiles.git
git clone https://github.com/burakaksoy/uwb_gazebo_plugin.git
git clone https://github.com/burakaksoy/Swarm-Robotics-2.git
```

### Project Dependencies
#### System-Level Packages
First, we start by installing the system-level package dependencies. The list of packages required to run this project are listed in *[apt_requirements.txt](https://github.com/Chukwuemeka-Ike/multiRobotPlanner/blob/master/workspace/apt_requirements.txt)*. The *[install_apt_packages](https://github.com/Chukwuemeka-Ike/multiRobotPlanner/blob/master/workspace/install_apt_packages.bash)* bash script installs all the packages in that file.

```bash
cd workspace
chmod +x install_apt_packages.bash # make the script executable
sudo ./install_apt_packages.bash
```

The user interfaces in this project contain RViz windows that show a simulated representation of what is happening on the factory floor. To do so, we rely on the work done by [Burak Aksoy](https://github.com/burakaksoy). His work brings additional package dependencies to this project, which can be installed using the commands below:
```bash
chmod +x install_burak_dependencies.bash
sudo ./install_burak_dependencies.bash
```

#### Python Packages
Next, we install the Python dependencies.
```bash
pip3 install -r py_requirements.txt
```

### Build
Navigate to *workspace*, build the packages, and source the built workspace packages with the following commands.
```bash
cd workspace
catkin_make
source devel/setup.bash
```
If this runs correctly, the project's components should be ready to use.

## Usage
### High-Level System
The high-level planning system is made up of the following components.
1. Ticket Manager (TM)
2. Task Scheduler (TS)
3. Machine Manager (MM)
4. Robot Assigner (RA)

To use the high level system, we need components 1-4 running to support the overall operation. Open a terminal window and launch those components using:
```bash
roslaunch arm_utils high_level.launch
```

With these running, the system can now accept tickets into the TM, schedule those tickets (TS), assign the tickets to machines, and assign available robots to the jobs (RA).

For testing, the *add_tickets* script adds a set of tickets, which triggers the TM to request a schedule from the TS and to update the tickets when the schedule is generated. Use the script by running:
```bash
rosrun ticket_manager add_tickets.py
```


### User Interface
There are two user interfaces for working with the system.
1. Supervisor GUI (SG) - ticket management and overall system monitoring
2. Operator GUI (OG) - ticket operation and ticket-specific robot control

With the high level background components running, we can now run the UI's.
```bash
roslaunch arm_gui both_guis.launch
```

## Descriptions
### Tickets
A Ticket is the atomic component of the high-level system. It represents a task that needs to be completed towards building a piece. Jobs can be built using multiple tickets. The Ticket message template is shown below.
```bash
# Ticket message.
Header header
uint32 job_id
uint32 ticket_id
uint32[] parents
float32 duration
uint32 machine_type
uint32 num_robots
string status
uint32 start
uint32 end
uint32 machine_id 
float32 time_left
```
Each Ticket starts with basic information about the task that is needed for scheduling:
1. ticket ID - unique identifier for the ticket
2. parents - the set of tickets that must be completed before the ticket can start
3. duration - estimate of how long the ticket should take
4. machine_type - the type of machine needed for the task
5. num_robots - number of robots requested for the task

When the scheduler generates a schedule, it adds information for the task's start and end times, and assigns the task a machine_id.

### Machines
Each machine in the workspace is given a unique identifier by the Machine Manager. The scheduler assigns tickets to machines using their ID's, and instances of the Operator GUI are bound to specific machine IDs to determine which tickets they can work on.