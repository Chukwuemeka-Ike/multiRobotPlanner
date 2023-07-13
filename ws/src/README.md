# ARM High-Level Planner ROS Packages
The packages in this workspace encompass the high-level components of the ARM project's ROS ecosystem. These components cover such aspects as:
1. User interfacing - Supervisor and Operator GUI's
2. Ticket management - Ticket Manager
3. Task scheduling - Task Scheduler
4. Robot assignment - Robot Assigner
5. Destination planning - Destination Planner

The overall objective of the high-level is to plan jobs and tasks, assign robots to those jobs, and to ensure that the robots head to the correct destinations at the right time. The ticket manager also maintains the states of jobs and tasks in the system.

## Core Packages
1. arm_gui
2. arm_msgs
3. arm_utils
4. destination_planner
5. robot_assigner
6. task_scheduler
7. ticket_manager

## External Packages
### Clearpath
1. dingo_control
2. dingo_desktop
2. dingo_description
3. dingo_msgs
4. dingo_simulator
5. dingo_viz

### Object Visualizer
1. RVizMeshVisualizer

### Burak
1. dingo_gazebo
2. dingo_description - dingo-o-prefixed.urdf.xacro
2. lab_gazebo
3. swarm2_launch
<!-- 3. swarm_msgs -->
3. topic_tf_transformers
4. uwb_pose_publisher
5. uwb_reader