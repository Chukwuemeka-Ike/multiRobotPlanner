# Multi-Robot Task Scheduling and Assignment

This repository holds the high-level components developed in fulfillment of the **[ARM-TEC-22-01-F-30]()** project titled "Multi-Robot Task Planning in Large Structure Manufacturing" and funded by the [ARM Institute](https://arminstitute.org/). The high-level components are responsible for the generation of optimal task schedules and robot assignments to support a manufacturing process, and for providing user interfaces for working on those tasks with the assigned robots.

These components are designed to tie into the lower-level control components designed by [Burak Aksoy](https://github.com/burakaksoy) to complete the entire project. Where appropriate, the complementary repositories are described and linked.

## Project Status
![Project Status - Work in Progress](https://img.shields.io/badge/status-Work%20in%20Progress-yellow)

This project is currently under active development.

## Folder Descriptions
Each major folder in this repo represents a specific iteration of the task scheduler and path planner that were developed for the project. Detailed descriptions are available in the folder README's.

**NOTE: ws is the only folder intended for use. Other folders are included here because they represent the entirety of the design process for this project.**

### ws
*ws* contains the final version of the entire high-level system, including the Operations scheduler. The workspace contains all the packages necessary for running the version of this project intended for use and distribution. Those interested in using this project can skip the remainder of this README. More details and documentation can be found [here](https://github.com/Chukwuemeka-Ike/multiRobotPlanner/tree/master/ws).

### Finegrained
The finegrained planner was the first iteration for robot planning. It used an optimization tool and a constraint solver to generate feasible paths for mobile robots in a 2D plane.

The approach was not used because it was not suitable for the overall problem. It planned paths, but provided a tedious input mechanism to sequence multiple visits in the manner needed for creation of a product. More details and documentation can be found [here](https://github.com/Chukwuemeka-Ike/multiRobotPlanner/tree/master/Finegrained).

### Metric
The metric planner was designed to generate task schedules and paths for multiple robots in a mapped environment while accounting for the robot's physical capabilities - movement speed and planning time-step. The approach required too much computational effort in the initial setup stage, and it was forgone in favor of a Topological representation of the map. More details and documentation can be found [here](https://github.com/Chukwuemeka-Ike/multiRobotPlanner/tree/master/Metric).

### Topological
The topological planner was designed to overcome the Metric planner's computational inefficiency, and it provided a 60% speedup on average on the same tasks. The robot's physical capabilities were ignored, and the entire workspace was converted to a graph representation. It was eventually understood that the modularity this approach provided did not completely fit the intended use, and thus the planning time became a disadvantage. More details and documentation can be found [here](https://github.com/Chukwuemeka-Ike/multiRobotPlanner/tree/master/Topological).

### Operations
This version of scheduler was eventually chosen to be part of the high-level system. In this iteration, we separated the task scheduling, robot assignment, and path planning into three distinct parts unlike in its predecessors where they were combined. Doing so sped up the entire scheduling and planning system and fit the company's use case significantly better. More details and documentation can be found [here](https://github.com/Chukwuemeka-Ike/multiRobotPlanner/tree/master/Operations).


## Links
### Project in The News
1. [ARM Institute Announcement](https://arminstitute.org/news/new-tech-projects-2023/)
2. [RPI News](https://news.rpi.edu/content/2023/02/09/rpi-awarded-two-technology-projects-advanced-robotic-manufacturing-arm-address)