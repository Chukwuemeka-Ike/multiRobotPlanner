# Operations

## Status
![Status - Complete](https://img.shields.io/badge/status-Complete-gb)

In this section, we employ operations research methods to solve the scheduling and assignment problems in a decoupled manner.

On this level, we consider jobs in a less modular fashion than the Metric/ and Topological/ methods. Where those considered tasks to be the atoms in the scheduling space, for the Operations approach, we look on the job/product level. The reasoning behind this is that based on our site visits, we reckon that the added burden of having to load and unload materials onto the robots between tasks will diminish the overall utility of the entire system. To this end, we instead opt to keep the same robots assigned to all the tasks within a job.


## Approach
We formulate the scheduling-assignment problem in two layers. First, we tackle the scheduling problem by casting it as a flexible job shop scheduling problem (FJSSP), which is an extension of the job shop scheduling problem (JSSP) that allows for route flexibility in the scheduling plan.

The basic FJSSP is structured as follows: Given *n* jobs with *p<sub>i</sub>* sub-tasks and *m* machines/stations, where each job's sub-tasks need to be performed in a specific order, and sub-tasks must run on specified machines for specified durations, generate a minimum time schedule for all the jobs.

We implemented a mixed integer linear programming (MILP) algorithm<sup>[1]</sup> to solve the problem with constraints for:
* Machine types and durations of tasks
* Order of tasks within a job
* No overlap between tasks on the same machine

We then use [Google OR-Tools CP-SAT Solver](https://developers.google.com/optimization/cp/cp_solver) to solve the optimization problem and generate a schedule that meets our requirements.

The figures below show the job set that was input to the scheduler and the schedule that was subsequently generated.
|![Input Job Set](Images/inputData.png)|![Output Schedule](Images/schedule.png)|
|-|-|
|<center>Input Job Set</center>|<center>Output Schedule</center>|


## References
[1] - Mathematical models for job-shop scheduling problems with routing and process plan flexibility – Özgüven (2010)
