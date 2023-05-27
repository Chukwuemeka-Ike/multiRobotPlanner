# Schedulers

Scripts in this folder are different types of schedulers for quickly comparing how they perform on different job sets, decoupled from ROS and any other mechanisms we're using elsewhere in the project.

## FIFO Scheduler
The FIFO scheduler follows the first-in-first-out (FIFO) method used by the company. The basic idea is that as products go down the line, they are sent to the machines they need in a queue fashion. Tasks are sent to the shortest queue of a station type to attempt to optimize the overall production time.

We use this scheduler as a benchmark for the optimal scheduler. Given actual execution times of the tasks that make up a product, could we have sped up the process using the optimal scheduler?

### How does the script work?
1. Converts the job list to a dictionary of tickets. Each entry in this dict has the ticket ID as its key, and the remmaining ticket information in a dictionary as the value.
2. Creates queues for each unique station in the workspace. 
3. Creates a queue schedule Pandas DataFrame.
4. Creates a dictionary of queues to allow us use queue ID's as keys.
5. Adds all loading tickets in the job lis to the loading queues.
6. Creates the ticket schedule Pandas DataFrame.
7. Runs the iterate() function. The iterate() function repeatedly steps the system forward and updates ticket states until there are no more tickets available.

## Optimal Scheduler
The optimal scheduler uses the flexible job shop scheduling problem (FJSSP) to attempt to minimize the overall time spent on building all products in the input set. The method implemented is directly from the paper "*Mathematical models for job-shop scheduling problems with routing and process plan flexibility*" - Ozguven et al (2010). There are two solvers provided by Google OR-Tools used here - the linear program (LP) solver and the constraint programming satisfiability (CP-SAT) solver. We've found that the CP-SAT solver runs significantly faster than the LP solver, so that's what we have been using in the Schedule Monitor ROS node.

## Optimal Scheduler Idle Time
Uses the same optimization method from the Optimal Schedulers, but has a second optimization that instead minimizes the total idle time in the schedule given the output of the initial optimization.

The second optimization adds the optimal makespan from the first as a constraint, then changes the objective function to minimize the total idle time between consecutive tasks within a job. It allows us to prioritize reducing how much idle time the robots might have. We have this implemented but don't know if it is of value to the company.