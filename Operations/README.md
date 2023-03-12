# Operations

In this section, we employ operations research methods to solve the scheduling and assignment problems in a decoupled manner.

On this level, we consider jobs in a less modular fashion than the Metric/ and Topological/ methods. Where those considered tasks to be the atoms in the scheduling space, for the Operations approach, we look on the job/product level. The reasoning behind this is that based on our site visits, we reckon that the added burden of having to load and unload materials onto the robots between tasks will diminish the overall utility of the entire system. To this end, we instead opt to keep the same robots assigned to all the tasks within a job.


## Approach
We formulate the scheduling-assignment problem in two layers. First, we tackle the scheduling problem by casting it as a flexible job shop scheduling problem (FJSSP), which is an extension of the job shop scheduling problem (JSSP) that allows for route flexibility in the scheduling plan.


 for sticky jobs - machines are tied to jobs instead of their more granular subtasks.