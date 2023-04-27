

# def get_immediate_child(id: int, task_list: dict, linear_job: list):
def get_immediate_child(id: int, task_list: dict, linear_job: list):
    '''Recursively get the next child in a list of tasks and add it to linear job.
    '''
    for ticket_id, ticket in task_list.items():
        # TODO: Referring to tix that don't exist? Can maybe reduce the
        # effective time by only starting from row.
        # print(ticket)
        if id in ticket["parents"]:
            linear_job.append(ticket_id)
            get_immediate_child(
                ticket_id, 
                task_list, 
                linear_job
            )

def get_immediate_parent(id: int, task_list: dict, linear_job: list):
    '''Recursively get the parents of a node using DFS.'''
    for parent in task_list[id]["parents"]:
        if parent in task_list.keys():
            linear_job.append(parent)
            get_immediate_parent(
                parent,
                task_list,
                linear_job
            )