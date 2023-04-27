from collections import deque
from itertools import permutations
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from utils.draw_utils import draw_tree_schedule
from utils.draw_queue_utils import draw_queues

class FIFOScheduler:
    '''Class that generates a FIFO schedule given lists of jobs and stations.
    '''
    def __init__(self, job_list: list, station_type_names: list,
                num_station_types: int, num_stations: list) -> None:
        '''.

        Args:
            job_list: list of jobs in the desired priority order.
            num_station_types: number of unique station types.
            num_stations: number of each station type.
        '''
        self.job_list = job_list
        self.station_type_names = station_type_names

        self.waiting_tix = self.get_all_tix_dict()
        self.ready_tix = {}
        self.done_tix = {}

        self.cur_time = 0
        self.times = [self.cur_time]

        self.station_queues = self.StationQueues(
            num_station_types, num_stations
        )

        self.queue_schedule = pd.DataFrame(columns=["t"])
        for i, value in enumerate(self.station_queues.flat_station_list):
            col_name = f"Queue {i}"
            self.queue_schedule = self.queue_schedule.assign(
                **{col_name: value}
            )

        self.add_loading_to_queue1()

        self.schedule = pd.DataFrame(
            columns=[
                "Job #", "Task #", "Ticket ID","Parents", "Station #",
                "Station Type #", "Location", "Start", "End", "Duration"
            ]
        )


    def get_all_tix_dict(self) -> dict:
        '''Creates a dict of all tickets for access within the class.'''
        waiting_tix = {}
        for job_id, job in enumerate(self.job_list):
            for ticket in job:
                tix = {}
                tix["job_num"] = job_id
                tix["parents"] = ticket["parents"]
                tix["station_type"] = ticket["station_type"]
                tix["duration"] = ticket["duration"]
                tix["time_left"] = ticket["duration"]
                waiting_tix[ticket["ticket_id"]] = tix
        return waiting_tix

    def add_loading_to_queue1(self):
        '''Adds all the loading tickets to the loading queue.

        Does so in the order they appear in the job list.
        '''
        deletion_idx = []
        for ticket_id, ticket in self.waiting_tix.items():
            if ticket["station_type"] == 0:
                self.ready_tix[ticket_id] = self.waiting_tix[ticket_id]
                self.station_queues.queue_ticket_to_station_type(
                    ticket_id, ticket["duration"], ticket["station_type"]
                )
                deletion_idx.append(ticket_id)

        self.add_queue_states_to_schedule()

        # Remove the ticket_ids from waiting_tix.
        for ticket_id in deletion_idx:
            del(self.waiting_tix[ticket_id])

    def add_queue_states_to_schedule(self):
        '''.'''
        queue_row = {}
        for queue_id, queue in enumerate(self.station_queues.queues):
            queue_row[f"Queue {queue_id}"] = [list(queue)]
        queue_row["t"] = [self.cur_time]
        self.queue_schedule = pd.concat(
            [self.queue_schedule, 
             pd.DataFrame(queue_row)], ignore_index=True
        )

    def add_done_ticket_to_schedule(self, ticket_id, ticket, queue_id):
        '''Adds the ticket information to the schedule.'''
        ticket_row = {}
        ticket_row["Job #"] = [ticket["job_num"]]
        ticket_row["Task #"] = [len(
            self.schedule.loc[self.schedule["Job #"] == ticket["job_num"]]
        )]
        ticket_row["Ticket ID"] = [ticket_id]
        ticket_row["Parents"] = [ticket["parents"]]
        ticket_row["Station #"] = [queue_id]
        ticket_row["Station Type #"] = [ticket["station_type"]]
        ticket_row["Location"] = [self.station_type_names[ticket["station_type"]]]
        ticket_row["Start"] = [self.cur_time - ticket["duration"]]
        ticket_row["End"] = [self.cur_time]
        ticket_row["Duration"] = [ticket["duration"]]
        
        # self.schedule = self.schedule.append(ticket_row, ignore_index=True)
        # print(self.schedule)
        # print(ticket_row)
        self.schedule = pd.concat(
            [self.schedule,
             pd.DataFrame(ticket_row, index=[0])
            ],
            ignore_index=True
        )

    def step_forward(self):
        '''Updates the current time and finishes the next ticket.
        
        Steps the system forward by:
            1. finding the next finishing ticket,
            2. adds its duration to the current time,
            3. subtracts its duration from the other ongoing tickets,
            4. finishes the ticket by popping it from the queue and
                transferring it from ready_tix to done_tix.
        '''
        lowest_time_left = 1000
        lowest_id = 0
        next_ticket_queue_id = 0
        
        # Find the ongoing ticket with lowest duration.
        for ticket_id, ticket in self.ready_tix.items():
            for queue_id, queue in enumerate(self.station_queues.queues):
                if len(queue) and ticket_id == queue[0]:
                    if ticket["time_left"] < lowest_time_left:
                        lowest_time_left = ticket["time_left"]
                        lowest_id = ticket_id
                        next_ticket_queue_id = queue_id
        
        # Transfer the lowest ticket from ready to done.
        self.done_tix[lowest_id] = self.ready_tix[lowest_id]
        del(self.ready_tix[lowest_id])

        # Subtract the lowest time left from all other ongoing tickets.
        for ticket_id, ticket in self.ready_tix.items():
            for queue_id, queue in enumerate(self.station_queues.queues):
                if len(queue) and ticket_id == queue[0]:
                    ticket["time_left"] = ticket["time_left"] - lowest_time_left
                    self.station_queues.lengths[queue_id] -= lowest_time_left
        
        # Remove the lowest duration ticket from the top of the queue.
        self.station_queues.remove_ticket_from_queue(
            next_ticket_queue_id, lowest_time_left
        )

        # Update current time and time vector.
        self.cur_time += lowest_time_left
        self.times.append(self.cur_time)

        # Add done ticket to the schedule.
        self.add_done_ticket_to_schedule(
            lowest_id, self.done_tix[lowest_id], next_ticket_queue_id
        )

    def update_ticket_states(self) -> None:
        '''Transfers any newly ready tickets from waiting to ready.

        If a waiting ticket's parents are now both done, it will be
        transferred to the ready dictionary.
        '''
        # Check waiting_tix for new ready_tickets.
        deletion_idx = []
        for ticket_id, ticket in self.waiting_tix.items():
            num_parents = len(ticket["parents"])
            done_parents = 0
            for parent in ticket["parents"]:
                if parent in self.done_tix:
                    done_parents += 1

            if done_parents == num_parents:
                self.ready_tix[ticket_id] = self.waiting_tix[ticket_id]
                self.station_queues.queue_ticket_to_station_type(
                    ticket_id, ticket["duration"], ticket["station_type"]
                )
                deletion_idx.append(ticket_id)

        # Remove the ticket_id from deletion_idx.
        for ticket_id in deletion_idx:
            del(self.waiting_tix[ticket_id])

    def iterate(self):
        '''Iteratively steps the system forward and updates tickets.

        Until there are no more ready tickets, we alternate between
        stepping forward the ongoing tickets and updating the waiting
        tickets.
        '''
        while len(self.ready_tix):
            self.step_forward()
            self.update_ticket_states()
            self.add_queue_states_to_schedule()



    class StationQueues:
        '''Inner class that takes care of the queue information.

        Holds data and methods for keeping track of and
        modifying the queues and their lengths through the FIFO
        scheduling.
        '''
        def __init__(self, num_station_types: int, num_stations: list) -> None:

            self.num_station_types = num_station_types
            self.num_stations = num_stations
            self.tiered_station_list, self.flat_station_list = \
                self.create_unique_station_lists()
            self.queues, self.lengths = self.create_station_queues()

        def create_unique_station_lists(self):
            station_num = 0
            Mj, all_machines = [], []
            for i in range(self.num_station_types):
                stations = []
                for _ in range(1, self.num_stations[i]+1):
                    stations.append(station_num)
                    all_machines.append(station_num)
                    station_num += 1
                Mj.append(stations)
            return Mj, all_machines
        
        def create_station_queues(self) -> tuple:
            '''Queues for each station.'''
            station_queues, station_queue_lengths = [], []
            for _ in self.flat_station_list:
                station_queues.append(deque())
                station_queue_lengths.append(0)
            return station_queues, station_queue_lengths
        
        def queue_ticket_to_station_type(
            self, ticket_id: int, ticket_duration: int, station_type: int
        ):
            '''Adds the ticket to the station type with shortest queue.'''
            shortest_queue_length = 1000
            shortest_queue_id = 0
            for station_idx in self.tiered_station_list[station_type]:
                if self.lengths[station_idx] < shortest_queue_length:
                    shortest_queue_length = self.lengths[station_idx]
                    shortest_queue_id = station_idx
            
            self.queues[shortest_queue_id].append(ticket_id)
            self.lengths[shortest_queue_id] += ticket_duration

        def remove_ticket_from_queue(self, queue_id, ticket_time_left):
            '''.'''
            self.queues[queue_id].popleft()
            self.lengths[queue_id] -= ticket_time_left
                

# Test a simple case.
if __name__ == '__main__':
    from constants import tree_jobs, fifo_jobs, station_type_names,\
        station_type_numbers, num_stations

    # horizon = sum(task["duration"] for job in fifo_jobs for task in job)
    # print(f"Worst case: {horizon}.")

    # jobs = tree_jobs
    jobs = fifo_jobs

    # # Permutations of job orders.
    # indices = list(range(len(jobs)))
    # perm = permutations(indices)
    # makespans = []
    # print(len(list(perm)))
    #
    # for p in perm:
    #     input_jobs = [jobs[i] for i in p]
    #     f = FIFOScheduler(
    #         input_jobs, station_type_names, len(station_type_numbers), num_stations
    #     )
    #     f.iterate()
    #     makespans.append(f.cur_time)

    #     # print(f"Makespan: {f.cur_time}")
    #     # print(f.times)
    #     # draw_tree_schedule(f.schedule, "Images/FIFOTreeSchedule.png")
        
    f = FIFOScheduler(
        jobs, station_type_names, len(station_type_numbers), num_stations
    )
    f.iterate()
    # # print(f.schedule)
    # # print(f.queue_schedule)
    draw_queues(f.queue_schedule)
    f.schedule.to_csv(f"Plans/FIFOTreeSchedule.csv")
    draw_tree_schedule(f.schedule, "Images/FIFOTreeSchedule.png")

    # makespans = np.array(makespans)
    # counts, bins = np.histogram(makespans)
    # plt.stairs(counts, bins)
    # # plt.hist(makespans)
    # plt.show()