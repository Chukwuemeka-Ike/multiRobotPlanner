#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
'''
import rospy

from std_msgs.msg import String, UInt32
from arm_msgs.msg import IntList, Ticket, Tickets
from arm_msgs.srv import MachinesOverview, MachinesOverviewResponse,\
    MachineStatus, MachineStatusResponse, TicketList, TicketListRequest,\
    UnboundMachines, UnboundMachinesResponse

from arm_utils.conversion_utils import convert_ticket_list_to_task_dict


log_tag = "Machine Manager"


class MachineManager():
    '''Machine manager maintains the set of existing machines.

    The manager maintains machine IDs, types, and statuses, and it provides
    that information tonodes that need it.
    '''
    def __init__(self) -> None:
        '''.'''
        rospy.init_node('machine_manager')
        rospy.on_shutdown(self.shutdown_machine_manager)
        rospy.loginfo(f"{log_tag}: Node started.")

        # Machine type information parameters.
        # Names of each machine type.
        self.machine_type_names = rospy.get_param('machine_type_names')
        # Abbreviations to use when drawing schedules.
        self.machine_type_abvs = rospy.get_param('machine_type_abvs')
        # Number of each machine type.
        self.machine_type_nums = rospy.get_param('machine_type_nums')

        # # Hardcoded values for testing.
        # self.machine_type_names = [
        #     "Loading",
        #     "Mega",
        #     "Vinyl",
        #     "Hem",
        #     "Tack",
        #     "Weld",
        #     "Grommet",
        #     "Inspection",
        # ]
        # self.machine_type_abvs = ["L", "MS", "V", "H", "T", "RF", "G", "I"]
        # self.machine_type_nums = [2, 2, 2, 4, 2, 4, 4, 1]

        # print(f"Names: {self.machine_type_names}")
        # print(f"Abvs: {self.machine_type_abvs}")
        # print(f"Nums: {self.machine_type_nums}")

        # Create machine IDs based on how many there are of each type.
        # Grouped machine IDs is a list of lists where each list is the set
        # of unique IDs associated with that machine type.
        machine_id = 0
        self.machine_ids, self.grouped_machine_ids = [], []
        for i in range(len(self.machine_type_names)):
            machines = []
            for j in range(1, self.machine_type_nums[i]+1):
                machines.append(machine_id)
                self.machine_ids.append(machine_id)
                machine_id += 1
            self.grouped_machine_ids.append(machines)
        # print(f"All machines: {self.machine_ids}")

        # Variables for high level visualization. This is the old set.
        machine_type_ws_nums = {
            "Loading Area": "WS_0_",
            "Mega Stitch": "WS_1_",
            "RF": "WS_2_",
            "Perimeter": "WS_3_",
            "Inspection": "WS_4_",
        }

        # Idx of machine number of its type. For WS_0_# where # is the number of
        # that particular machine, not the unique ID.
        self.machine_type_indices = []
        for i in range(len(self.grouped_machine_ids)):
            for j in range(len(self.grouped_machine_ids[i])):
                self.machine_type_indices.append(j)

        # Dictionary to hold the status of each machine.
        # {machine_id: "busy" or "free", ...}.
        self.machine_states = {id: "free" for id in self.machine_ids}

        # Dictionary to hold each machine's assigned tickets.
        # {machine_id: [], ...}.
        self.assigned_tickets = {id: [] for id in self.machine_ids}
        self.tickets = {}

        # List to hold machines with no GUI bound to them.
        # Gives the machines that a new operator GUI can choose from.
        self.unbound_machines = self.machine_ids.copy()

        # print(f"Grouped machine IDs: {self.grouped_machine_ids}")
        # print(f"All machines: {self.machine_ids}")
        # print(f"machine_type_indices: {self.machine_type_indices}")
        # print(f"Machine states: {self.machine_states}")
        # print(f"Assigned: {self.assigned_tickets}")
        # print(f"Unbound machines: {self.unbound_machines}")
        # print(f"")

        # Subscriber for ticket list updates. Ticket Manager is the publisher.
        self.ticket_list_update_sub = rospy.Subscriber(
            'ticket_list_update', String, self.request_ticket_list
        )
        # Subscribers for starting and ending tasks.
        self.start_ticket_sub = rospy.Subscriber(
            "start_ticket", Ticket, self.start_ticket_message_callback
        )
        self.end_ticket_sub = rospy.Subscriber(
            "end_ticket", Ticket, self.end_ticket_message_callback
        )
        #  Subscriber for binding/releasing a machine to/from a GUI instance.
        self.bind_machine_sub = rospy.Subscriber(
            "bind_machine", UInt32, self.bind_machine_callback
        )
        self.release_machine_sub = rospy.Subscriber(
            "release_machine", UInt32, self.release_machine_callback
        )

        # Service for all machines.
        self.machine_overview_service = rospy.Service(
            'machine_overview_service', MachinesOverview, self.send_machine_overview
        )
        # Service for machine status and assigned tickets.
        self.machine_status_service = rospy.Service(
            'machine_status_service', MachineStatus, self.send_machine_status
        )
        # Service for unbound machines.
        self.unbound_machine_service = rospy.Service(
            'unbound_machine_service', UnboundMachines, self.send_unbound_machines
        )

        rospy.spin()

    def shutdown_machine_manager(self):
        '''Gracefully shutdown the machine manager.'''
        rospy.loginfo(f"{log_tag}: Node shutdown.")

    def send_machine_overview(self, request):
        '''Sends the list of existing machines.'''
        rospy.logdebug(f"{log_tag}: Returning machine list.")
        grouped_machine_ids = []
        for _, group in enumerate(self.grouped_machine_ids):
            group_ids = IntList()
            group_ids.int_list = group
            grouped_machine_ids.append(group_ids)

        # Return the response.
        return MachinesOverviewResponse(
            self.machine_ids,
            grouped_machine_ids,
            self.machine_type_indices,
            self.machine_type_names
        )

    def send_machine_status(self, request):
        '''Sends the status and ticket IDs assigned to a specific machine.'''
        machine_id = request.machine_id
        return MachineStatusResponse(
            self.machine_states[machine_id],
            self.assigned_tickets[machine_id]
        )

    def send_unbound_machines(self, request):
        '''Returns the IDs and names of machines that are not bound to GUIs.'''
        # Get the names of the unbound machines.
        # TODO: Double check this for clarity and robustness.
        machine_names = []
        for machine_id in self.unbound_machines:
            # Find the machine in the grouped_machine_ids.
            for machine_type_num, id_group in enumerate(self.grouped_machine_ids):
                if machine_id in id_group:
                    machine_names.append(
                        self.machine_type_names[machine_type_num] + " " +\
                        str(self.machine_type_indices[machine_id])
                    )
                    break

        return UnboundMachinesResponse(
            self.unbound_machines,
            machine_names
        )

    def request_ticket_list(self, _):
        '''Request the current ticket list from the ticket_service.'''
        rospy.wait_for_service('ticket_service')
        try:
            # TicketListRequest() is empty.
            request = TicketListRequest()
            ticket_list = rospy.ServiceProxy('ticket_service', TicketList)
            response = ticket_list(request)

            self.all_tickets = convert_ticket_list_to_task_dict(response.all_tickets)
            self.waiting = response.waiting
            self.ready = response.ready
            self.ongoing = response.ongoing
            self.done = response.done

            # Update the machine assignments.
            self.update_machine_assignments()
        except rospy.ServiceException as e:
            rospy.logerr(f'{log_tag}: Ticket list request failed: {e}.')
        except KeyError as e:
            rospy.logwarn_once(f"{log_tag}: The ticket list does not yet have "
                            f"machine assignments. {e}")

    def update_machine_assignments(self):
        '''Updates the machine assignments when the ticket list is updated.

        Old information in the machines dictionary does not need to be kept,
        since the only relevant assignments are those that are up to date with
        the ticket list. This function overwrites the ticket IDs in the dict.
        '''
        # Empty out both dictionaries.
        self.assigned_tickets = {id: [] for id in self.machine_ids}
        self.tickets = {}

        # Go through the tickets and set their assignments.
        for ticket_id, ticket in self.all_tickets.items():
            machine_id = ticket["machine_id"]
            self.assigned_tickets[machine_id].append(ticket_id)
            self.tickets[ticket_id] = machine_id

        print(self.assigned_tickets)
        print(self.tickets)

    def start_ticket_message_callback(self, msg):
        '''Sets the machine assigned to that ticket to "busy".'''
        ticket_id = msg.ticket_id

        # Set the machine to "busy".
        self.machine_states[self.tickets[ticket_id]] = "busy"
        rospy.loginfo(f"{log_tag}: Ticket {ticket_id} started. "
                      f"Machine {self.tickets[ticket_id]} set busy")

    def end_ticket_message_callback(self, msg):
        '''Sets the machine assigned to that ticket to "free".'''
        ticket_id = msg.ticket_id

        # Set the machine to "busy".
        self.machine_states[self.tickets[ticket_id]] = "free"
        rospy.loginfo(f"{log_tag}: Ticket {ticket_id} ended. "
                      f"Machine {self.tickets[ticket_id]} set free")

    def bind_machine_callback(self, msg):
        '''Removes the machine from the unbound set.'''
        machine_id = msg.data
        self.unbound_machines.remove(machine_id)
        rospy.loginfo(f"{log_tag}: Machine {machine_id} bound.")

    def release_machine_callback(self, msg):
        '''Adds the machine to the unbound set.'''
        machine_id = msg.data
        self.unbound_machines.append(machine_id)
        rospy.loginfo(f"{log_tag}: Machine {machine_id} released.")