import rospy

from arm_msgs.srv import TicketLog, TicketLogRequest
from arm_utils.conversion_utils import convert_ticket_list_to_task_dict


def request_ticket_log() -> dict:
    '''.'''
    rospy.wait_for_service('ticket_log_service')
    try:
        request = TicketLogRequest()
        ticket_log = rospy.ServiceProxy('ticket_log_service', TicketLog)
        response = ticket_log(request)

        log_dict = convert_ticket_list_to_task_dict(response.ticket_log)
        return log_dict
    except rospy.ServiceException as e:
        print(f"Ticket log request failed: {e}.")


if __name__ == "__main__":
    rospy.init_node('ticket_log_requester')
    log_dict = request_ticket_log()

    for id, ticket in log_dict.items():
        print(f"Ticket {id}:")
        print(f"{ticket}")
        print()
