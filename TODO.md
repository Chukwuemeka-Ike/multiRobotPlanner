# TODO
Holds TODO's especially when I don't yet know the best place the functionalities should be put.

1. Disable edits to done tickets from edit dialogs - doesn't make sense.
2. Prevent adding a ticket with its own ID as a parent. Prevent user from entering a parent that's not in ticket_dict or done.
3. Change all nodes and utils to use machines from ROS parameters instead of constants file.
4. Shutdown behaviors of the scheduler, ticket manager, GUIs, etc.
5. Make a decision on deleting tickets. Currently leaning towards not letting it happen. Just don't put too many tickets in. You can always put too few in.
6. Decide whether the number of robots can be changed.