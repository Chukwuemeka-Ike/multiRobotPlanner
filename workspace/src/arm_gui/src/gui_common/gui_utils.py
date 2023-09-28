#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utils for manipulating the GUIs.
'''
from PyQt5.QtWidgets import *

def enable_layout(layout):
    '''Enables all the widgets in the given layout.'''
    for i in range(layout.count()):
        item = layout.itemAt(i)
        if isinstance(item, QWidgetItem):
            widget = item.widget()
            if widget:
                widget.setEnabled(True)
        elif isinstance(item, QLayoutItem):
            sub_layout = item.layout()
            if sub_layout:
                enable_layout(sub_layout)

def disable_layout(layout):
    '''Disables all the widgets in the given layout.'''
    for i in range(layout.count()):
        item = layout.itemAt(i)
        if isinstance(item, QWidgetItem):
            widget = item.widget()
            if widget:
                widget.setEnabled(False)
        elif isinstance(item, QLayoutItem):
            sub_layout = item.layout()
            if sub_layout:
                disable_layout(sub_layout)

def clear_layout(layout):
    '''Clears the specified layout.

    Need to do so recursively to clean everything properly.
    '''
    while layout.count():
        item = layout.takeAt(0)
        widget = item.widget()
        if widget is not None:
            widget.setParent(None)
        elif isinstance(item, QLayout):
            clear_layout(item)
        else:
            layout.removeItem(item)

def lists_have_same_values(list1, list2):
    '''
    Checks if two lists have all the same values unordered.

    Args:
        list1: The first list.
        list2: The second list.

    Returns:
        True if the two lists have all the same values, False otherwise.
    '''
    set1 = set(list1)
    set2 = set(list2)

    # If the lengths are different, they can't be same.
    if len(set1) != len(set2):
        return False

    # With same lengths, iterate through one, and if it has a value not in
    # the other, they aren't same.
    for value in set1:
        if value not in set2:
            return False

    return True

def float_minutes_to_minutes_seconds(float_minutes):
    # Extract the integer part as minutes and the fractional part as seconds
    minutes = int(float_minutes)
    seconds = int((float_minutes - minutes) * 60)

    # Format as "minute:second"
    formatted_time = f"{minutes:02}:{seconds:02}"
    
    return formatted_time
