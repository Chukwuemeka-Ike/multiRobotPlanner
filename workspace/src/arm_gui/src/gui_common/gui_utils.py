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
