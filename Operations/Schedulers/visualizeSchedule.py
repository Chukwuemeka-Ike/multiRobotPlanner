#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Draws a schedule.
'''

from utils.draw_utils import draw_tree_schedule
from utils.sched_utils import load_schedule
import json
import pandas as pd

load_dir = "/home/mekahertz/arm/Operations/catkin_ws/saves/save 9/"
schedule = load_schedule(load_dir+"savedSched.csv")
schedule["duration"] = schedule.apply(lambda x: x["end"]-x["start"], axis=1)

lowest_start = schedule["start"].min()
schedule["start"] = schedule["start"].apply(lambda x:x-lowest_start)
schedule["end"] = schedule["end"].apply(lambda x:x-lowest_start)

print(schedule)
# print(schedule.iloc[6]["parents"][0])
# print(type(schedule.iloc[6]["parents"]))
draw_tree_schedule(schedule, "dd.png")