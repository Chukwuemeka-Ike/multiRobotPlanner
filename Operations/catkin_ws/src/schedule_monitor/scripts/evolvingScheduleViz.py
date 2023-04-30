import glob
import json
import pandas as pd
from utils.draw_utils import draw_evolving_schedule, draw_tree_schedule
from utils.sched_utils import load_schedule

search_dir = '/home/mekahertz/arm/Operations/catkin_ws/saves/save 7'
search_word = 'schedule'

draw_evolving_schedule(search_dir)
schedule = load_schedule(f"{search_dir}/savedSched.csv")
lowest_start = schedule["start"].min()
schedule["start"] = schedule["start"].apply(lambda x:x-lowest_start)
schedule["end"] = schedule["end"].apply(lambda x:x-lowest_start)
schedule["duration"] = schedule.apply(lambda x: x["end"]-x["start"], axis=1)

# print(schedule)
draw_tree_schedule(schedule, "dd.png")