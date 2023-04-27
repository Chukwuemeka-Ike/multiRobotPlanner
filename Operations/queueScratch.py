from collections import deque
from itertools import permutations
import pandas as pd
import random

from constants import fifo_jobs
from utils.draw_queue_utils import *



# q1 = deque()
# q2 = deque()
# q3 = deque()
# q4 = deque()

# for _ in range(10):
#     q1.append(random.randint(0, 100))
#     q2.append(random.randint(0, 100))
#     q3.append(random.randint(0, 100))
#     q4.append(random.randint(0, 100))

# print(list(q1))
# print(list(q2))
# print(list(q3))
# print(list(q4))

# queues = pd.DataFrame(columns=['T', 'q1', 'q2', 'q3', 'q4'])

# row = {"T": [0], 
#  "q1": [list(q1)],
#  "q2": [list(q2)],
#  "q3": [list(q3)],
#  "q4": [list(q4)],
# }
# # print(row)
# queues = pd.concat([queues, pd.DataFrame(row)], ignore_index=True)
# # queues['T'] = [0]
# # queues['q1'] = list(q1)
# # queues['q2'] = q2
# # queues['q3'] = q3

# # print(queues)
# # row = queues.iloc[0]
# # print(row["q1"])
# # draw_queues(queues)




# ad = pd.DataFrame(
#     columns=[
#         "t", [f"Queue {i}" for i in [0,1]]
#     ]
# )
# ad["t"] = 0
# # ad["Queue 0"] = "ter"
# # ad["Queue 1"] = 2.0

# new_col_values = [0,1]
# for i, value in enumerate(new_col_values):
#     col_name = f'Queue {i}'  # create a new column name
#     ad = ad.assign(**{col_name: value})  # use assign() to add a new column

# print(ad)




perm = permutations(fifo_jobs)
i = 0
for p in perm:
    i+=1
    if i == 3:
        [print(p[j]) for j in range(len(p))]