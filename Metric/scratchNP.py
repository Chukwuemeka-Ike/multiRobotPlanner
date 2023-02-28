import numpy as np

path = [[120, 74, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 48, 0, 96, 169, 148, 152, 152, 152, 152, 152, 150, 124, 120],
        [119, 167, 167, 167, 116, 162, 163, 167, 164, 113, 86, 182, 156, 60, 14, 37, 58, 79, 175, 175, 175, 175, 175, 104, 106, 36, 107, 111, 115, 119],
        [23, 119, 190, 139, 88, 133, 178, 127, 76, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 25, 48, 75, 30, 57, 11, 61, 65, 19, 23]]


path = np.array(path)

# print(path[:, 1])
planHorizon = path.shape[1]
trail_len = 5

for k in range(10):
    endIdx = k+1
    if k >= trail_len:
        startIdx = k+1-trail_len
        print(path[2][startIdx:endIdx])
    else:
        print(path[2][:endIdx])