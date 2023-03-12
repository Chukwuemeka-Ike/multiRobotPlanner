import numpy as np

def extract_location_names(path: np.ndarray, loc_names: list):
    '''Convert path to location names.'''
    
    locations = np.empty((path.shape), dtype=np.dtype('U15'))

    for k in range(path.shape[1]):
        locations[:, k] = [loc_names[a-1] for a in path[:, k]]

    return locations