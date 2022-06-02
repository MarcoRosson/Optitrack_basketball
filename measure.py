import numpy as np
def distance_eval(traj: list) -> int:
    distance = 0
    for i in range(len(traj)-1):
        x0, y0, z0 = traj[i]
        x1, y1, z1 = traj[i+1]
        distance += np.sqrt((x0-x1)**2 + (y0-y1)**2 + (z0-z1)**2)
    return distance

