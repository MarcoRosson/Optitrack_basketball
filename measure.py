import numpy as np
from numpy import round

def distance_eval(traj: list) -> int:
    distance = 0
    for i in range(len(traj)-1):
        x0, y0, z0 = traj[i]
        x1, y1, z1 = traj[i+1]
        distance += np.sqrt((x0-x1)**2 + (y0-y1)**2 + (z0-z1)**2)
    return distance

def interpolate(traj: list) -> list:
    trajectory = traj.copy()
    for i in range(100):
        if trajectory[i] != None:
            trajectory[0] = trajectory[i]
            break
    for i in range(1,100):
        if trajectory[-i] != None:
            trajectory[-1] = trajectory[-i]
            break
    interpolated_traj = []

    missing = 0
    for key, pos in enumerate(trajectory):
        if pos != None:
            interpolated_traj.append(pos)
            if missing != 0:
                end = pos
                dx = (end[0] - start[0])/(missing + 1)
                dy = (end[1] - start[1])/(missing + 1)
                dz = (end[2] - start[2])/(missing + 1)
                for i in range(0, missing):
                    interpolated_traj[i+start_key][0] = round(start[0] + dx*(i+1), 6)
                    interpolated_traj[i+start_key][1] = round(start[1] + dy*(i+1), 6)
                    interpolated_traj[i+start_key][2] = round(start[2] + dz*(i+1), 6)
                missing = 0

        else:
            if missing == 0:
                start = interpolated_traj[key-1]
                start_key = key
            missing += 1
            interpolated_traj.append([0, 0, 0])
        
    return interpolated_traj
