import numpy as np
from numpy import float32, round
from cv2 import KalmanFilter
from traitlets import Bool
import optitrack.csv_reader_2 as csv2

def distance_eval(traj: list) -> int:
    distance = 0
    for i in range(len(traj)-1):
        x0, y0, z0 = traj[i]
        x1, y1, z1 = traj[i+1]
        distance += np.sqrt((x0-x1)**2 + (y0-y1)**2 + (z0-z1)**2)
    return distance

def path_difference(traj1: list, traj2: list) -> int:
    difference = 0
    for key, point in enumerate(traj1):
        x0, y0, z0 = traj1[key]
        x1, y1, z1 = traj2[key]
        traj_diff = np.sqrt((x0-x1)**2 + (y0-y1)**2 + (z0-z1)**2)
        difference = difference + traj_diff
    difference = difference/(distance_eval(traj1))
    return difference

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

def fill_gaps(traj: list, return_missing: Bool = False):
    """Fills the gaps in a list with None with the previous values.
    Args:
        list: the list to fill
        Bool: set to True to return missing frames
    Retruns:
        list: the filled list
        str: the number of gaps present in the original list
    """
    trajectory = traj.copy()
    filled_traj = []
    missed = 0
    total = len(trajectory)
    for i in range(100):
        if trajectory[i] != None:
            trajectory[0] = trajectory[i]
            break
        if i == 0:
            missed += 1
    for i in range(1,100):
        if trajectory[-i] != None:
            trajectory[-1] = trajectory[-i]
            break
        if i == 1:
            missed += 1
    
    for key, point in enumerate(trajectory):
        if point == None:
            trajectory[key] = trajectory[key-1]
            missed += 1
    
    missing = "Missed ball frames: " + str(missed) + "/" + str(total)
    if return_missing:
        return trajectory, missing
    return trajectory

def kalman_filt(traj: list) -> list:
    kalman = KalmanFilter(6,3)

    kalman.measurementMatrix = \
        np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0]], np.float32)
    kalman.transitionMatrix = \
        np.array([
            [1,0,0,1,0,0],
            [0,1,0,0,1,0],
            [0,0,1,0,0,1],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]], np.float32)
    kalman.processNoiseCov = \
        np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]], np.float32) * 0.003
    kalman.measurementNoiseCov = \
        np.array([
            [1,0,0],
            [0,1,0],
            [0,0,1]], np.float32) * 1

    mes = traj.copy()
    filtered_mes = []
    last_prediction = 0
    cycle = 0
    for i in mes:
        measurement = np.array([[np.float32(i[0])],[np.float32(i[1])],[np.float32(i[2])]])
        kalman.correct(measurement)
        prediction = kalman.predict()
        if cycle < 50:
            last_prediction = measurement
            cycle += 1
        filtered_mes.append([*last_prediction[0], *last_prediction[1], *last_prediction[2]])
        last_prediction = prediction
    
    return filtered_mes


def kalman_pred(traj: list) -> list:
    trajectory = traj.copy()
    for i in range(100):
        if trajectory[i] != None:
            trajectory[0] = trajectory[i]
            break
    for i in range(1,100):
        if trajectory[-i] != None:
            trajectory[-1] = trajectory[-i]
            break

    trajectory[:50] = interpolate(trajectory[:50])
    kalman = KalmanFilter(6,3)

    kalman.measurementMatrix = \
        np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0]], np.float32)
    kalman.transitionMatrix = \
        np.array([
            [1,0,0,1,0,0],
            [0,1,0,0,1,0],
            [0,0,1,0,0,1],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]], np.float32)
    kalman.processNoiseCov = \
        np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]], np.float32) *0.01 #* 0.003
    kalman.measurementNoiseCov = \
        np.array([
            [1,0,0],
            [0,1,0],
            [0,0,1]], np.float32) * 1

    filtered_mes = []
    last_pre = np.array(([trajectory[0][0]],[trajectory[0][1]],[trajectory[0][2]],[0],[0],[0]), np.float32)
    cycle = 0
    for i in trajectory:
        if i != None:
            measurement = np.array([[np.float32(i[0])],[np.float32(i[1])],[np.float32(i[2])]])
            kalman.correct(measurement)
        else:
            measurement = last_pre
            
        prediction = kalman.predict()
        if cycle < 50:
            prediction = measurement
            cycle += 1
        filtered_mes.append([*last_pre[0], *last_pre[1], *last_pre[2]])
        last_pre = prediction
    
    return filtered_mes

import optitrack.csv_reader as csv

def read_ball(path: str, name: str, MAX_LENGTH):
    ball = csv2.Take().readCSV(path)
    ball = ball.rigid_bodies[name]
    error = ball.error
    ball = ball.positions
    ball = np.array(ball).T.tolist()
    ball = ball[:MAX_LENGTH]
    error = error[:MAX_LENGTH]
    return ball, error

def read_stat(path: str, name: str, MAX_LENGTH):
    ball = csv.Take().readCSV(path)
    ball = ball.rigid_bodies[name]
    ball = ball.positions
    ball = np.array(ball).T.tolist()
    ball = ball[:MAX_LENGTH]
    return ball

def ball_cordinates(ball_traj: list, TD: bool = False):
    ball_x = []
    ball_z = []
    ball_y = []
    for row in ball_traj:
        ball_x.append(row[2])
    for row in ball_traj:
        ball_z.append(row[0])
    for row in ball_traj:
        ball_y.append(row[1])
    if TD:
        return ball_x,ball_z,ball_y

    return ball_x,ball_y