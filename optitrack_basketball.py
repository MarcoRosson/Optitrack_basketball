import open3d as o3d
import time
import numpy as np
from measure import *

import optitrack.csv_reader as csv

FRAMERATE = 120

# Select take 

# filename = "Takes\Basket-Marco-Interaction.csv" ## Handling
filename = "Takes\Basket-Marco-Interaction_001.csv" ## Dribles
# filename = "Takes\Basket-Marco-Interaction_002.csv" ## Shot
# filename = "Takes\Basket-Marco-Interaction_003.csv" ## Under legs
# filename = "Takes\Basket-Marco-Interaction_004.csv" ## Too much corrupted!!

take = csv.Take().readCSV(filename)

print("Found rigid bodies:", take.rigid_bodies.keys())

body = take.rigid_bodies.copy()
ball = {'Ball': take.rigid_bodies['Ball']}
body.pop('Ball')

body_edges = [[0,1],[1,2],[2,3],[3,4],[3,5],[5,6],[6,7],[7,8],[3,9],[9,10],[10,11],[11,12],[0,13],[13,14],[14,15],
                [0,16],[16,17],[17,18],[18,20],[15,19]]

LFHAND = 8
RHAND = 12
HIP = 0

# Skeleton instantiation 
bones_pos = []
if len(body) > 0:
    for body in body: 
        bones = take.rigid_bodies[body]
        bones = interpolate(bones.positions)
        bones = kalman_filt(bones)
        #bones = kalman_pred(bones.positions)
        #bones = bones.positions
        bones_pos.append(bones)


bones_pos = np.array(bones_pos).transpose(1,0,2).tolist()

bone_joint = bones_pos[0]

# Generation of the skeleton
colors = [[1, 0, 0] for i in range(len(body_edges))]
keypoints = o3d.geometry.PointCloud()
keypoints.points = o3d.utility.Vector3dVector(bone_joint)
keypoints_center = keypoints.get_center()
keypoints.points = o3d.utility.Vector3dVector(bone_joint)
skeleton_joints = o3d.geometry.LineSet()
skeleton_joints.points = o3d.utility.Vector3dVector(bone_joint)
center_skel = skeleton_joints.get_center()
skeleton_joints.points = o3d.utility.Vector3dVector(bone_joint)
skeleton_joints.lines = o3d.utility.Vector2iVector(body_edges)
skeleton_joints.colors = o3d.utility.Vector3dVector(colors)

# Ball instantiation

ball_pos = []
if len(ball) > 0:
    for ball in ball: 
        b = take.rigid_bodies[ball]
        ball_pos = b.positions

ball_pos = np.array(ball_pos).T.tolist()

ball_pos = interpolate(ball_pos) # Linear interpolation
#ball_pos = kalman_filt(ball_pos) # Kalman filtering
#ball_pos = kalman_pred(ball_pos) # Interpolation with Kalman


ball_joint = ball_pos[0]

# Keypoint sequence for the trajectory
trajectory_edges = []

# Generation of ball and trajectory
color = [1, 0.5, 0]
ball_keypoint = o3d.geometry.PointCloud()
ball_keypoint.points = o3d.utility.Vector3dVector([ball_joint])
ball_keypoint.colors = o3d.utility.Vector3dVector([color])
ball_trajectory = o3d.geometry.LineSet()
ball_trajectory.points = o3d.utility.Vector3dVector([ball_joint])
ball_trajectory.lines = o3d.utility.Vector2iVector(trajectory_edges)
body_trajectory = o3d.geometry.LineSet()
body_trajectory.points = o3d.utility.Vector3dVector([bone_joint[HIP]])
body_trajectory.lines = o3d.utility.Vector2iVector(trajectory_edges)

vis = o3d.visualization.Visualizer()

WINDOW_WIDTH=1920 
WINDOW_HEIGHT=1080 

# Insertion of geometries in the visualizer
vis.create_window(width=WINDOW_WIDTH, height=WINDOW_HEIGHT)
vis.add_geometry(skeleton_joints)
vis.add_geometry(keypoints)
vis.add_geometry(ball_keypoint)
vis.add_geometry(ball_trajectory)
vis.add_geometry(body_trajectory)

ball = o3d.geometry.TriangleMesh.create_sphere(radius = 0.1)
translation = np.array(ball_joint, np.float64)
ball.translate(translation)
ball_color = np.array([1,0.5,0], np.float64)
ball.paint_uniform_color(ball_color)
vis.add_geometry(ball)
time.sleep(2)

frame_count = 0
missed_ball = 0
missed_body = 0
ball_update = 0
ball_traj = []
ball_traj.append(ball_joint)
body_traj = []
body_traj.append(bone_joint[HIP])
max_speed = 0
contacts = 0
touch = False
bounces = 0
ground_touch = False


for i in range(len(bones_pos)):
    #To reduce the framerate (Will affect trajectory resolution and Ball speed)
    if i%1 == 0:
        frame_count += 1
        # If the measurements are correct the model updates
        if ball_pos[i] != None:
            ball_joint = ball_pos[i]
            ball_update += 1
        else:
            missed_ball += 1
        if None not in bones_pos[i]:
            new_joints = bones_pos[i]
        else:
            missed_body += 1

        left_hand = new_joints[LFHAND]
        right_hand = new_joints[RHAND]

        left_dist = distance_eval([left_hand, ball_joint])
        right_dist = distance_eval([right_hand, ball_joint])
        if left_dist < 0.3 or right_dist < 0.3:
            if touch == False:
                contacts += 1
            touch = True
        else:
            touch = False

        if ball_joint[1] < 0.15:
            if ground_touch == False:
                bounces += 1
            ground_touch = True
        else:
            ground_touch = False

        ball_traj.append(ball_joint)
        trajectory_edges.append([frame_count-1, frame_count])
        body_traj.append(new_joints[HIP])

        # Count for corrupted measurements
        # print(f"Missed ball: {missed_ball}/{frame_count}, Missed body: {missed_body}/{frame_count}")

        # Size of the speed average window
        RESOLUTION = 10
        if frame_count % RESOLUTION == 0 and frame_count != 0:
            t = (RESOLUTION/FRAMERATE)
            distance = distance_eval(ball_traj[frame_count-RESOLUTION+1:frame_count])
            body_distance = distance_eval(body_traj[frame_count-RESOLUTION+1:frame_count])
            speed = distance/t
            body_speed = body_distance/t
            if speed > max_speed:
                max_speed = speed
            #print(f"Ball speed {speed} [m/s]")
            print(body_speed)

        skeleton_joints.points = o3d.utility.Vector3dVector(new_joints)
        keypoints.points = o3d.utility.Vector3dVector(new_joints)
        ball_keypoint.points = o3d.utility.Vector3dVector([ball_joint])
        ball_trajectory.points = o3d.utility.Vector3dVector(ball_traj)
        ball_trajectory.lines = o3d.utility.Vector2iVector(trajectory_edges)
        body_trajectory.points = o3d.utility.Vector3dVector(body_traj)
        body_trajectory.lines = o3d.utility.Vector2iVector(trajectory_edges)


        translation = np.array(ball_joint, np.float64)
        ball.translate(translation, relative = False)

        # Update of skeleton and ball
        vis.update_geometry(skeleton_joints)
        vis.update_geometry(keypoints)
        vis.update_geometry(ball_keypoint)
        vis.update_geometry(ball_trajectory)
        vis.update_geometry(body_trajectory)
        vis.update_geometry(ball)
        
        vis.update_renderer()
        vis.poll_events()

        #time.sleep(0.05)
    
vis.run()

t = (len(bones_pos)/FRAMERATE)
distance = distance_eval(ball_traj)
average_speed = distance/t
print(f"The ball traveled for {distance} [m], with and average speed of {average_speed} [m/s], and a max speed of {max_speed} [m/s]")
