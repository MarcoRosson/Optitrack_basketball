from PIL import Image, ImageDraw, ImageFont
import open3d as o3d
import time
import numpy as np
from measure import distance_eval

import optitrack.csv_reader as csv

FRAMERATE = 120

# Select take 

# filename = "Takes\Basket-Marco-Interaction.csv" ## Handling
# filename = "Takes\Basket-Marco-Interaction_001.csv" ## Dribles
filename = "Takes\Basket-Marco-Interaction_002.csv" ## Shot
# filename = "Takes\Basket-Marco-Interaction_003.csv" ## Under legs
# filename = "Takes\Basket-Marco-Interaction_004.csv" ## Too much corrupted!!

take = csv.Take().readCSV(filename)

print("Found rigid bodies:", take.rigid_bodies.keys())

body = take.rigid_bodies.copy()

ball = {'Ball': take.rigid_bodies['Ball']}
body.pop('Ball')

body_edges = [[0,1],[1,2],[2,3],[3,4],[3,5],[5,6],[6,7],[7,8],[3,9],[9,10],[10,11],[11,12],[0,13],[13,14],[14,15],
                [0,16],[16,17],[17,18],[18,20],[15,19]]

# Skeleton instantiation 
bones_pos = []
if len(body) > 0:
    for body in body: 
        bones = take.rigid_bodies[body]
        bones_pos.append(bones.positions)

bones_pos = np.array(bones_pos).T.tolist()

# Checking for the first non-corrupted position
for i in range(100):
    if None not in bones_pos[i]:
        print(f"Body starting from position {i}")
        bone_joint = bones_pos[i]
        break

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

# Checking for the first non-corrupted position
for i in range(100):
    if ball_pos[i] != None:
        print(f"Ball starting from position {i}")
        ball_joint = ball_pos[i]
        break

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

vis = o3d.visualization.Visualizer()
label = o3d.visualization.gui.Label('We')

WINDOW_WIDTH=1920 # change this if needed
WINDOW_HEIGHT=1080 # change this if needed
img = Image.new('RGB', (WINDOW_WIDTH, WINDOW_HEIGHT), color = (255,255,255))
#fnt = ImageFont.load('arial.ttf')
d = ImageDraw.Draw(img)
d.text((1300,100), "STATUS: GOOD", fill=(0,0,0)) # puts text in upper right
img.save('pil_text.png')

# Insertion of geometries in the visualizer
vis.create_window(width=WINDOW_WIDTH, height=WINDOW_HEIGHT)
vis.add_geometry(skeleton_joints)
vis.add_geometry(keypoints)
vis.add_geometry(ball_keypoint)
vis.add_geometry(ball_trajectory)

im = o3d.io.read_image("./pil_text.png")
vis.add_geometry(im)

time.sleep(2)

frame_count = 0
missed_ball = 0
missed_body = 0
ball_update = 0
trajectory = []
trajectory.append(ball_joint)
trajectory_speed = trajectory.copy()

for i in range(len(bones_pos)):
    #To reduce the framerate
    if i%1 == 0:
        #print(i)
        frame_count += 1
        # If the measurements are correct the model updates
        if ball_pos[i] != None:
            ball_joint = ball_pos[i]
            trajectory.append(ball_joint)
            trajectory_edges.append([ball_update, ball_update+1])
            trajectory_speed.append(ball_joint)
            ball_update += 1
        else:
            missed_ball += 1
            trajectory_speed.append(ball_joint)
        if None not in bones_pos[i]:
            new_joints = bones_pos[i]
        else:
            missed_body += 1

        # Count for corrupted measurements
        #print(f"Missed ball: {missed_ball}/{frame_count}, Missed body: {missed_body}/{frame_count}")

        RESOLUTION = 10
        if frame_count % RESOLUTION == 0 and frame_count != 0:
            t = (RESOLUTION/FRAMERATE)
            distance = distance_eval(trajectory_speed[frame_count-RESOLUTION+1:frame_count])
            #print(trajectory_speed[frame_count-RESOLUTION+1:frame_count])
            speed = distance/t
            print(f"Speed {speed} [m/s]")

        center_skel = skeleton_joints.get_center()
        skeleton_joints.points = o3d.utility.Vector3dVector(new_joints)
        keypoints.points = o3d.utility.Vector3dVector(new_joints)
        ball_keypoint.points = o3d.utility.Vector3dVector([ball_joint])
        ball_trajectory.points = o3d.utility.Vector3dVector(trajectory)
        ball_trajectory.lines = o3d.utility.Vector2iVector(trajectory_edges)

        # Update of skeleton and ball
        vis.update_geometry(skeleton_joints)
        vis.update_geometry(keypoints)
        vis.update_geometry(ball_keypoint)
        vis.update_geometry(ball_trajectory)
        
        vis.update_renderer()
        vis.poll_events()

        time.sleep(0.05)
    
vis.run()

t = (len(bones_pos)/FRAMERATE)
distance = distance_eval(trajectory)
average_speed = distance/t
print(f"The ball traveled for {distance} [m], with and average speed of {average_speed} [m/s]")
