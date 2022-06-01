import open3d as o3d
import time
import numpy as np

import optitrack.csv_reader as csv

filename = "Takes\Basket-Marco-Interaction_001.csv"

take = csv.Take().readCSV(filename)

print("Found rigid bodies:", take.rigid_bodies.keys())

body = take.rigid_bodies.copy()

ball = {'Ball': take.rigid_bodies['Ball']}
body.pop('Ball')

xaxis = [1,0,0]
yaxis = [0,1,0]

body_edges = [[0,1],[1,2],[2,3],[3,4],[3,5],[5,6],[6,7],[7,8],[3,9],[9,10],[10,11],[11,12],[0,13],[13,14],[14,15],
                [0,16],[16,17],[17,18],[18,20],[15,19]]

# Skeleton instantiation 
bones_pos = []
if len(body) > 0:
    for body in body: 
        bones = take.rigid_bodies[body]
        bones_pos.append(bones.positions)

bones_pos = np.array(bones_pos).T.tolist()
colors = [[1, 0, 0] for i in range(len(body_edges))]
keypoints = o3d.geometry.PointCloud()
keypoints.points = o3d.utility.Vector3dVector(bones_pos[0])
keypoints_center = keypoints.get_center()
keypoints.points = o3d.utility.Vector3dVector(bones_pos[0])
skeleton_joints = o3d.geometry.LineSet()
skeleton_joints.points = o3d.utility.Vector3dVector(bones_pos[0])
center_skel = skeleton_joints.get_center()
skeleton_joints.points = o3d.utility.Vector3dVector(bones_pos[0])
skeleton_joints.lines = o3d.utility.Vector2iVector(body_edges)
skeleton_joints.colors = o3d.utility.Vector3dVector(colors)

# Ball instantiation

ball_pos = []
if len(ball) > 0:
    for ball in ball: 
        b = take.rigid_bodies[ball]
        #print('ball position', b.positions)
        ball_pos.append(b.positions)

ball_pos = np.array(ball_pos).T.tolist()
color = [1, 0, 0]
ball_keypoint = o3d.geometry.PointCloud()
ball_keypoint.points = o3d.utility.Vector3dVector(ball_pos[0])



vis = o3d.visualization.Visualizer()

# Generation of skeleton and ball
vis.create_window()
vis.add_geometry(skeleton_joints)
vis.add_geometry(keypoints)
vis.add_geometry(ball_keypoint)

time.sleep(2)

ball_joint = ball_pos[0]
for i in range(0,len(bones_pos)):
    #To reduce the framerate
    if i%20 == 0:
        print(i)
        # If the measurements are correct the model updates
        if ball_pos[i] != [None]:
            ball_joint = ball_pos[i]
            print('ball update')
        new_joints = bones_pos[i]

        #print(new_joints, 'ball', ball_joint)
        center_skel = skeleton_joints.get_center()
        skeleton_joints.points = o3d.utility.Vector3dVector(new_joints)
        keypoints.points = o3d.utility.Vector3dVector(new_joints)
        ball_keypoint.points = o3d.utility.Vector3dVector(ball_joint)

        # Update of skeleton and ball
        vis.update_geometry(skeleton_joints)
        vis.update_geometry(keypoints)
        vis.update_geometry(ball_keypoint)
        
        vis.update_renderer()
        vis.poll_events()

        time.sleep(0.2)
    
vis.run()

