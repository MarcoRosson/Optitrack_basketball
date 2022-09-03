from measure import *
import matplotlib.pyplot as plt

MAX_LENGTH = 3400

ball_6 = read_ball('Takes\Ball6.csv', 'Ball_6', MAX_LENGTH)
ball_10 = read_ball('Takes\Ball10.csv', 'Ball_10', MAX_LENGTH)
ball_14 = read_ball('Takes\Ball14.csv', 'Ball_14', MAX_LENGTH)

ball_6_inter = interpolate(ball_6) # Linear interpolation
ball_6_kal_filt = kalman_filt(ball_6_inter) # Kalman filtering
ball_6_kal_pred = kalman_pred(ball_6) # Interpolation with Kalman
ball_6_filled, missing_6 = fill_gaps(ball_6, return_missing=True) # No interpolation

ball_10_inter = interpolate(ball_10) # Linear interpolation
ball_10_kal_filt = kalman_filt(ball_10_inter) # Kalman filtering
ball_10_kal_pred = kalman_pred(ball_10) # Interpolation with Kalman
ball_10_filled, missing_10 = fill_gaps(ball_10, return_missing=True) # No interpolation

ball_14_inter = interpolate(ball_14) # Linear interpolation
ball_14_kal_filt = kalman_filt(ball_14_inter) # Kalman filtering
ball_14_kal_pred = kalman_pred(ball_14) # Interpolation with Kalman
ball_14_filled, missing_14 = fill_gaps(ball_14, return_missing=True) # No interpolation

print(f"""Missed frame for every marker combination:
6 Markers: {missing_6},
10 Markers: {missing_10},
14 Markers: {missing_14}
""")

difference_lin_kal = 0
difference_lin_kal_pred = 0
difference_kal_kal_pred = 0
for i, _ in enumerate(ball_14_inter):
    difference_lin_kal += distance_eval([ball_14_inter[i], ball_14_kal_filt[i]])
    difference_lin_kal_pred += distance_eval([ball_14_inter[i], ball_14_kal_pred[i]])
    difference_kal_kal_pred += distance_eval([ball_14_kal_pred[i], ball_14_kal_filt[i]])
difference_lin_kal = difference_lin_kal/len(ball_14_inter)
difference_lin_kal_pred = difference_lin_kal_pred/len(ball_14_inter)
difference_kal_kal_pred = difference_kal_kal_pred/len(ball_14_inter)

print(f"""Path average differences: 
Linear Interpolation/Kalman Filter: {difference_lin_kal}
Linear Interpolation/Kalman Predictor: {difference_lin_kal_pred}
Kalman Filter/Kalman Predictor: {difference_kal_kal_pred}""")

while True:
    print('0 - 3D Plot\n1 - 2D Plot')
    proj = input()
    if proj == '0':
        proj = '3d'
        break
    elif proj == '1':
        proj = None
        break

fig = plt.figure()
#ax = fig.add_subplot(211, projection='3d')
ax = plt.axes(projection=proj)
if proj == '3d':
    ax.plot(*ball_cordinates(ball_14_inter, TD=True))
    ax.plot(*ball_cordinates(ball_14_kal_filt, TD=True))
    ax.plot(*ball_cordinates(ball_14_kal_pred, TD=True))
    ax.plot(*ball_cordinates(ball_14_filled, TD=True))
else:
    ax.plot(*ball_cordinates(ball_14_inter))
    ax.plot(*ball_cordinates(ball_14_kal_filt))
    ax.plot(*ball_cordinates(ball_14_kal_pred))
    ax.plot(*ball_cordinates(ball_14_filled))

plt.legend(['Linear Interpolation', 'Linear + Kalman Filter', 'Kalman Predictor', 'No interpolation'])

plt.figure()
ax2 = plt.axes(projection=proj)
if proj == '3d':
    ax2.plot(*ball_cordinates(ball_6_inter, TD=True))
    ax2.plot(*ball_cordinates(ball_10_inter, TD=True))
    ax2.plot(*ball_cordinates(ball_14_inter, TD=True))
else:
    ax2.plot(*ball_cordinates(ball_6_inter))
    ax2.plot(*ball_cordinates(ball_10_inter))
    ax2.plot(*ball_cordinates(ball_14_inter))
plt.legend(['6 Markers', '10 Markers', '14 Markers'])
plt.show()