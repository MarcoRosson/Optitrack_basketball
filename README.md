# Optitrack basketball

Optitrack basketball is a python program used to plot animations of a basketball player (skeleton + ball), taken with an OptiTrack motion capture system.

The project is already provided with four takes of a basketball player doing basic movements, but other takes can be used, just by adding them to the *filename* variable (in .csv format).

The program also deals with the problem of missing frames, providing three different types of interpolation to choose from.

## Installation
 
In order to make the code work, some packages need to be installed, if not already present:

**Open3d**

```bash
pip install open3d
```

**Numpy**

```bash
pip install numpy
```

**OpenCV**

```bash
pip install opencv-python
```

The code have been tested with the following versions:

**Python**: 3.8.10

**Open3d**: 0.15.1

**Numpy**: 1.22.4

**OpenCV**: 4.6.0.66

## Usage

To use the code, run the file 'optitrack_basketball.py'. 

In the terminal, you will have to select which one of the embedded takes you want to run:

```bash
Select the take: 
0 - Ball Handling
1 - Dribbles     
2 - Shot
3 - Under The Legs
```

Then, the program will ask you wich type of interpolation you want to use: 

```bash
What kind of interpolation do you want to use?:
0 - Linear interpolation
1 - Linear interpolation+Kalman Filter
2 - Kalman predictor
3 - No interpolation
```
The animation will start in a new window.
In the terminal will be shown some information about the ball and its trajectory:

```bash
Ball speed: 0.37329354903095924 [m/s]
Body speed: 0.03739861417603988 [m/s]
Bounces: 0
Hand contacts: 2
Path differences:
Linear interpolation: 0.0
Linear interpolation+Kalman Filter: 0.36954737131135
Kalman predictor: 0.38603455939622
No interpolation: 1.12656240908354
```

## Ball analysis

The file 'ball_analysis.py' can be used to compare the behavior of the different interpolation techniques and to compare how the OptiTrack system works when different numbers of markers are applied.

When you run the code, in the terminal will be displayed the frames lost for every combination of markers:

```bash
Missed frame for every marker combination:
6 Markers: Missed ball frames: 1443/3400,
10 Markers: Missed ball frames: 1381/3400,
14 Markers: Missed ball frames: 1497/3400
```

and the average displacement with respect to every type of interpolation:

```bash
Path average differences: 
Linear Interpolation/Kalman Filter: 0.002157624622
Linear Interpolation/Kalman Predictor: 0.00182577228
Kalman Filter/Kalman Predictor: 0.0006042764587
```

Then, the program will ask the user which type of plot has to be shown. The '3D Plot' gives an overview on the trajectory, the '2D Plot' shows better the differences between the different types of trajectories.

```bash
0 - 3D Plot
1 - 2D Plot
```

## License
[MIT](https://choosealicense.com/licenses/mit/)