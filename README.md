# Optitrack basketball

Optitrack basketball is a python program used to plot animations of a basketball player (skeleton + ball), taken with an OptiTrack motion capture system.

The project is already provided with four takes of a basketball player doing basic movements, but other takes can be used, just adding them to the *filename* variable (in .csv format).

## Installation
 
In order to make the code work, some pakages need to be installed, if not already present:

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

In the terminal, you have to select wich one of the embedded takes you want to run:

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
0 Linear interpolation
1 Linear interpolation+Kalman Filter
2 Kalman predictor
3 No interpolation
```
The animation would start in a new window.
In the terminal wil be shown some information about the ball and its trajectory:

```bash
Ball speed: 0.37329354903095924 [m/s]
Body speed: 0.03739861417603988 [m/s]
Bounces: 0
Hand contacts: 2
Path differences:
Linear interpolation: 0.0
Linear interpolation+Kalman Filter: 0.36954737131135595
Kalman predictor: 0.3860345593962202
No interpolation: 1.1265624090835493
```

