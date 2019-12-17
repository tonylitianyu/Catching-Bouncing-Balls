# ECE470_FA2019
## The Repo for ECE470 Fall 2019 Project

### The goal of this project

The overall goal is to have a six-degree-of-freedom robot arm holding a basket to collect ping-pong balls shot from a machine in the V-REP simulation environment.

### Project Milestone

|#| Date          | Details     |
|-| ------------- |-------------|
|1| 09/08/2019    | Project Proposal |
|2| 09/22/2019    | Interface V-REP with Python using remote API |
|3| 10/13/2019 | Forward Kinematics + Moving Joints + Buggy Bouncing Balls      |
|4| 11/17/2019 | Inverse Kinematics + Linear Regression + Smooth Bouncing Balls      |
|5| 12/16/2019 | Project Finished with 73.3% accuracy      |

### Important File Descriptions

|#| File Name          | Description     |
|-| ------------- |-------------|
|1| main.py    | The main running script |
|2| FK_calculation.py    | Perform forward kinematics calculations |
|3| InvK.py    | Perform inverse kinematics |
|4| collectData.py | Collect Data for Linear Regression |

Other files come from the package, don't modify them for now.

### Prerequisite Installed

1. Python3
2. V-REP simulator



### Steps for running this repo

1. Download V-REP from http://www.coppeliarobotics.com/downloads.html


2. Launch V-REP

3. In V-REP, file->open scene, find this project directory, select pingpong_machine.ttt

4. The scene will load into V-REP

5. In V-REP, Click 'Run' button on toolbar, V-REP starts simulating.

6. While V-REP is simulating, in terminal, cd to this project directory

7. run python3 main.py, the robot arm will move.



### Important Message for 32-bit Windows users:

You will need to include the 32-bit lib from V-REP, here is how:

1. go to V-REP simulator directory
2. go to folder programming/remoteApiBindings/lib/lib/Windows/32Bit
3. copy the remoteApi.dll into the project directory, replace the original remoteApi.dll


### Reference

Using code from the code library accompanying Modern Robotics: Mechanics, Planning, and Control (Kevin Lynch and Frank Park, Cambridge University Press 2017)
https://github.com/NxRLab/ModernRobotics
