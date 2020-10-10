# Manipulator Optimization 
This Project finds an optimal kinematics design of manipulators using [Evolution Strategies](http://www.scholarpedia.org/article/Evolution_strategies) and [Multi-Object Optimization](https://en.wikipedia.org/wiki/Multi-objective_optimization).

# Author
[Tamir Mhabary](https://www.linkedin.com/in/tamirmhabary/) Algorithm and Robotics Reasercher

## Introduction 
The code is part of my Master thesis: "AN OPTIMAL KINEMATIC DESIGN OF A MANIPULATOR FOR EARLY DETECTION OF STRESSES IN GREENHOUSE CROPS"
This thesis will try to develop a method to find an optimal manipulator based on the kinematic design to detect stresses in greenhouse crops, using a set-based concept approach based on evolutionary algorithms.

The Code is built from :
* ros.py: a file that handles all the communication between ROS, Moveit, Gazebo 
* simulator.py:  a file that handles the simulation:  which URDF's files to create, and calculate the results
* optimization.py: the main file of the project.   select which configurations to create and enter to the simulation, according to Evolution Strategies and Multi-Object Optimization methods
* other.py, hv.py, and test_mutation.py:  have helping functions and results analysis.

The ROS files are in the following link *********

## Technologies
* Python 2.7 (requirements file are added)
* Ubuntu 18.04

## Tools and methods
[Evolution Strategies](http://www.scholarpedia.org/article/Evolution_strategies) 
