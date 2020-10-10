# Manipulator Optimization 
This Project finds an optimal kinematics design of manipulators using Evolution Strategies and Multi-Object Optimization.

## Introduction 
The code is part of my Master thesis: "AN OPTIMAL KINEMATIC DESIGN OF A MANIPULATOR FOR EARLY DETECTION OF STRESSES IN GREENHOUSE CROPS"
This thesis will try to develop a method to find an optimal manipulator based on the kinematic design to detect stresses in greenhouse crops, using a set-based concept approach based on evolutionary algorithms.

The Code is built from :
1) ros.py: a file that handles all the communication between ROS, Moveit, Gazebo 
2) simulator.py:  a file that handles the simulation:  which URDF's files to create, and calculate the results
3) optimization.py: the main file of the project.   select which configurations to create and enter to the simulation, according to Evolution Strategies and Multi-Object Optimization methods
4) other.py, hv.py, and test_mutation.py:  have helping functions and results analysis.

The ROS files are in the following link *********

## Tools and methods
