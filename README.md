# Manipulator Optimization 
This Project finds an optimal kinematics design of manipulators using [Evolution Strategies](http://www.scholarpedia.org/article/Evolution_strategies) and [Multi-Object Optimization](https://en.wikipedia.org/wiki/Multi-objective_optimization).
This thesis using the 'Set-based concept' and Dynamic Window of Interest approaches.

# Author
[Tamir Mhabary](https://www.linkedin.com/in/tamirmhabary/) Algorithm and Robotics Researcher

## Introduction 
The code is part of my Master thesis: "AN OPTIMAL KINEMATIC DESIGN OF A MANIPULATOR FOR EARLY DETECTION OF STRESSES IN GREENHOUSE CROPS".
This thesis will try to develop a method to find an optimal manipulator based on the kinematic design to detect stresses in greenhouse crops, using a set-based concept approach based on evolutionary algorithms.

The Code is built from :
* ros.py: a file that handles all the communication between ROS, Moveit, Gazebo 
* simulator.py:  a file that handles the simulation:  which URDF's files to create, and calculate the results
* optimization.py: the main file of the project.   select which configurations to create and enter to the simulation, according to Evolution Strategies and Multi-Object Optimization methods
* other.py, hv.py, and test_mutation.py:  have helping functions and results analysis.

# Notes
* The ROS files can be found [here](https://github.com/tamirmha/manipulator_ros)
* With minor changes this code can work without concepts approach
* Reach points of the simulator can be added or changed

## Technologies
* Python 2.7 (requirements file are attached)
* Ubuntu 18.04

## Tools and methods
![Algorithm](./Algorithm.png)

[Evolution Strategies](http://www.scholarpedia.org/article/Evolution_strategies) done by using a population of 1, Roulette Wheel Selection, elitism, and mutation only.

The Set-based concept approach simultaneously explores different design concepts, which are meaningful sub-sets of the entire set of solutions. The set-based concept search approach is not optimization, but gaining general knowledge of the design space. The approach to design space exploration includes predefined design concepts that are used to explore the design space at both the level of concepts and the particular designs that accompanies it.
The Window Of Interest (WOI) indicates what is considered as an acceptable performance vector. Rather than being interested in finding concepts' fronts, here the designers are interested in finding which of the considered concepts have at least one solution with a performance vector within a dynamically changed WOI. Concepts that meet this requirement are considered satisficing.

The WOI is dynamic (DWOI), meanings that WOI is updated during the processes and continues to approach the origin of axes. The evolutionary mating is performed only inside each concept, isn’t done between concepts. In concepts with a small number of configurations the selection will be random and for concepts with a large number of configurations the selection will be done by the Evolution Strategies. 

The results of this algorithm return several design concepts that can be explored more deeply.
