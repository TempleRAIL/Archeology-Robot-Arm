# Robot Arm for Archaeology

ROS package to use a modified LoCoBot as a robot arm for archaeology.  The arm detects pottery sherds against a background of uniform color, picks them up, and places them in desired locations.  This package should be used in conjunction with the PyRobot Interbotix repo: `https://github.com/Interbotix/pyrobot`  

While this package is intended ultimately for use with the physical LoCoBot, currently it has only been developed on the LoCoBot model in **Gazebo version 7**.  Perception nodes have been written for a modified URDF which moves the Intel RealSense depth camera to the wrist of the LoCoBot.   

## Quick Start (Simulation in Gazebo)

### 1) Create SherdMat model

The custom **sherd.world** Gazebo world referenced by the launch files above contains a custom model of a blue mat.  To make it available in Gazebo, execute the following commands in the terminal:  

`cp -R sherdmat ~/.gazebo/models`
`cd ~/.gazebo/models/sherdmat/meshes`
`chmod +x SherdMat.dae`
    
### 2) Launch the Simulation
1. In the terminal: `roslaunch robot_arm archaeology.launch`
2. In another terminal window: `roslaunch robot_arm arm.launch`
