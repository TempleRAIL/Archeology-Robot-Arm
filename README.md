# Robot Arm for Archaeology

ROS package to use a modified LoCoBot as a robot arm for archaeology.  The arm detects pottery sherds against a background of uniform color, picks them up, and places them in desired locations.  This package should be used in conjunction with the PyRobot Interbotix repo: `https://github.com/Interbotix/pyrobot`  

While this package is intended ultimately for use with the physical LoCoBot, currently it has only been developed on the LoCoBot model in **Gazebo version 7**.  Perception nodes have been written for a modified URDF which moves the Intel RealSense depth camera to the wrist of the LoCoBot.   

## Quick Start (Simulation in Gazebo)

### 1) Copy these files from the robot_arm package to these directories in the PyRobot low_cost_ws

From the command line, execute:  
 
From `~/catkin_ws/robot_arm/src/launch`:  

`cd ~/catkin_ws/src/robot_arm/launch`  
`cp archaeology.launch ~/low_cost_ws/src/pyrobot/robots/LoCoBot/locobot_control/launch`  
`cp gazebo_locobot_archaeology.launch sherd_world.launch ~/low_cost_ws/src/pyrobot/robots/LoCoBot/locobot_gazebo/launch`  

`cd ~/catkin_ws/robot_arm/src/worlds`  
`cp sherd.world ~/low_cost_ws/src/pyrobot/robots/LoCoBot/locobot_gazebo/worlds`  

`cd ~/catkin_ws/robot_arm/src/urdf`
`cp arch_locobot_description.urdf ~/low_cost_ws/src/pyrobot/robots/LoCoBot/locobot_description/urdf`  

### 2) Create SherdMat model

The custom **sherd.world** Gazebo world referenced by the launch files above contains a custom model of a blue mat.  To make it available in Gazebo, execute the following commands in the terminal:  

`cp -R ~/catkin_ws/src/robot_arm/sherdmat ~/.gazebo/models`  
`cd ~/.gazebo/models/sherdmat/meshes`  
`chmod +x SherdMat.dae`  
    
### 3) Launch the Simulation
1. In a the terminal: `roslaunch locobot_control archaeology.launch use_sim:=true use_arm:=true use:_camera:=true`  
2. In a new terminal window: `roslaunch robot_arm perception.launch`  
3. In a new terminal window: `roslaunch robot_arm arm.launch` 
