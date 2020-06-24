ROS package to use a modified LoCoBot as a robot arm for archaeology.  Detect pottery sherds through color masking, pick up, place, and discard.  This package should be used in conjunction with the PyRobot Interbotix repo: https://github.com/Interbotix/pyrobot

Quick Start:

1) Copy these files to these directories:

	~/catkin_ws/src/robot_arm/worlds/sherd.world --> ~/low_cost_ws/src/pyrobot/robots/LoCoBot/locobot_gazebo/worlds
	~/catkin_ws/src/robot_arm/worlds/SherdMat.dae --> ~TO BE CONTINUED
	archaeology.launch --> ~/low_cost_ws/src/pyrobot/robots/LoCoBot/locobot_control/launch
	gazebo_locobot_archaeology.launch --> ~/low_cost_ws/src/pyrobot/robots/LoCoBot/locobot_gazebo/launch
	arch_locobot_description.urdf --> ~/low_cost_ws/src/pyrobot/robots/LoCoBot/locobot_description/urdf
	

2) In a new terminal: roslaunch locobot_control archaeology.launch use_sim:=true use_arm:=true use:_camera:=true
3) In a new terminal: roslaunch robot_arm perception.launch
4) In a new terminal: roslaunch robot_arm arm.launch
