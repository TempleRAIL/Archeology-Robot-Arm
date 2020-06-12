#!/usr/bin/env python

#Import Python libraries
import time
import math
import numpy as np
from pyrobot import Robot
from pyrobot.locobot.gripper import LoCoBotGripper
from pyrobot.locobot import camera
import random

# Import ROS libraries and message types
#import message_filters
import rospy
import ros_numpy
import tf2_ros
from tf2_geometry_msgs import PointStamped
#from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool, Int16MultiArray
#from vision_msgs.msg import Detection3D, Detection3DArray
from robot_arm.msg import Detection3DRPY, Detection3DArrayRPY

# global variables
DEF_STATUS = True
DEF_HEIGHT = 0.25  # working height
DEF_MAT = np.array( [0.14, -0.24, DEF_HEIGHT] ) # x,y,z meters
DEF_SCALE = np.array([0.14, 0.3, DEF_HEIGHT])  # x,y,z meters
DEF_CAMERA = np.array([0, -0.175, 0])  # x,y,z meters

DEF_PITCH = np.pi/2  # gripper orthogonal to ground; roll will be defined by sherd rotation anglea
DEF_NUMERICAL = False # target pose argument

bot = Robot("locobot")
configs = bot.configs
gripper = LoCoBotGripper(configs)
#camera = SimpleCamera(configs)

class AutoCore:

    def discardFun(self):
    	print("discardFun triggered.")
    	# random points over discard area
    	x_discard_lims = (0.09, 0.17)
    	y_discard_lims = (0.22, 0.35)
    	random.seed()
    	DEF_DISCARD = np.array( [random.uniform(x_discard_lims[0], x_discard_lims[1]), random.uniform(y_discard_lims[0], y_discard_lims[1]), DEF_HEIGHT] )
    	print("Generated these random x,y points over discard area: ", DEF_DISCARD)

    	print("Moving over discard area.")
    	discardLoc = {"position": DEF_DISCARD, "pitch": DEF_PITCH, "roll": 0, "numerical": DEF_NUMERICAL}    
    	self.moveFun(**discardLoc)

    # Function to call IK to plot and execute trajectory
    def moveFun(self, **pose):
    	print("moveFun triggered.")
    	try:
    	    bot.arm.set_ee_pose_pitch_roll(**pose)
    	    #bot.arm.set_ee_pose(**pose)
    	    time.sleep(1)
    	except:
    	    print("Exception to moveFun() thrown.")
    	    DEF_STATUS = False
      

def discard_test():
    #rospy.init_node('discard_test')

    do = AutoCore()
    print("AutoCore class instantiated.")
    bot.arm.go_home()
    print("Arm sent home.")

    while DEF_STATUS:
    	do.discardFun()
    	if not DEF_STATUS:
            break

    bot.arm.go_home()
  
if __name__ == '__main__':
    discard_test()

