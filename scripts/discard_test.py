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

# ROS publishers
calibrate_trigger_pub = rospy.Publisher('Calibrate_Trigger', Bool, queue_size=1)
detect_trigger_pub = rospy.Publisher('Detect_Trigger', Bool, queue_size=1)

# pick-up area geometry [meters]
x_offset = 0.22
x_length = 0.25
x_rects = 3
x_sublength = x_length/x_rects
pickup_x_center = x_offset + x_length/2

y_length = 0.5
y_rects = 4
y_sublength = y_length/y_rects
pickup_y_center = 0

x_centers = [x_offset+0.5*x_sublength, x_offset+1.5*x_sublength, x_offset+2.5*x_sublength]
y_centers = [-1.5*y_sublength, -.5*y_sublength, .5*y_sublength, 1.5*y_sublength]

# global variables
DEF_STATUS = True
DEF_HEIGHT = 0.25  # working height
DEF_MAT = np.array( [0.14, -0.24, DEF_HEIGHT] ) # x,y,z meters
DEF_SCALE = np.array([0.14, 0.3, DEF_HEIGHT])  # x,y,z meters
DEF_CAMERA = np.array([0, -0.175, 0])  # x,y,z meters

DEF_PITCH = np.pi/2  # gripper orthogonal to ground; roll will be defined by sherd rotation anglea
DEF_NUMERICAL = False # target pose argument

DEF_SHARDS = np.array( [ [x_centers[0], y_centers[0], DEF_HEIGHT], [x_centers[0], y_centers[1], DEF_HEIGHT],
    	    	    	 [x_centers[0], y_centers[2], DEF_HEIGHT], [x_centers[0], y_centers[3], DEF_HEIGHT],
    	    	    	 [x_centers[1], y_centers[3], DEF_HEIGHT], [x_centers[1], y_centers[2], DEF_HEIGHT],
    	    	    	 [x_centers[1], y_centers[1], DEF_HEIGHT], [x_centers[1], y_centers[0], DEF_HEIGHT],
    	    	    	 [x_centers[2], y_centers[0], DEF_HEIGHT], [x_centers[2], y_centers[1], DEF_HEIGHT],
    	    	    	 [x_centers[2], y_centers[2], DEF_HEIGHT], [x_centers[2], y_centers[3], DEF_HEIGHT] ] ) # x,y,z meters
bot = Robot("locobot")
configs = bot.configs
gripper = LoCoBotGripper(configs)
#camera = SimpleCamera(configs)

class AutoCore:

    def discardFun(self):
    	print("discardFun triggered.")
    	# random points over discard area
    	random.seed()
    	discardSpot = np.array( [round(random.uniform(-x_offset-x_length, -x_offset), 4), round(random.uniform(-y_length/2, y_length/2), 4), DEF_HEIGHT] )
    	print("Generated these random x,y points over discard area: ", discardSpot)

    	print("Moving over discard area.")
    	discardLoc = {"position": discardSpot, "pitch": DEF_PITCH, "roll": 0, "numerical": DEF_NUMERICAL}    
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
      

def process_sherds():
    #rospy.init_node('process_sherds')

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
    process_sherds()

