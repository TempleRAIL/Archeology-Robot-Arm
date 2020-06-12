#!/usr/bin/env python

#Import Python libraries
import time
import math
import numpy as np
from pyrobot import Robot
#from pyrobot.locobot.gripper import LoCoBotGripper
#or from pyrobot.core import Robot
from numpy.random import random_integers as rand

# Import ROS libraries and message types
#import message_filters
import rospy
import ros_numpy
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray

# pick-up area geometry [meters]
x_offset = 0.22
x_length = 0.25
x_rects = 3
x_sublength = x_length/x_rects
pickup_x_center = (2*x_offset + x_length)/2

y_length = 0.5
y_rects = 4
y_sublength = y_length/y_rects
pickup_y_center = 0

x_centers = [x_offset+0.5*x_sublength, x_offset+1.5*x_sublength, x_offset+2.5*x_sublength]
y_centers = [-1.5*y_sublength, -.5*y_sublength, .5*y_sublength, 1.5*y_sublength]


DEF_STATUS = True
DEF_HEIGHT = 0.25
DEF_POSITION = np.array([0.04, .18, 0]) # Placeholder
DEF_ZERO = np.array([0, 0, 0, 0, 0])  # All joints = 0 rad
DEF_SCALE = np.array([0, 0.175, .05])  # x,y,z meters
DEF_CAMERA = np.array([0, -0.175, 0])  # x,y,z meters
DEF_DISCARD = np.array([ -pickup_x_center, pickup_y_center, DEF_HEIGHT]) # Placeholder MAKE SURE YOU DEFINE THIS
DEF_SHARDS = np.array( [ [x_centers[0], y_centers[0], DEF_HEIGHT], [x_centers[0], y_centers[1], DEF_HEIGHT],
    	    	    	 [x_centers[0], y_centers[2], DEF_HEIGHT], [x_centers[0], y_centers[3], DEF_HEIGHT],
    	    	    	 [x_centers[1], y_centers[3], DEF_HEIGHT], [x_centers[1], y_centers[2], DEF_HEIGHT],
    	    	    	 [x_centers[1], y_centers[1], DEF_HEIGHT], [x_centers[1], y_centers[0], DEF_HEIGHT],
    	    	    	 [x_centers[2], y_centers[0], DEF_HEIGHT], [x_centers[2], y_centers[1], DEF_HEIGHT],
    	    	    	 [x_centers[2], y_centers[2], DEF_HEIGHT], [x_centers[2], y_centers[3], DEF_HEIGHT] ] ) # x,y,z meters

print("Sub-rectangle centers: ",DEF_SHARDS)

DEF_ORIENTATION = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

bot = Robot('locobot')


class AutoCore:
   
    # Function to look for object in box
    # Searches until found and then retrieves
    def shardFun(self):
    	print("AutoCore.shardFun(self) triggered.")
    	loop = 0
    	status = True
    	while status:
    	    for DEF_CURRENT in range(0,12):
    	    	print("Loop = ", loop)
    	    	#DEF_ORIENTATION = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    	    	DEF_ORIENTATION = np.array([0, 1.5, 0])  #Euler angles in rad
    	    	heightLoc = {'position': DEF_SHARDS[DEF_CURRENT] , 'orientation': DEF_ORIENTATION} 
    	    	self.moveFun(**heightLoc)
    	    	time.sleep(1)
    	    	#found, sherds = detectFun()
    	    	found = True # Placeholder for call to check for sherd detections (False if DetectionArray is empty)
    	    	if found: # Place returned variable here
    	    	    status = False
    	    	    angle = 0.75 # Place returned angle here (from DetectionArray msg)
    	    	    pos = [0.35, 0, 0] # Place returned center position (from DetectionArray msg)
    	    	    DEF_ORIENTATION = np.array([[math.cos(angle),  math.sin(angle),      0],
    	    	    	    	    	    	[math.sin(angle), -math.cos(angle),      0],
                                                [              0,                0,     -1]])
    	    	    DEF_POSITION = np.array([pos(0), pos(1), 0]) # Place center location of object her
    	    	    self.pickPlaceFun(DEF_POSITION, DEF_ORIENTATION, 0)
    	    	loop += 1
    	    	if loop > 11:
    	    	    report = False
    	    	    print ("Returning 'False' to trigger break in process_sherds function.")
    	    	    return report
        # Once the object is secured, move to same location but at working height


    # Function to call IK to plot and execute trajectory
    def moveFun(self, **pose):
    	print("moveFun triggered.")
    	try:
            bot.arm.set_ee_pose(**pose)
            time.sleep(1)
    	except:
    	    print("Exception to moveFun() thrown.")
    	    DEF_STATUS = False

    #def callback_found(Detection2DArray)
    #	index = Detection2DArray.detections
    #	if not index:
	#    report = False
    	#    sherds = []
    	#    return report, sherds
    	#else
    	#    report = True
    	#    for i in index:
    	    	

    def detectFun(self)
    	sherds = []
    	msg = rospy.wait_for_message("/Bounding_Boxes", Detection2DArray)
    	detections = msg.detections
    	if not index:
    	    report = False
    	    return report, sherds
    	else:
    	    report = True
    	    for i in len(detections):
    	    	sherds[i] = [detections[i].bbox.center.x, detections[i].bbox.center.y, detections[i].bbox.center.theta]
    	    sherds = np.array(sherds)
    	    return report, sherds   	    


    # Function to retrieve or place an object
    # mode = 0 is retrieve, mode = 1 is place
    def pickPlaceFun(self, position, orientation, mode):
    	self.moveFun(position, orientation)
    	gripper = LocoBotGripper(bot.gripper)
    	if(mode == 0):
    	    gripper.close()  # closing around sherd
    	    time.sleep(2)
    	    gripper_state = get_gripper_state(gripper)
    	    if gripper_state == 3:  # gripper is fully closed and failed to grasp sherd
    	    	DEF_STATUS = False
    	    time.sleep(1)
    	else:
    	    gripper.open()
    	    time.sleep(1)
    	DEF_ORIENTATION = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        # Send command to close gripper
        # If closed completely, set to False
        # DEF_STATUS = False
        # Else, if it does not close completely (meaning it gripped the object), continue
        # Send the command to rise to the current location up set Z = DEF_HEIGHT
    	self.moveFun(np.array([position(0), position(1), DEF_HEIGHT]), DEF_ORIENTATION)
    	time.sleep(5)
        
    def discardFun(self):
        self.moveFun(DEF_DISCARD, DEF_ORIENTATION)
        pos_x = rand.uniform(-x_offset, -(x_offset + x_length)) # min_x, max_x
        pos_y = rand.uniform(-y_length/2, y_length/2)  # min_y, max_y
        self.moveFun(np.array([pos_x, pos_y, DEF_HEIGHT]), DEF_ORIENTATION)
        self.pickPlaceFun(np.array([pos_x, pos_y, 0]), DEF_ORIENTATION, 1)
        
def process_sherds():
    rospy.init_node('process_sherds')

    do = AutoCore()
    print("AutoCore class instantiated.")
    bot.arm.go_home()
    print("Arm sent home.")

    while DEF_STATUS:
        # Locate Object
    	result = do.shardFun()
    	if not result:
            break
        # Capture image of object
    	do.moveFun(np.array([DEF_CAMERA[0], DEF_CAMERA[1], DEF_HEIGHT], DEF_ORIENTATION))
    	if not DEF_STATUS:
            break
    	do.pickPlaceFun(DEF_CAMERA, DEF_ORIENTATION, 1)
    	if not DEF_STATUS:
            break
    	do.pickPlaceFun(DEF_CAMERA, DEF_ORIENTATION, 0)
    	if not DEF_STATUS:
            break
        # Weigh object
    	do.moveFun(np.array([DEF_SCALE[0], DEF_SCALE[1], DEF_HEIGHT]), DEF_ORIENTATION)
    	if not DEF_STATUS:
            break
    	do.pickPlaceFun(DEF_SCALE, DEF_ORIENTATION, 1)
    	if not DEF_STATUS:
            break
    	do.pickPlaceFun(DEF_SCALE, DEF_ORIENTATION, 0)
    	if not DEF_STATUS:
            break
        # Discard object
    	do.discardFun()
    	if not DEF_STATUS:
            break

    bot.arm.go_home()
  
if __name__ == '__main__':
    process_sherds()

