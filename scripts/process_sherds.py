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
SCALE_Z = 0.085
DEF_CAMERA = np.array([0, -0.175, 0])  # x,y,z meters
CAMERA_Z = SCALE_Z


random.seed()
DEF_DISCARD = np.array( [random.uniform(-x_offset-x_length, -x_offset), random.uniform(-y_length/2, y_length/2), DEF_HEIGHT] )  # random points over discard area 

DEF_PITCH = np.pi/2  # gripper orthogonal to ground; roll will be defined by sherd rotation anglea
DEF_NUMERICAL = False # required argument for bot.arm.set_ee_pose_pitch_roll

# constsruct array of subrectangle centers
DEF_SHARDS = []

for x in x_centers:
    for y in y_centers:
    	center = [x,y]
    	DEF_SHARDS.append(center)

DEF_SHARDS = np.array(DEF_SHARDS)


print("End-effector positions: ",DEF_SHARDS)

bot = Robot("locobot")
configs = bot.configs
gripper = LoCoBotGripper(configs)
#camera = SimpleCamera(configs)

class AutoCore:

    # Function to trigger generation of color mask
    # Captures image of empty background mat
    def calibrateFun(self):
    	print("AutoCore.calibrateFun(self) triggered.")
    	#calibrate_xyz = np.array( [0.14, -0.24, DEF_HEIGHT] )
    	calibrateLoc = {"position": DEF_MAT, "pitch": DEF_PITCH, "numerical": DEF_NUMERICAL} 
    	self.moveFun(**calibrateLoc)

    	generate_mask = True
    	msg = Bool()
    	msg.data = generate_mask

    	calibrate_trigger_pub.publish(msg)
    	print("/Calibrate_Trigger message published.")
    	
  
    # Function to look for object in box
    # Searches until found and then retrieves
    def shardFun(self):
    	print("AutoCore.shardFun(self) triggered.")
    	loop = 0
    	status = True
    	while status:
    	    for DEF_CURRENT in range(0,len(DEF_SHARDS)):
    	    	print("Loop = ", loop)
    	    	heightLoc = {"position": DEF_SHARDS[DEF_CURRENT], "pitch": DEF_PITCH, "roll": 0, "numerical": DEF_NUMERICAL} 
    	    	self.moveFun(**heightLoc)
    	    	time.sleep(1)

    	    	found, sherds = self.detectFun()  # check for sherd detections and get list of locations / rotations
    	    	print("Sherds were found: ", found)
    	    	if found:  # if sherds is not an empty list
    	    	    for sherd in sherds:
    	    	    	SHERD_Z = sherd[2]-0.030    # correct the target z position from top face of sherd
    	    	    	SHERD_POSITION = [ sherd[0], sherd[1], DEF_HEIGHT ]
    	    	    	SHERD_ANGLE = sherd[3]
    	    	    	sherdLoc = {"position": SHERD_POSITION, "pitch": DEF_PITCH, "roll": SHERD_ANGLE, "numerical": DEF_NUMERICAL}

    	    	    	self.pickFun(SHERD_Z, **sherdLoc)  # pick up sherd

    	    	    	scaleLoc = {"position": DEF_SCALE, "pitch": DEF_PITCH, "roll": 0, "numerical": DEF_NUMERICAL}
     	    	    	self.placeFun(SCALE_Z, **scaleLoc)  # move down and place sherd on scale, then move back up

    	    	    	time.sleep(1)
    	    	    	found, sherds = self.detectFun()  # re-detect sherd on scale (may have rotated/shifted during place)
    	    	    	for sherd in sherds:
    	    	    	    SHERD_Z = sherd[2]-0.03    # correct the target z position from top face of sherd
    	    	    	    SHERD_POSITION = [ sherd[0], sherd[1], 0.085 ]

    	    	    	    SHERD_ANGLE = sherd[3]
    	    	    	    sherdLoc = {"position": SHERD_POSITION, "pitch": DEF_PITCH, "roll": SHERD_ANGLE, "numerical": DEF_NUMERICAL}
    	    	    	    self.pickFun(SHERD_Z, **sherdLoc)
    	    	    	    cameraLoc = {"position": DEF_CAMERA, "pitch": DEF_PITCH, "roll": 0, "numerical": DEF_NUMERICAL}
     	    	    	    self.placeFun(CAMERA_Z, **cameraLoc)  # move down and place sherd for camera, then move back up
    	    	    	   
    	    	    	    time.sleep(1)
    	    	    	    found, sherds = self.detectFun()  # re-detect sherd (may have rotated/shifted during place)
    	    	    	    for sherd in sherds:
    	    	    	    	SHERD_Z = sherd[2]-0.03    # correct the target z position from top face of sherd
    	    	    	    	SHERD_POSITION = [ sherd[0], sherd[1], DEF_HEIGHT ]  # at working height
    	    	    	    	SHERD_ANGLE = sherd[3]
    	    	    	    	sherdLoc = {"position": SHERD_POSITION, "pitch": DEF_PITCH, "roll": SHERD_ANGLE, "numerical": DEF_NUMERICAL}
    	    	    	    	self.pickFun(SHERD_Z, **sherdLoc)
    	    	    	    	self.discardFun()
    	    	    	    	
    	    	loop += 1
    	    	if loop > 11:
    	    	    report = False
    	    	    print ("Returning 'False' to trigger break in process_sherds function.")
    	    	    return report


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


    # Function to check for and return sherd detections as list of lists: [x_center, y_center, rotation_angle]
    def detectFun(self):

    	# confirm that color mask exists
    	Color_Mask_msg = rospy.wait_for_message("/Color_Mask", Int16MultiArray)
    	if Color_Mask_msg.data:
    	    print("Color mask exists.  Proceed.")
    	else:
    	    print("Color mask does not exist.")
    	    report = False
    	    return report, sherds
  	
    	# run segment_sherds.py on what robot sees in this position
    	detect_sherd = True
    	msg = Bool()
    	msg.data = detect_sherd
    	detect_trigger_pub.publish(msg)
    	print("/Detect_Trigger message published.")
    	
    	msg = rospy.wait_for_message("/Bounding_Boxes", Detection3DArrayRPY)
    	detections = msg.detections
    	print("/Bounding_Boxes detections message: ", detections)

    	sherds = [] # initialize empty list of lists: [sherd_x, sherd_y, sherd_angle]

    	if not detections:
    	    report = False
    	    return report, sherds
    	else:
    	    report = True
    	    tfBuffer = tf2_ros.Buffer()
    	    tf_listener = tf2_ros.TransformListener(tfBuffer)
    	    rate = rospy.Rate(5.0)
    	    print("tf_listener created.")

    	    rospy.sleep(1.0)

   	    for item in detections:
    	    	point_cam = PointStamped()  # build ROS message for conversion
    	    	point_cam.header.frame_id = "camera_link"

    	    	# get center x,y,z from /Bounding_Boxes detections
    	    	point_cam.point.x, point_cam.point.y, point_cam.point.z = item.bbox.center.position.x, item.bbox.center.position.y, 0

    	    	sherd_angle = item.bbox.center.roll  # get sherd rotation angle

    	    	try:
    	    	    point_base = tfBuffer.transform(point_cam, "arm_base_link")
    	    	except:  # tf2_ros.buffer_interface.TypeException as e:
    	    	    e = sys.exc_info()[0]
    	    	    rospy.logerr(e)
    	    	    sys.exit(1)
    	    	print("Obtained transform between camera_link and arm_base_link.")    	    	
    	    	print("Sherd center point (x,y,z) [m] in arm_base_link frame: ", point_base)
    	    	sherds.append( [point_base.point.x, point_base.point.y, point_base.point.z, sherd_angle] )
    	    sherds = np.array(sherds)
    	    print("sherds list = ", sherds)
    	    return report, sherds


    # Function to retrieve an object
    def pickFun(self, z, **pose):
    	print("pickFun triggered.")
    	gripper.open()
    	self.moveFun(**pose)  # position gripper at working height
    	descend_z = np.array( [0, 0, -(DEF_HEIGHT-z)] )  # z-displacement downwards to 3 cm below top face of sherd
    	bot.arm.move_ee_xyz(descend_z, plan=True)  # move gripper down to sherd
    	time.sleep(1)
    	gripper.close()  # close around sherd
    	#gripper_state = gripper.get_gripper_state()
    	#print("gripper_state = ", gripper_state)
    	#if gripper_state == 3:  # '3' is even when gripper has closed around sherd, so this check does not work
    	    #report = False
    	    #return report
    	    #time.sleep(1)

    	# Move gripper back up
    	bot.arm.move_ee_xyz(-descend_z, plan=True)


    # Function to place an object
    def placeFun(self, z, **pose):
    	print("placeFun triggered.")

    	self.moveFun(**pose)  # position gripper at working height
    	descend_z = np.array( [0, 0, -(DEF_HEIGHT-z)] )    # z-displacement downwards to z
    	bot.arm.move_ee_xyz(descend_z, plan=True)  # move gripper down near surface
    	time.sleep(2)
    	gripper.open()
    	time.sleep(1)
   	#gripper_state = gripper.get_gripper_state()
    	#print("gripper_state = ", gripper_state)
 
    	# Move gripper back up
    	bot.arm.move_ee_xyz(-descend_z, plan=True)

        
    def discardFun(self):
    	print("Moving over discard area.")
    	discardLoc = {"position": DEF_DISCARD, "pitch": DEF_PITCH, "roll": 0, "numerical": DEF_NUMERICAL}    
    	self.moveFun(**discardLoc)
        

def process_sherds():
    #rospy.init_node('process_sherds')

    do = AutoCore()
    print("AutoCore class instantiated.")
    bot.arm.go_home()
    print("Arm sent home.")

    while DEF_STATUS:
        # Calibration
    	result = do.calibrateFun()

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

