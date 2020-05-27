#!/usr/bin/env python

#Import Python libraries
import time
import math
import numpy as np
from pyrobot import Robot
from pyrobot.locobot.gripper import LoCoBotGripper
from pyrobot.locobot import camera
from numpy.random import random_integers as rand

# Import ROS libraries and message types
#import message_filters
import rospy
import ros_numpy
import tf2_ros
from tf2_geometry_msgs import PointStamped
#from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Int16MultiArray
from vision_msgs.msg import Detection2D, Detection2DArray

# ROS publishers
calibrate_trigger_pub = rospy.Publisher('Calibrate_Trigger', Bool, queue_size=1)
detect_trigger_pub = rospy.Publisher('Detect_Trigger', Bool, queue_size=1)

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

# global variables
DEF_STATUS = True
DEF_HEIGHT = 0.25
#DEF_POSITION = np.array([0.04, .18, 0]) # Placeholder
DEF_ZERO = np.array([0, 0, 0, 0, 0])  # All joints = 0 rad
DEF_SCALE = np.array([0, 0.175, .05])  # x,y,z meters
DEF_CAMERA = np.array([0, -0.175, 0])  # x,y,z meters

#DEF_ORIENTATION = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # this led to IK failure
#DEF_ORIENTATION = np.array([0, 1.5, 0])  # abandoning 'set_ee_pose' method for now - MoveIt continuously fails
DEF_PITCH = np.pi/2  # gripper orthogonal to ground; roll will be defined by sherd rotation angle
DEF_NUMERICAL = False # target pose argument


DEF_DISCARD = np.array([ -pickup_x_center, pickup_y_center, DEF_HEIGHT]) # Placeholder MAKE SURE YOU DEFINE THIS
DEF_SHARDS = np.array( [ [x_centers[0], y_centers[0], DEF_HEIGHT], [x_centers[0], y_centers[1], DEF_HEIGHT],
    	    	    	 [x_centers[0], y_centers[2], DEF_HEIGHT], [x_centers[0], y_centers[3], DEF_HEIGHT],
    	    	    	 [x_centers[1], y_centers[3], DEF_HEIGHT], [x_centers[1], y_centers[2], DEF_HEIGHT],
    	    	    	 [x_centers[1], y_centers[1], DEF_HEIGHT], [x_centers[1], y_centers[0], DEF_HEIGHT],
    	    	    	 [x_centers[2], y_centers[0], DEF_HEIGHT], [x_centers[2], y_centers[1], DEF_HEIGHT],
    	    	    	 [x_centers[2], y_centers[2], DEF_HEIGHT], [x_centers[2], y_centers[3], DEF_HEIGHT] ] ) # x,y,z meters

print("End-effector positions: ",DEF_SHARDS)

bot = Robot("locobot")


class AutoCore:

    # Function to trigger generation of color mask
    # Captures image of empty background mat
    def calibrateFun(self):
    	print("AutoCore.calibrateFun(self) triggered.")
    	calibrate_xyz = np.array( [0.14, 0.24, DEF_HEIGHT] )
    	calibrateLoc = {"position": calibrate_xyz, "pitch": DEF_PITCH, "numerical": DEF_NUMERICAL} 
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
    	    for DEF_CURRENT in range(0,12):
    	    	print("Loop = ", loop)
    	    	#heightLoc = {"position": DEF_SHARDS[DEF_CURRENT], "orientation": DEF_ORIENTATION}
    	    	heightLoc = {"position": DEF_SHARDS[DEF_CURRENT], "pitch": DEF_PITCH, "numerical": DEF_NUMERICAL} 
    	    	self.moveFun(**heightLoc)
    	    	time.sleep(1)

    	    	found, sherds = self.detectFun()  # check for sherd detections and get list of locations / rotations
    	    	print("Sherds were found: ", found)
    	    	if found:  # if sherds is not an empty list
    	    	    mode = 0  # retrieve mode

    	    	    for sherd in sherds:
    	    	    	#pos = (sherd[0], sherd[1])  # (might be redundant) Place returned center position (from DetectionArray msg)
    	    	    	#SHERD_ORIENTATION = np.array([[math.cos(angle),  math.sin(angle),      0],
    	    	    	    	    	    	#[math.sin(angle), -math.cos(angle),      0],
                                                #[              0,                0,     -1]])
    	    	    	SHERD_POSITION = np.array([sherd[0], sherd[1], 0]) # xyz center location of object
    	    	    	SHERD_ANGLE = sherd[2]
    	    	    	sherdLoc = {"position": SHERD_POSITION, "pitch": DEF_PITCH, "roll": SHERD_ANGLE, "numerical": DEF_NUMERICAL}
    	    	    	grasp_success = self.pickPlaceFun(mode, **sherdLoc)  # did gripper grasp sherd?
    	    	    	if not grasp_success:  # if not, break out of all loops and go home
    	    	    	    report = False
    	    	    	    return report

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
    	    bot.arm.set_ee_pose_pitch_roll(**pose)
    	    #bot.arm.set_ee_pose(**pose)
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
    	
    	msg = rospy.wait_for_message("/Bounding_Boxes", Detection2DArray)
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
    	    rate = rospy.Rate(10.0)
    	    print("tf_listener created.")

    	    while not rospy.is_shutdown():  # block until transform between frames becomes available
    	    	try:
    	    	    # second waitForTransform: try at time = now
    	    	    #trans = tfBuffer.lookup_transform("camera_color_optical_frame", "arm_base_link", now, rospy.Duration(4.0))
    	    	    trans = tfBuffer.lookup_transform("camera_link", "arm_base_link", rospy.Time(), rospy.Duration(4.0))
    	    	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    	    	    rate.sleep()
    	    	    raise

    	    print("Obtained transform between camera_link and arm_base_link.")

    	    #time = tf_listener.getLatestCommonTime("/arm_base_link", "/camera_color_optical_frame")
    	    for item in detections:
    	    	sherd_angle = item.bbox.center.theta  # radians
    	    	point_cam = PointStamped()  # build ROS message for conversion
    	    	point_cam.header.frame_id = "camera_link"
    	    	point_cam.point.x, point_cam.point.y, point_cam.point.z = item.bbox.center.x, item.bbox.center.y, 0
    	    	point_base = tfBuffer.transform(point_cam, "arm_base_link")
    	    	#point_base = tf_listener.transformPoint("arm_base_link", point_cam)  # convert between frames
    	    	print("Sherd center point (x,y) [m] in arm_base_link frame: ", point_base)
    	    	sherds.append( [point_base.point.x, point_base.point.y, sherd_angle] )
    	    sherds = np.array(sherds)
    	    print("sherds list = ", sherds)
    	    return report, sherds

    	    #tf_listener.waitForTransform("/arm_base_link", "/camera_color_optical_frame", rospy.Time(), rospy.Duration(4.0))
    	    #while not rospy.is_shutdown():
    	    #	try:
    	    #	    now = rospy.Time.now()
    	    #	    tf_listener.waitForTransform("/arm_base_link", "/camera_color_optical_frame", now, rospy.Duration(4.0))
    	    #	    trans, rot = tf_listener.lookupTransform("/arm_base_link", "/camera_color_optical_frame", now)
    	    #	except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    	    #	    continue


    # Function to retrieve or place an object
    # mode = 0 is retrieve, mode = 1 is place
    def pickPlaceFun(self, mode, **pose):
    	print("pickPlacefun triggered.")

    	if(mode == 0): # retrieve mode
    	    gripper.open()
    	    self.moveFun(**pose)
    	    time.sleep(2)
    	    gripper.close()  # closing around sherd
    	    time.sleep(2)
    	    gripper_state = gripper.get_gripper_state()
    	    print("gripper_state = ", gripper_state)
    	    #if gripper_state == 3:  # gripper is fully closed and failed to grasp sherd
    	    #  report = False
    	    #  return report
    	    #time.sleep(1)
    	else: # place mode
    	    gripper.open()
    	    time.sleep(1)

    	DEF_ORIENTATION = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

    	# Move gripper straight up z-axis to DEF_HEIGHT
    	displacement = np.array([0, 0, DEF_HEIGHT])
    	bot.arm.move_ee_xyz(displacement, plan=True)
    	time.sleep(5)
        
    def discardFun(self):
        self.moveFun(DEF_DISCARD, DEF_ORIENTATION)
        pos_x = rand.uniform(-x_offset, -(x_offset + x_length)) # min_x, max_x
        pos_y = rand.uniform(-y_length/2, y_length/2)  # min_y, max_y
        self.moveFun(np.array([pos_x, pos_y, DEF_HEIGHT]), DEF_ORIENTATION)
        self.pickPlaceFun(np.array([pos_x, pos_y, 0]), DEF_ORIENTATION, 1)
        
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

