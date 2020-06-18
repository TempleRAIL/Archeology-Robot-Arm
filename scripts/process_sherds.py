#!/usr/bin/env python

#Import Python libraries
import time
import numpy as np
from pyrobot import Robot
from pyrobot.locobot.gripper import LoCoBotGripper
from pyrobot.locobot import camera
import random
import sys

# Import ROS libraries and message types
#import message_filters
import rospy
import ros_numpy
import tf2_ros
from tf2_geometry_msgs import PointStamped
from robot_arm.srv import *

# ROS service clients
rospy.wait_for_service('color_mask')
color_mask_srv = rospy.ServiceProxy('color_mask', ColorMask)
#rospy.wait_for_service('sherd_detections')
#detection_srv = rospy.ServiceProxy('sherd_detections', SherdDetections)
rospy.wait_for_service('detect_sherds')
detection_srv = rospy.ServiceProxy('detect_sherds', SherdDetections)

bot = Robot("locobot")
configs = bot.configs
gripper = LoCoBotGripper(configs)
#camera = SimpleCamera(configs)

class AutoCore:

    DEF_STATUS = True
    color_mask = None

    # get parameters
    calibrate_location = eval(rospy.get_param('~calibrate_location'))
    pickup_area = eval(rospy.get_param('~pickup_area'))
    working_pose = eval(rospy.get_param('~working_pose'))
    numerical = rospy.get_param('~numerical')  # default boolean for using numerical method when solving IK
    scale_location = eval(rospy.get_param('~scale_location'))
    cam_location = eval(rospy.get_param('~cam_location'))
    discard_area = eval(rospy.get_param('~discard_area'))
    
    # x,y coordinates where arm will generate color mask (viewing empty mat)
    cal_x, cal_y = calibrate_location['x'], calibrate_location['y']
    
    # number of sub-rectangles w/i pickup area, in x and y directions
    pick_rects_x, pick_rects_y = pickup_area['rects_x'], pickup_area['rects_y']
    
    # x,y coordinates of midpoint of the edge of the pickup area closest to robot origin
    pick_offset_x, pick_offset_y = pickup_area['offset_x'], pickup_area['offset_y']
 
    # dimensions of rectangular pickup area
    pick_length_x, pick_length_y = pickup_area['length_x'], pickup_area['length_y']

    # working pose parameters (for when arm is just moving around, not picking or placing)
    work_height, work_roll, work_pitch = working_pose['z'], working_pose['roll'], working_pose['pitch']

    # x,y,z center coordinates of scale top face
    scale_x, scale_y, scale_z = scale_location['x'], scale_location['y'], scale_location['z']
    
    # x,y,z center coordinates of sherd position for archival imaging by camera
    cam_x, cam_y, cam_z = cam_location['x'], cam_location['y'], cam_location['z']

    # x and y limits of rectangular discard area (actual discard point will be randomly generated within limits)
    dis_min_x, dis_max_x = discard_area['min_x'], discard_area['max_x']
    dis_min_y, dis_max_y = discard_area['min_y'], discard_area['max_y']
    
    # key positions at working height
    DEF_CAL = np.array([cal_x, cal_y, work_height])
    DEF_SCALE = np.array([scale_x, scale_y, work_height])
    DEF_CAM = np.array([cam_x, cam_y, work_height])
       
    # pick-up area geometry [meters]
    rect_length_x = pick_length_x/pick_rects_x  # dimensions of rectangles w/i pickup area
    rect_length_y = pick_length_y/pick_rects_y

    # construct lists for rectangle centers w/i pickup area: one for x coords of centers, one for y coords of centers
    frac = 0.5
    pick_centers_x, pick_centers_y = [], []

    for i in range(0, pick_rects_x):
    	pick_centers_x.append(pick_offset_x + (frac+i)*rect_length_x)

    for i in range(0, pick_rects_y/2):
    	pick_centers_y.append(pick_offset_y + (frac+i)*rect_length_y)
    	pick_centers_y.append(pick_offset_y - (frac+i)*rect_length_y)  # mirror about y = 0

    pick_centers_y.sort()  # place in order
    print("pick_centers_x: ",pick_centers_x)
    print("pick_centers_y: ",pick_centers_y)

    # constsruct array of subrectangle centers
    DEF_SHARDS = []

    for x in pick_centers_x:
    	for y in pick_centers_y:
    	    center = [x, y, work_height]
    	    DEF_SHARDS.append(center)

    DEF_SHARDS = np.array(DEF_SHARDS)
    print("End-effector positions: ",DEF_SHARDS)


    # Function to trigger generation of color mask
    # Captures image of empty background mat
    def calibrateFun(self):
    	print("AutoCore.calibrateFun(self) triggered.")
    	calibrateLoc = {"position": self.DEF_CAL, "pitch": self.work_pitch, "numerical": self.numerical} 
    	self.moveFun(**calibrateLoc)
    	
    	req = ColorMaskRequest()
    	req.num_colors = 1
    	req.show_chart = True
    	
    	res = color_mask_srv(req)
    	self.color_mask = res.color_mask
    	
    	print("Got color mask.")
    	
  
    # Function to look for object in box
    # Searches until found and then retrieves
    def shardFun(self):
    	print("AutoCore.shardFun(self) triggered.")
    	loop = 0
    	status = True
    	while status:
    	    for current in range(0,len(self.DEF_SHARDS)):
    	    	print("Loop = ", loop)
    	    	heightLoc = {"position": self.DEF_SHARDS[current], "pitch": self.work_pitch, "roll": self.work_roll, "numerical": self.numerical} 
    	    	self.moveFun(**heightLoc)
    	    	time.sleep(1)

    	    	found, sherds = self.detectFun()  # check for sherd detections and get list of locations / rotations
    	    	print("Sherds were found: ", found)
    	    	if found:  # if sherds is not an empty list
    	    	    for sherd in sherds:
    	    	    	SHERD_Z = sherd[2]-0.030    # correct the target z position from top face of sherd
    	    	    	SHERD_POSITION = [ sherd[0], sherd[1], self.work_height ]
    	    	    	SHERD_ANGLE = sherd[3]
    	    	    	sherdLoc = {"position": SHERD_POSITION, "pitch": self.work_pitch, "roll": SHERD_ANGLE, "numerical": self.numerical}

    	    	    	self.pickFun(SHERD_Z, **sherdLoc)  # pick up sherd

    	    	    	scaleLoc = {"position": self.DEF_SCALE, "pitch": self.work_pitch, "roll": self.work_roll, "numerical": self.numerical}
     	    	    	self.placeFun(self.scale_z, **scaleLoc)  # move down and place sherd on scale, then move back up

    	    	    	time.sleep(1)
    	    	    	found, sherds = self.detectFun()  # re-detect sherd on scale (may have rotated/shifted during place)
    	    	    	for sherd in sherds:
    	    	    	    SHERD_Z = sherd[2]-0.03    # correct the target z position from top face of sherd
    	    	    	    SHERD_POSITION = [ sherd[0], sherd[1], 0.085 ]

    	    	    	    SHERD_ANGLE = sherd[3]
    	    	    	    sherdLoc = {"position": SHERD_POSITION, "pitch": self.work_pitch, "roll": SHERD_ANGLE, "numerical": self.numerical}
    	    	    	    self.pickFun(SHERD_Z, **sherdLoc)
    	    	    	    cameraLoc = {"position": self.DEF_CAM, "pitch": self.work_pitch, "roll": self.work_roll, "numerical": self.numerical}
     	    	    	    self.placeFun(self.cam_z, **cameraLoc)  # move down and place sherd for camera, then move back up
    	    	    	   
    	    	    	    time.sleep(1)
    	    	    	    found, sherds = self.detectFun()  # re-detect sherd (may have rotated/shifted during place)
    	    	    	    for sherd in sherds:
    	    	    	    	SHERD_Z = sherd[2]-0.03    # correct the target z position from top face of sherd
    	    	    	    	SHERD_POSITION = [ sherd[0], sherd[1], self.work_height ]  # at working height
    	    	    	    	SHERD_ANGLE = sherd[3]
    	    	    	    	sherdLoc = {"position": SHERD_POSITION, "pitch": self.work_pitch, "roll": SHERD_ANGLE, "numerical": self.numerical}
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
    	except:
    	    print("Exception to moveFun() thrown.")
    	    self.DEF_STATUS = False


    # Function to check for and return sherd detections as list of lists: [x_center, y_center, rotation_angle]
    def detectFun(self):
        # confirm that color mask exists
    	if not (self.color_mask is None):
    	    print("Color mask exists.  Proceed.")
    	else:
    	    print("Color mask does not exist.")
    	    report = False
    	    return report, sherds
  	
    	# run segment_sherds.py on what robot sees in this position
    	req = SherdDetectionsRequest()
    	req.color_mask = self.color_mask
    	
    	res = detection_srv(req)
    	detections = res.detections.detections
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
    	    	point_cam.header.frame_id = "camera_link"  # need this to prevent error "frame_ids cannot be empty"

    	    	# get center x,y,z from /Bounding_Boxes detections
    	    	point_cam.point.x, point_cam.point.y = item.bbox.center.position.x, item.bbox.center.position.y
    	    	point_cam.point.z = item.bbox.center.position.z

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
    	descend_z = np.array( [0, 0, -(self.work_height-z)] )  # z-displacement downwards to 3 cm below top face of sherd
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
    	self.moveFun(**pose)


    # Function to place an object
    def placeFun(self, z, **pose):
    	print("placeFun triggered.")

    	self.moveFun(**pose)  # position gripper at working height
    	descend_z = np.array( [0, 0, -(self.work_height-z)] )    # z-displacement downwards to z
    	bot.arm.move_ee_xyz(descend_z, plan=True)  # move gripper down near surface
    	time.sleep(2)
    	gripper.open()
    	time.sleep(1)
   	#gripper_state = gripper.get_gripper_state()
    	#print("gripper_state = ", gripper_state)
 
    	# Move gripper back up
    	self.moveFun(**pose)

        
    def discardFun(self):
    	print("discardFun triggered.")
    	# random points over discard area
    	random.seed()
    	DEF_DISCARD = np.array([random.uniform(dis_min_x, dis_max_x), random.uniform(dis_min_y, dis_max_y), work_height])
    	print("Generated these random x,y points over discard area: ", DEF_DISCARD)

    	print("Moving over discard area.")
    	discardLoc = {"position": DEF_DISCARD, "pitch": self.work_pitch, "roll": self.work_roll, "numerical": self.numerical}    
    	self.moveFun(**discardLoc)
        

def process_sherds():
    #rospy.init_node('process_sherds')

    do = AutoCore()
    print("AutoCore class instantiated.")
    bot.arm.go_home()
    print("Arm sent home.")

    while do.DEF_STATUS:
        # Calibration
    	result = do.calibrateFun()

    	# Locate Object
    	result = do.shardFun()
    	if not result:
            break

        # Capture image of object
    	do.moveFun(np.array([do.DEF_CAMERA[0], do.DEF_CAMERA[1], do.DEF_HEIGHT], do.DEF_ORIENTATION))
    	if not do.DEF_STATUS:
            break
    	do.pickPlaceFun(do.DEF_CAMERA, do.DEF_ORIENTATION, 1)
    	if not do.DEF_STATUS:
            break
    	do.pickPlaceFun(do.DEF_CAMERA, do.DEF_ORIENTATION, 0)
    	if not DEF_STATUS:
            break

        # Weigh object
    	do.moveFun(np.array([do.DEF_SCALE[0], do.DEF_SCALE[1], do.DEF_HEIGHT]), do.DEF_ORIENTATION)
    	if not DEF_STATUS:
            break
    	do.pickPlaceFun(do.DEF_SCALE, do.DEF_ORIENTATION, 1)
    	if not do.DEF_STATUS:
            break
    	do.pickPlaceFun(do.DEF_SCALE, do.DEF_ORIENTATION, 0)
    	if not do.DEF_STATUS:
            break

        # Discard object
    	do.discardFun()
    	if not do.DEF_STATUS:
            break

    bot.arm.go_home()
  
if __name__ == '__main__':
    process_sherds()

