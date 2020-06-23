#!/usr/bin/env python

#Import Python libraries
import time
import numpy as np
import random

# Import pyrobot libraries
from pyrobot import Robot
from pyrobot.locobot.gripper import LoCoBotGripper
from pyrobot.locobot import camera

# Import ROS libraries and message types
import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped
from robot_arm.msg import Detection3DRPY, Detection3DRPYArray
from robot_arm.srv import *

class AutoCore():
    
    def __init__(self, bot, gripper):
        # Robot passed from state machine
        self.bot = bot
        self.gripper = gripper
        
        # Pyrobot parameters
        self.use_numerical_ik = rospy.get_param('~use_numerical_ik', False)  # default boolean for using numerical method when solving IK
        
        # Initialize standard operation parameters (when not picking/placing)
        working_pose = rospy.get_param('~working_pose')
        self.working_z, self.working_r, self.working_p = working_pose['z'], working_pose['roll'], working_pose['pitch']
        
        # Initialize calibration information
        calibrate_location = rospy.get_param('~calibrate_location')
        self.DEF_CAL = np.array([calibrate_location['x'], calibrate_location['y'], self.working_z])
        
        # Initialize pickup area
        pickup_area = rospy.get_param('~pickup_area')
        sherd_locations = []
        for i in range(0, pickup_area['rects_x']):
            x = pickup_area['offset_x'] + (i+0.5) * pickup_area['length_x'] / pickup_area['rects_x']
            y_vals = range(0, pickup_area['rects_y'])
            for j in y_vals if i % 2 == 0 else reversed(y_vals):
                y = pickup_area['offset_y'] + (j+0.5) * pickup_area['length_y'] / pickup_area['rects_y']
                sherd_locations.append([x, y, self.working_z])
        self.pickup_locations = np.array(sherd_locations)
        rospy.loginfo("Sherd locations:\n{}".format(self.pickup_locations))
        
        # Initialize scale location
        scale_location = rospy.get_param('~scale_location')
        self.scale_location = np.array([scale_location['x'], scale_location['y'], self.working_z])
        self.scale_z = scale_location['z']
        
        # Initialize camera location
        cam_location = rospy.get_param('~cam_location')
        self.camera_location = np.array([cam_location['x'], cam_location['y'], self.working_z])
        self.cam_z = cam_location['z']
        
        # Initialize discard_area
        discard_area = rospy.get_param('~discard_area')
        self.discard_offset_x, self.discard_offset_y = discard_area['offset_x'], discard_area['offset_y']
        self.discard_length_x, self.discard_length_y = discard_area['length_x'], discard_area['length_y'] # dimensions of rectangular pickup area
        
        # Initialize other class members
        self.status = True # variable used to track when errors occur
        self.color_mask = None # color mask for sherd detection
        self.location = None # placeholder for current location dictionary
        
        # ROS service clients
        rospy.wait_for_service('color_mask')
        self.color_mask_srv = rospy.ServiceProxy('color_mask', ColorMask)
        rospy.wait_for_service('detect_sherds')
        self.detection_srv = rospy.ServiceProxy('detect_sherds', SherdDetections)
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
    
    
    ########## pyrobot interface ##########
    # Function to go home
    def go_home(self):
        self.bot.arm.go_home()


    # Function to call IK to plot and execute trajectory
    def move_fun(self, pose):
        rospy.logdebug("AutoCore: move_fun triggered.")
        try:
            self.bot.arm.set_ee_pose_pitch_roll(**pose)
            self.location = pose
            time.sleep(1)
        except Exception as e:
            rospy.logwarn("AutoCore: Exception to move_fun() thrown. {}".format(e))
            self.status = False


    ########## Sensor interface ##########
    # Function to check for and return sherd detections as list of lists: [x_center, y_center, rotation_angle]
    def detect_fun(self):
        sherds = [] # initialize empty list of lists: [sherd_x, sherd_y, sherd_angle]
        # confirm that color mask exists
        if not (self.color_mask is None):
            rospy.logdebug("AutoCore: Color mask exists.")
        else:
            rospy.logwarn("AutoCore: No color mask received.")
            report = False
            return report, sherds
      
        # run segment_sherds.py on what robot sees in this position
        req = SherdDetectionsRequest()
        req.color_mask = self.color_mask
        
        res = self.detection_srv(req)
        detections = res.detections.detections
        rospy.logdebug("Bounding boxes message: {}".format(detections))

        if not detections:
            report = False
            return report, sherds
        else:
            report = True
            for item in detections:
                point_cam = PointStamped()  # build ROS message for conversion
                point_cam.header.frame_id = "camera_link"  # need this to prevent error "frame_ids cannot be empty" # TODO set up these frames as parameters
                # get center x,y,z from /Bounding_Boxes detections
                point_cam.point = item.bbox.center.position
                #point_cam.point.x, point_cam.point.y = item.bbox.center.position.x, item.bbox.center.position.y
                #point_cam.point.z = item.bbox.center.position.z
                sherd_angle = item.bbox.center.roll  # get sherd rotation angle
                try:
                    point_base = tfBuffer.transform(point_cam, "arm_base_link")
                except:  # tf2_ros.buffer_interface.TypeException as e:
                    e = sys.exc_info()[0]
                    rospy.logerr(e)
                    sys.exit(1)
                rospy.logdebug("Obtained transform between camera_link and arm_base_link.")                
                rospy.logdebug("Sherd center point (x,y,z) [m] in arm_base_link frame: ", point_base)
                sherds.append( [point_base.point.x, point_base.point.y, point_base.point.z, sherd_angle] )
            sherds = np.array(sherds)
            rospy.logdebug("sherds list = {}".format(sherds))
            return report, sherds
            
    
    ########## Motion primitives ##########
    # Function to trigger generation of color mask
    # Captures image of empty background mat
    def calibrate_fun(self):
        rospy.logdebug("AutoCore: calibrate_fun triggered.")
        # Move arm to calibration location
        calibrateLoc = {"position": self.DEF_CAL, "pitch": self.working_p, "roll": 0., "numerical": self.use_numerical_ik} 
        self.move_fun(calibrateLoc)
        # Request data from service
        req = ColorMaskRequest()
        req.num_colors = 1
        req.show_chart = False
        # Save results from service
        try:
            res = self.color_mask_srv(req)
            self.color_mask = res.color_mask
            rospy.loginfo("AutoCore: Got color mask.")
        except rospy.ServiceException as e:
            rospy.logwarn("AutoCore: Color mask service call failed: {}".format(e))
        
  
    # Function to look for object in box
    def shard_fun(self, location):
        rospy.loginfo("Examining surface for sherds")
        rospy.loginfo('position: {}'.format(self.pickup_locations[location]))
        # Move to search location
        heightLoc = {'position': self.pickup_locations[location], "pitch": self.working_p, "roll": 0., "numerical": self.use_numerical_ik}
        self.move_fun(heightLoc)
        # Check for sherd detections and get list of locations / rotations
        found, sherds = self.detect_fun()  
        if found:
            self.location['position'] = np.array(sherds[0, 0:3]) # Place returned center position (from DetectionArray msg)
            self.location['roll'] = sherds[0, 3]
        return found


    # Function to retrieve an object
    def pick_place_fun(self, pose, z, pick=False, place=False):
        rospy.logdebug("AutoCore: pickFun triggered.")
        if pick and place:
            rospy.logwarn("Only one of pick or place can be selected")
        # Ensure gripper open if picking up a sherd
        if pick:
            self.gripper.open()
        # Move to desired location
        self.move_fun(pose)  # position gripper at working height
        # Descend down to table
        descend_z = np.array( [0, 0, -(self.working_z - z)] )  # z-displacement downwards to 3 cm below top face of sherd
        self.bot.arm.move_ee_xyz(descend_z, plan=True)  # move gripper down to sherd
        time.sleep(1)
        # Toggle the gripper
        if pick:
            self.gripper.close()  # close around sherd
        elif place:
            self.gripper.open()
        else:
            rospy.logwarn("Pick or place must be selected")
        time.sleep(1)
        #gripper_state = gripper.get_gripper_state()
        #print("gripper_state = ", gripper_state)
        #if gripper_state == 3:  # '3' is even when gripper has closed around sherd, so this check does not work
            #report = False
            #return report
            #time.sleep(1)
        # Move gripper back up
        self.bot.arm.move_ee_xyz(-descend_z, plan=True)
        
    
    # Randomly draw dropoff location
    def dropoff_location(self):
        rospy.logdebug("AutoCore: Discard triggered.")
        # Generate random location in dropoff area
        discard_x = self.dicard.offset_x + random.random() * self.discard_length_x
        discard_y = self.dicard.offset_y + random.random() * self.discard_length_y
        discard_z = self.working_z
        return np.array(discard_x, discard_y, discard_z)
        
