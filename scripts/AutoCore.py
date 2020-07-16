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
from robot_arm.srv import ColorMask, ColorMaskRequest, SherdDetections, SherdDetectionsRequest


class PlanningFailure(Exception):
    """
    Exception raised for errors in movement

    Attributes:
        goal_pose -- where the robot was trying to go
        message -- explanation of the error
    """

    def __init__(self, goal_pose, message):
        self.goal_pose = goal_pose
        self.message = message


class GraspFailure(Exception):
    """
    Exception raised when gripper does not grasp object

    Attributes:
        goal_pose -- where the robot was trying to go
        message -- explanation of the error
    """

    def __init__(self, goal_pose, message):
        self.goal_pose = goal_pose
        self.message = message


class AutoCore():
    
    def __init__(self, bot, gripper):
        # Robot passed from state machine
        self.bot = bot
        self.gripper = gripper
 
        # Pyrobot parameters
        self.use_numerical_ik = rospy.get_param('~use_numerical_ik', False)  # default boolean for using numerical method when solving IK
        self.gripper_len = rospy.get_param('~gripper_len')
        self.clearance = rospy.get_param('~clearance')
        
        # Initialize standard operation parameters (when not picking/placing)
        working_pose = rospy.get_param('~working_pose')
        self.working_z, self.working_r, self.working_p = working_pose['z'], working_pose['roll'], working_pose['pitch']

        # Initialize calibration information
        calibrate_location = rospy.get_param('~calibrate_location')
        self.calibrate_position = np.array([calibrate_location['x'], calibrate_location['y'], self.working_z])
        
        # Initialize pickup area
        pickup_positions = []
        pickup_area = rospy.get_param('~pickup_area')
        for i in range(0, pickup_area['rects_x']):
            x = pickup_area['offset_x'] + (i+0.5) * pickup_area['length_x'] / pickup_area['rects_x']
            y_vals = range(-pickup_area['rects_y']/2, pickup_area['rects_y']/2)
            for j in y_vals if i % 2 == 0 else reversed(y_vals):
                y = pickup_area['offset_y'] + (j+0.5) * pickup_area['length_y'] / pickup_area['rects_y']
                pickup_positions.append([x, y, self.working_z])
        self.pickup_positions = np.array(pickup_positions)
        rospy.loginfo('Pickup locations:\n{}'.format(self.pickup_positions))
        
        # Initialize scale location
        scale_location = rospy.get_param('~scale_location')
        self.scale_position = np.array([scale_location['x'], scale_location['y'], scale_location['z']])
        
        # Initialize camera location
        cam_location = rospy.get_param('~cam_location')
        self.camera_position = np.array([cam_location['x'], cam_location['y'], cam_location['z']])
        
        # Initialize discard_area
        discard_area = rospy.get_param('~discard_area')
        self.discard_z = discard_area['z']
        self.discard_offset_x, self.discard_offset_y = discard_area['offset_x'], discard_area['offset_y']
        self.discard_length_x, self.discard_length_y = discard_area['length_x'], discard_area['length_y'] # dimensions of rectangular pickup area
        
        # Initialize other class members
        self.color_mask = None # color mask for sherd detection
        self.mat_z = None # average z value of mat in camera optical frame
        self.pose = None # placeholder for current location dictionary
        
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
        self.bot.arm.go_home(plan=True)

    # Function to call IK to plot and execute trajectory
    def move_fun(self, pose):
        #rospy.logdebug('AutoCore: move_fun triggered.')
        rospy.loginfo('AutoCore: move_fun triggered.')
        try:
            success = self.bot.arm.set_ee_pose_pitch_roll(**pose)
        except Exception:
            raise
        else:
            if not success:
                raise PlanningFailure(pose, 'AutoCore: move_fun: Planning failed')
            self.pose = pose
            time.sleep(1)


    ########## Sensor interface ##########
    # Function to check for and return sherd detections as list of lists: [x_center, y_center, rotation_angle]
    def detect_fun(self):
        found = False
        sherds = None # initialize empty list of lists: [sherd_x, sherd_y, sherd_z, sherd_angle]
        # confirm that color mask exists
        if self.color_mask is None:
            rospy.logwarn('AutoCore: No color mask received.')
            return found, sherds
      
        # run segment_sherds.py on what robot sees in this position
        req = SherdDetectionsRequest()
        req.color_mask = self.color_mask
        try:
            res = self.detection_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr('AutoCore: Bounding box service call failed: {}'.format(e))
            raise
        else:
            detections = res.detections.detections
            rospy.logdebug('Bounding boxes message: {}'.format(detections))
        
        if detections:
            found = True
            sherds = []
            for item in detections:
                #rospy.logwarn('item = {}'.format(item))
                point_cam = PointStamped()  # build ROS message for conversion
                point_cam.header = item.header
                point_cam.point.x, point_cam.point.y, point_cam.point.z = item.bbox.center.position.x, item.bbox.center.position.y, item.bbox.center.position.z
                #rospy.logwarn('point_cam = {}'.format(point_cam))
                try:
                    point_base = self.tfBuffer.transform(point_cam, 'base_link')
                except tf2_ros.buffer_interface.TypeException as e:
                    rospy.logerr('AutoCore: tf failure: {}'.format(e))
                    raise
                #rospy.logwarn('point_base = {}'.format(point_base))
                rospy.logdebug('Obtained transform between camera_link and base_link.')                
                rospy.logdebug('Sherd center point (x,y,z) [m] in base_link frame: {}'.format(point_base))
                sherd_angle = item.bbox.center.roll  # get sherd rotation angle
                sherds.append( [point_base.point.x, point_base.point.y, point_base.point.z, sherd_angle] )
            sherds = np.array(sherds)
            rospy.logdebug('sherds list = {}'.format(sherds))
        return found, sherds
        
    
    ########## Motion primitives ##########
    # Function to trigger generation of color mask
    # Captures image of empty background mat
    def calibrate_fun(self):
        rospy.logdebug('AutoCore: calibrate_fun triggered.')
        # Move arm to calibration location
        calibrate_pose = {'position': self.calibrate_position, 'pitch': self.working_p, 'roll': self.working_r, 'numerical': self.use_numerical_ik}
        try:
            self.move_fun(calibrate_pose)
        except:
            raise
        # Request data from service
        req = ColorMaskRequest()
        req.num_colors = 1
        req.show_chart = False
        # Save results from service
        try:
            res = self.color_mask_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr('AutoCore: Color mask service call failed: {}'.format(e))
            raise
        else:
            self.color_mask = res.color_mask
            self.mat_z = res.mat_z
            rospy.loginfo('AutoCore: Got color mask.')
            rospy.logwarn('Average z value of mat (top face): {}'.format(self.mat_z))
        
  
    # Function to look for object in box
    def shard_fun(self, index):
        rospy.loginfo('Examining surface for sherds')
        rospy.loginfo('position: {}'.format(self.pickup_positions[index]))
        # Move to search location
        search_pose = {'position': self.pickup_positions[index], 'pitch': self.working_p, 'roll': self.working_r, 'numerical': self.use_numerical_ik}
        self.move_fun(search_pose)
        # Check for sherd detections and get list of locations / rotations
        try:
            found, sherds = self.detect_fun()
        except:
            raise
        sherd_pose = None
        if found:
            sherd_x, sherd_y = sherds[0,0], sherds[0,1]
            targ_z = sherds[0,2] + self.gripper_len + self.clearance
            sherd_roll = sherds[0,3]
            #sherd_pose = {'position': np.array(sherds[0, 0:3]), 'roll': sherds[0, 3], 'pitch': self.working_p, 'numerical': self.use_numerical_ik}
            sherd_pose = {'position': np.array([sherd_x, sherd_y, targ_z]), 'roll': sherd_roll, 'pitch': self.working_p, 'numerical': self.use_numerical_ik}
            rospy.logwarn('pose = {}'.format(self.pose))
            rospy.logwarn('sherd_pose = {}'.format(sherd_pose))
        return found, sherd_pose


    # Function to retrieve an object
    def pick_place_fun(self, pose, pick=False, place=False):
        rospy.logdebug('AutoCore: pickFun triggered.')
        if pick and place:
            rospy.logerr('Only one of pick or place can be selected')
        elif not (pick or place):
            rospy.logerr('Pick or place must be selected')
        # Get gripper ready
        if pick:
            self.gripper.open() # ensure gripper open if picking up a sherd
        elif place:
            self.gripper.open()
        # Move gripper
        try:
            pose['position'][2] += 0.05
            self.move_fun(pose) # move above sherd and orient
            pose['position'][2] -= 0.05
            self.move_fun(pose) # move down to table
        except:
            raise
        time.sleep(1)
        #Finish gripper motion
        if pick:
            self.gripper.close()
            gripper_state = self.gripper.get_gripper_state()
            #if not gripper_state == 2:  # commented out for now because LoCoBot never publishes '2' to /gripper/state topic
                #raise GraspFailure(pose, 'Gripper_state = {}'.format(gripper_state))
        elif place:
            self.gripper.open()
        # Move gripper back up
        pose['position'][2] = self.working_z
        rospy.logwarn('Moving back up to {}'.format(self.pose))
        try:
            self.move_fun(pose) # move down to table
        except:
            raise

    
    # Randomly draw dropoff location
    def dropoff_position(self):
        rospy.logdebug('AutoCore: Discard triggered.')
        # Generate random location in dropoff area
        discard_x = self.discard_offset_x + random.random() * self.discard_length_x
        discard_y = self.discard_offset_y + random.random() * self.discard_length_y
        dropoff_pos = np.array([discard_x, discard_y, self.discard_z])
        rospy.logwarn('Dropff position: {}'.format(dropoff_pos))
        return dropoff_pos
