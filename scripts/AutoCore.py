#!/usr/bin/env python

#Import Python libraries
import time
import numpy as np

# Import pyrobot libraries
from pyrobot import Robot
from pyrobot.locobot.gripper import LoCoBotGripper
from pyrobot.locobot import camera

# Import ROS libraries and message types
import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped
from robot_arm.msg import SherdData
from robot_arm.srv import *


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
        self.gripper_len = rospy.get_param('~gripper_len')
        self.clearance = rospy.get_param('~clearance')
        self.sherd_allowance = rospy.get_param('~sherd_allowance')

        # Initialize other class members
        self.color_masks = {'mat': None, 'scale': None} # color masks for sherd detection
        self.mass = None # mass of sherd on scale
        self.photo = None # archival photo image
        self.mat_z = None # average z value of mat in camera optical frame
        self.pose = None # placeholder for current location dictionary
        
        # ROS service clients
        rospy.wait_for_service('color_mask_server')
        self.color_mask_srv = rospy.ServiceProxy('color_mask_server', ColorMask)
        rospy.wait_for_service('detect_sherds_server')
        self.detection_srv = rospy.ServiceProxy('detect_sherds_server', SherdDetections)
        rospy.wait_for_service('read_scale_server')
        self.scale_srv = rospy.ServiceProxy('read_scale_server', ScaleReading)
        rospy.wait_for_service('take_photo_server')
        self.photo_srv = rospy.ServiceProxy('take_photo_server', Photo)
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
    
    
    ########## pyrobot interface ##########
    # Function to go home
    def go_home(self):
        self.bot.arm.go_home(plan=True)


    # Function to call IK to plot and execute trajectory
    def move_fun(self, pose):
        rospy.logdebug('AutoCore: move_fun triggered')
        try:
            success = self.bot.arm.set_ee_pose_pitch_roll(**pose)
        except Exception:
            raise
        else:
            if not success:
                raise PlanningFailure(pose, 'AutoCore: move_fun: Planning failed')
            self.pose = pose
            time.sleep(1)

    def move_fun_retry(self, pose):
        success = False
        while not success:
            try:
                self.move_fun(pose)
            except PlanningFailure:
                continue
            except:
                raise
            else:
                success = True


    ########## Sensor interface ##########
    # Function to check for object in gripper
    def grip_check_fun(self, pose):
        gripper_state = self.gripper.get_gripper_state()
        if not gripper_state == 2:
            raise GraspFailure(pose, 'Gripper_state = {}'.format(gripper_state))


    # Function to record mass of object on scale
    def get_mass_fun(self, msg):
        sherd_msg = msg
        # run read_scale_server.py
        req = ScaleReadingRequest()
        try:
            res = self.scale_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr('AutoCore: read_scale service call failed: {}'.format(e))
            raise
        else:
            sherd_msg.mass = res.mass
            rospy.logwarn('AutoCore: Got sherd mass: {} kg'.format(sherd_msg.mass))
            return sherd_msg


    # Function to take archival photo
    def take_photo_fun(self, msg):
        sherd_msg = msg
        rospy.logwarn('AutoCore: taking archival photo. If shown, close figure to continue. Toggle figure display in mat_layout.yaml.')
        # run take_photo_server.py
        req = PhotoRequest()
        try:
            res = self.photo_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr('AutoCore: take_photo service call failed: {}'.format(e))
            raise
        else:
            sherd_msg.archival_photo = res.image
            rospy.logwarn('AutoCore: Got archival photo of sherd.')
            return sherd_msg

    # Function to check for and return sherd detections as list of lists: [x_center, y_center, rotation_angle]
    def detect_fun(self, color_mask, num_colors=1):
        found = False
        sherd_poses = [] # initialize empty
        # confirm that color mask exists
        if color_mask is None:
            rospy.logwarn('AutoCore: This color mask is missing.')
            return found, sherd_poses
        # run segment_sherds.py on what robot sees in this position
        req = SherdDetectionsRequest()
        req.color_mask = color_mask
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
            # TODO return (first or random or smallest etc.) sherd from list
            for item in detections:
                point_cam = PointStamped()  # build ROS message for conversion
                point_cam.header = item.header
                point_cam.point.x, point_cam.point.y, point_cam.point.z = item.bbox.center.position.x, item.bbox.center.position.y, item.bbox.center.position.z
                try:
                    point_base = self.tfBuffer.transform(point_cam, 'base_link')
                except tf2_ros.buffer_interface.TypeException as e:
                    rospy.logerr('AutoCore: tf failure: {}'.format(e))
                    raise
                rospy.logdebug('Obtained transform between camera_link and base_link.')
                rospy.logdebug('Sherd center point (x,y,z) [m] in base_link frame: {}'.format(point_base))
                sherd_poses.append({
                    'position': np.array([point_base.point.x, point_base.point.y, point_base.point.z]),
                    'roll': item.bbox.center.roll
                })
            rospy.logdebug('sherds list = {}'.format(sherd_poses))
        return found, sherd_poses


    ########## Motion primitives ##########
    # Function to trigger generation of color mask
    # Captures image of empty background mat
    def calibrate_fun(self, calibrate_pose, location):
        #TODO logic check that only one station (mat, scale, etc.) is True
        rospy.logdebug('AutoCore: calibrate_fun triggered.')
        # Move arm to calibration location
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
            self.color_masks[location] = res.color_mask
            self.mat_z = res.mat_z
            rospy.logwarn('AutoCore: Got color masks.')
            rospy.loginfo('Average z value of mat (top face): {}'.format(self.mat_z))


    # Function to increment through survey positions in sherd pick-up area
    def shard_fun(self, pose):
        rospy.loginfo('Examining surface for sherds')
        rospy.loginfo('position: {}'.format(pose['position']))
        try:
            self.move_fun(pose)
        except:
            raise
        # Check for sherd detections and get list of locations / rotations
        try:
            return self.detect_fun(self.color_masks['mat'])
        except:
            raise


    # Function to retrieve an object
    def pick_place_fun(self, pose, working_z, pick=False, place=False):
        rospy.logdebug('AutoCore: pickFun triggered.')
        if pick and place:
            rospy.logerr('Only one of pick or place can be selected')
        elif not (pick or place):
            rospy.logerr('Pick or place must be selected')
        # PICK MODE
        if pick:
            self.gripper.open() # ensure gripper open if picking up a sherd
            try:
                pose['position'][2] += self.gripper_len # add gripper offset
                pose['position'][2] += 0.05
                self.move_fun(pose) # move above sherd and orient
                pose['position'][2] -= 0.05 + self.clearance
                self.move_fun(pose) # move down to surface
                self.gripper.close()
                self.grip_check_fun(pose) # TODO make it so arm goes back up even if gripper failure occurs
                pose['position'][2] = working_z # move back up to working height after grasping
                self.move_fun(pose)
            except:
                raise
    	# PLACE MODE
        else:
            try:
                pose['position'][2] += self.gripper_len + self.clearance + self.sherd_allowance
                self.move_fun(pose) # move down to surface
                self.gripper.open()
            except:
                raise
