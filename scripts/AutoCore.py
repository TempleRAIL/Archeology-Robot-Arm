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
from gazebo_msgs.srv import GetLinkProperties, GetLinkPropertiesRequest
from robot_arm.msg import SherdData
from robot_arm.srv import *
from std_msgs.msg import String


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
        self.sim = rospy.get_param('~sim')
        self.gripper_len = rospy.get_param('~gripper_len')
        self.clearance = rospy.get_param('~clearance')
        self.sherd_allowance = rospy.get_param('~sherd_allowance')

        # Initialize other class members
        self.color_masks = {'mat': None, 'scale': None} # color masks for sherd detection
        self.bgnds = {'camera': None} # background images to subtract after color masking for sherd detections
        self.scale_ft_mode = rospy.get_param('~scale_ft_mode') # Boolean
        self.mass = None # mass of sherd on scale
        self.photo = None # archival photo image
        self.mat_z = None # average z value of mat in camera optical frame
        self.pose = None # placeholder for current location dictionary
        self.pub_status = rospy.Publisher('status', String, queue_size=100) # locobot status publisher
        
        # ROS service clients
        rospy.wait_for_service('color_mask_server')
        self.color_mask_srv = rospy.ServiceProxy('color_mask_server', ColorMask)
        rospy.wait_for_service('detect_sherds_server')
        self.detection_srv = rospy.ServiceProxy('detect_sherds_server', SherdDetections)
        rospy.wait_for_service('take_photo_server')
        self.photo_srv = rospy.ServiceProxy('take_photo_server', Photo)

        if self.scale_ft_mode:
            rospy.wait_for_service('read_scale_server')
            self.read_scale_srv = rospy.ServiceProxy('read_scale_server', ScaleReading)
        else:
            rospy.wait_for_service('id_sherd_to_weigh_server')
            self.id_sherd_srv = rospy.ServiceProxy('id_sherd_to_weigh_server', SherdID)
            rospy.wait_for_service('gazebo/get_link_properties')
            self.link_props_srv = rospy.ServiceProxy('gazebo/get_link_properties', GetLinkProperties)
        
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
    
    
    ########## pyrobot interface ##########
    # Function to publish the locobots current status
    def publish_status(self, status):
        self.pub_status.publish(status)
        rospy.loginfo("#####" + status + "#####")
    
    # Function to go home
    def go_home(self):
        self.publish_status("Initialization")
        self.bot.arm.go_home(plan=False) #plan=False means don't use MoveIt

    # Function to call IK to plot and execute trajectory
    def move_fun(self, pose, use_MoveIt=False):
        rospy.logdebug('AutoCore: move_fun triggered')
        try:
            success = self.bot.arm.set_ee_pose_pitch_roll(plan=use_MoveIt, **pose) # plan=False means don't use MoveIt; args must be in this order due to the kwarg
            if not use_MoveIt: rospy.sleep(0.5)
        except Exception:
            raise
        else:
            if not success:
                raise PlanningFailure(pose, 'AutoCore: move_fun: Planning failed')
            self.pose = pose

    def move_fun_retry(self, pose, use_MoveIt=False):
        success = False
        while not success:
            try:
                self.move_fun(pose, use_MoveIt)
            except PlanningFailure:
                continue
            except:
                raise
            else:
                success = True


    ########## Sensor interface ##########
    # Function to check for object in gripper
    def grip_check_fun(self, pose):
        self.publish_status("Grasping")
        gripper_state = self.gripper.get_gripper_state()
        if not gripper_state == 2:
            raise GraspFailure(pose, 'Gripper_state = {}'.format(gripper_state))


    # Function to record mass of object on scale and save in ROS sherd_msg
    def get_mass_fun(self, msg):
        self.publish_status("Data Collection")
        sherd_msg = msg
        # FORCE_TORQUE SCALE MODE
        if self.scale_ft_mode:
            # run read_scale_srv
            req = ScaleReadingRequest()
            try:
                res = read_scale_srv(req)
            except rospy.ServiceException as e:
                rospy.logerr('read_scale service call failed: {}'.format(e))
                raise
        # CONTACT SCALE MODE
        else:
            # run id_sherd_srv to get link name of sherd on scale
            req = SherdIDRequest()
            try:
                res = self.id_sherd_srv(req)
            except rospy.ServiceException as e:
                rospy.logerr('AutoCore: id_sherd service call failed: {}'.format(e))
                raise
            link_name = res.link_name
            rospy.logwarn('AutoCore: got link name of sherd on scale: {}'.format(link_name))
            # call link_props_srv to get mass of sherd
            req = GetLinkPropertiesRequest()
            req.link_name = link_name
            try:
                res = self.link_props_srv(req)
            except rospy.ServiceException as e:
                rospy.logerr('AutoCore: id_sherd service call failed: {}'.format(e))
                raise
        sherd_msg.mass = res.mass
        rospy.logwarn('AutoCore: got mass of sherd on scale: {}'.format(sherd_msg.mass))
        return sherd_msg

    # Function to take archival photo and save in ROS sherd_msg
    def archival_photo_fun(self, msg):
        self.publish_status("Data Collection")
        sherd_msg = msg
        rospy.logwarn('AutoCore: taking archival photo. If shown, close figure to continue. Toggle figure display in mat_layout.yaml.')
        # run take_photo_server.py
        req = PhotoRequest()
        req.which_camera = 'archival'
        try:
            res = self.photo_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr('AutoCore: take_photo service call failed: {}'.format(e))
            raise
        else:
            sherd_msg.archival_photo = res.image
            rospy.logwarn('AutoCore: Got archival photo of sherd.')
            return sherd_msg


    # Function to generate color mask from image of empty background
    def get_color_mask_fun(self, calibrate_pose, mask_type, num_colors=1):
        self.publish_status("Initialization")
        rospy.logdebug('AutoCore: get_color_mask_fun triggered.')
        # Move arm to calibration location
        try:
            self.move_fun(calibrate_pose)
        except:
            raise
        # Request data from service
        req = ColorMaskRequest()
        req.num_colors = num_colors
        req.sim = self.sim
        req.show_chart = False
        # Save results from service
        try:
            res = self.color_mask_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr('AutoCore: Color mask service call failed: {}'.format(e))
            raise
        else:
            self.color_masks[mask_type] = res.color_mask
            self.mat_z = res.mat_z
            rospy.logwarn('AutoCore: Got color masks.')
            rospy.loginfo('Average z value of mat (top face): {}'.format(self.mat_z))


    #Function to save image of empty background
    def get_background_fun(self, calibrate_pose, station):
        self.publish_status("Initialization")
        rospy.logdebug('AutoCore: get_background_fun triggered.')
        # Move arm to calibration location
        try:
            self.move_fun(calibrate_pose)
        except:
            raise
        # Request data from service
        req = PhotoRequest()
        req.which_camera = 'robot'
        # Save results from service
        try:
            res = self.photo_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr('AutoCore: take_photo service call failed: {}'.format(e))
            raise
        else:
            self.bgnds[station] = res.image
            rospy.logwarn('AutoCore: Got background at {}'.format(station))


    # Function to check for and return sherd detections as list of lists: [x_center, y_center, rotation_angle]
    def detect_fun(self, color_mask, bgnd_img=None):
        self.publish_status("Planning")
        found = False
        sherd_poses = [] # initialize empty
        # confirm that color mask exists
        if color_mask is None:
            rospy.logwarn('AutoCore: This color mask is missing.')
            return found, sherd_poses
        # run segment_sherds.py on what robot sees in this position
        req = SherdDetectionsRequest()
        req.color_mask = color_mask
        if bgnd_img:
            req.subtract_background = True
            req.background_image = bgnd_img
        else:
            req.subtract_background = False
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
    # Function to increment through survey positions in sherd pick-up area
    def shard_fun(self, pose):
        rospy.loginfo('Examining surface for sherds')
        rospy.loginfo('position: {}'.format(pose['position']))
        try:
            self.publish_status("Locomotion")
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
            self.publish_status("Grasping")
            self.gripper.open() # ensure gripper open if picking up a sherd
            try:
                pose['position'][2] += self.gripper_len # add gripper offset
                pose['position'][2] += 0.03
                self.publish_status("Locomotion")
                self.move_fun(pose) # move above sherd and orient
                pose['position'][2] -= 0.03 + self.clearance
                self.move_fun(pose, use_MoveIt=True) # move down to surface. Use MoveIt to avoid grasp plugin failure.
                self.publish_status("Grasping")
                self.gripper.close()
                self.grip_check_fun(pose) # TODO make it so arm goes back up even if gripper failure occurs
                pose['position'][2] = working_z # move back up to working height after grasping
                self.publish_status("Locomotion")
                self.move_fun(pose)
            except:
                raise
    	# PLACE MODE
        else:
            try:
                pose['position'][2] += self.gripper_len + self.clearance + self.sherd_allowance
                self.publish_status("Locomotion")
                self.move_fun(pose) # move down to surface
                self.publish_status("Grasping")
                self.gripper.open()
            except:
                raise
