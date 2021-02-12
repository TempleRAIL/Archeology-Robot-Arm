#!/usr/bin/env python

#Import Python libraries
import time
import numpy as np
import copy

# Import pyrobot libraries
from pyrobot import Robot
from pyrobot.locobot.gripper import LoCoBotGripper
from pyrobot.locobot import camera

# Import ROS libraries and message types
import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped
from gazebo_msgs.srv import GetLinkProperties, GetLinkPropertiesRequest
from rascal_msgs.msg import SherdData
from rascal_msgs.srv import *
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

class RascalControl():
    
    def __init__(self, bot, gripper):
        # Robot passed from state machine
        self.bot = bot
        self.gripper = gripper

        # Pyrobot parameters
        self.gripper_len = rospy.get_param('~gripper_len')
        self.orient_z = rospy.get_param('~orient_z')
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

        # ROS publishers
        self.pub_status = rospy.Publisher('status', String, queue_size=100) # locobot status publisher
        
        # ROS service clients
        rospy.wait_for_service('color_mask')
        self.color_mask_srv = rospy.ServiceProxy('color_mask', ColorMask)
        rospy.wait_for_service('detect_sherds')
        self.detection_srv = rospy.ServiceProxy('detect_sherds', SherdDetections)
        rospy.wait_for_service('take_photo')
        self.photo_srv = rospy.ServiceProxy('take_photo', Photo)

        if self.scale_ft_mode:
            rospy.wait_for_service('read_scale')
            self.read_scale_srv = rospy.ServiceProxy('read_scale', ScaleReading)
        else:
            rospy.wait_for_service('sherd_mass')
            self.id_sherd_srv = rospy.ServiceProxy('sherd_mass', SherdID)
            rospy.wait_for_service('gazebo/get_link_properties')
            self.link_props_srv = rospy.ServiceProxy('gazebo/get_link_properties', GetLinkProperties)
        
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
    
    
    ########## pyrobot interface ##########
    # Function to publish the locobots current status
    def publish_status(self, status):
        self.pub_status.publish(status)
        rospy.loginfo("#####" + status + "#####")
    
    def cart_to_cyl(self, pose):
        r = np.sqrt(pose[0] ** 2 + pose[1] ** 2)
        theta = np.arctan2(pose[1], pose[0])
        return np.array([r, theta, pose[2]])
        
    def cyl_to_cart(self, pose):
        x = pose[0] * np.cos(pose[1])
        y = pose[0] * np.sin(pose[1])
        return np.array([x, y, pose[2]])
        
    # Function to go home
    def go_home(self):
        self.publish_status("Initialization")
        self.bot.arm.go_home(plan=False) #plan=False means don't use MoveIt
        time.sleep(2)
         
        #Set Initial Pose
        pose = self.bot.arm.pose_ee # https://github.com/facebookresearch/pyrobot/blob/master/src/pyrobot/core.py#L519
        self.pose = {"position": np.zeros(3), "pitch": 0.0, "roll": 0.0, "numerical": False}
        self.pose['position'][0] = pose[0][0]
        self.pose['position'][1] = pose[0][1]
        self.pose['position'][2] = pose[0][2]
        self.pose['roll'] = 0.0 # TODO set the angles, but need to convert quaternion to Euler angles https://github.com/facebookresearch/pyrobot/blob/master/src/pyrobot/utils/util.py#L111
        self.pose['pitch'] = 0.0
        self.pose['numerical'] = False
        
    # Function to call IK to plot and execute trajectory
    def move_fun(self, goal, use_MoveIt=False):
        rospy.logdebug('RascalControl: move_fun triggered')
        d_min = 0.05
        success = True
        # Convert to Cylindrical Coordinates
        start = self.cart_to_cyl(self.pose['position'])
        end = self.cart_to_cyl(goal['position'])
        # Compute number of steps
        dis = np.linalg.norm(goal['position'] - self.pose['position'])
        n_steps = np.ceil(dis/d_min).astype(int)
        if n_steps >= 1:
            step = (end - start) / n_steps
            step_roll = (goal['roll'] - self.pose['roll']) / n_steps
            step_pitch = (goal['pitch'] - self.pose['pitch']) / n_steps
            # Initialize location
            current = copy.deepcopy(self.pose)
            current_cyl = start
            try:
                for i in range(n_steps):
                    current_cyl += step
                    current['position'] = self.cyl_to_cart(current_cyl)
                    current['roll'] += step_roll
                    current['pitch'] += step_pitch
                    success = self.bot.arm.set_ee_pose_pitch_roll(plan=use_MoveIt, **current) # args must be in this order due to the kwarg
                    if not success:
                        raise PlanningFailure(current, 'RascalControl: move_fun: Planning failed')
            except Exception as e:
                rospy.logwarn("RascalControl move_fun exception: {} Why is this empty?".format(e))
            else:
                if not use_MoveIt: 
                    rospy.sleep(0.5) # pause for accurate calibration images
                self.pose = goal

        if not success or n_steps < 1:
            try:
                self.pose = None
                #plan=False means don't use MoveIt
                success = self.bot.arm.set_ee_pose_pitch_roll(plan=use_MoveIt, **goal) # args must be in this order due to the kwarg
                self.pose = goal
                if not use_MoveIt: rospy.sleep(0.5) # pause for accurate calibration images
            except Exception as e:
                rospy.logwarn("RascalControl move_fun: failed due to {}".format(e))
                raise
            else:
                if not success:
                    raise PlanningFailure(goal, 'RascalControl: move_fun: Planning failed')


    def move_fun_retry(self, pose, use_MoveIt=False):
        success = False
        while not success:
            try:
                self.move_fun(pose, use_MoveIt)
            except PlanningFailure:
                continue
            #except rospy.ROSInterruptException:
                #raise
            except Exception as e:
                rospy.logwarn("RascalControl move_fun_retry: failed due to {}".format(e))
                raise
            else:
                success = True


    ########## Sensor interface ##########
    # Function to check for object in gripper
    def grip_check_fun(self, pose):
        self.gripper.close()
        rospy.sleep(1.5) # wait for /gripper/state topic to update
        gripper_state = self.gripper.get_gripper_state()
        rospy.logwarn('Gripper_state = {}'.format(gripper_state))
        if not gripper_state == 2:
            raise GraspFailure(pose, 'Gripper_state = {}'.format(gripper_state))


    # Function to record mass of object on scale and save in ROS sherd_msg
    def get_mass_fun(self, msg):
        self.publish_status("Data Collection")
        sherd_msg = msg
        # FORCE_TORQUE MODE
        if self.scale_ft_mode:
            # run read_scale_srv
            req = ScaleReadingRequest()
            try:
                res = read_scale_srv(req)
            except rospy.ServiceException as e:
                rospy.logerr('read_scale service call failed: {}'.format(e))
                raise
        # CONTACT MODE
        else:
            # run id_sherd_srv to get link name of sherd on scale
            req = SherdIDRequest()
            try:
                res = self.id_sherd_srv(req)
            except rospy.ServiceException as e:
                rospy.logerr('RascalControl: id_sherd service call failed: {}'.format(e))
                raise
            link_name = res.link_name
            sherd_msg.which_sherd = link_name
            rospy.logwarn('RascalControl: got link name of sherd on scale: {}'.format(link_name))
            # call link_props_srv to get mass of sherd
            req = GetLinkPropertiesRequest()
            req.link_name = link_name
            try:
                res = self.link_props_srv(req)
            except rospy.ServiceException as e:
                rospy.logerr('RascalControl: id_sherd service call failed: {}'.format(e))
                raise
        sherd_msg.mass = res.mass
        rospy.logwarn('RascalControl: got mass of sherd on scale: {}'.format(sherd_msg.mass))
        return sherd_msg

    # Function to take archival photo and save in ROS sherd_msg
    def archival_photo_fun(self, msg):
        self.publish_status("Data Collection")
        sherd_msg = msg
        rospy.logwarn('RascalControl: taking archival photo. If shown, close figure to continue. Toggle figure display in mat_layout.yaml.')
        # run take_photo_server.py
        req = PhotoRequest()
        req.camera_type = PhotoRequest.ARCHIVAL
        try:
            res = self.photo_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr('RascalControl: take_photo service call failed: {}'.format(e))
            raise
        else:
            sherd_msg.archival_photo = res.image
            rospy.logwarn('RascalControl: Got archival photo of sherd.')
            return sherd_msg


    # Function to generate color mask from image of empty background
    def get_color_mask_fun(self, calibrate_pose, mask_type, num_colors=1):
        self.publish_status("Initialization")
        rospy.logdebug('RascalControl: get_color_mask_fun triggered.')
        # Move arm to calibration location
        try:
            self.move_fun_retry(calibrate_pose)
        except:
            raise
        # Request data from service
        req = ColorMaskRequest()
        req.num_colors = num_colors
        # Save results from service
        try:
            res = self.color_mask_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr('RascalControl: Color mask service call failed: {}'.format(e))
            raise
        else:
            self.color_masks[mask_type] = res.color_mask
            self.mat_z = res.mat_z
            rospy.logwarn('RascalControl: Got color mask(s).')
            rospy.loginfo('RascalControl z value of mat (top face): {}'.format(self.mat_z))


    #Function to save image of empty background
    def get_background_fun(self, calibrate_pose, station):
        self.publish_status("Initialization")
        rospy.logdebug('RascalControl: get_background_fun triggered.')
        # Move arm to calibration location
        try:
            self.move_fun_retry(calibrate_pose)
        except:
            raise
        # Request data from service
        req = PhotoRequest()
        req.camera_type = PhotoRequest.WRIST
        # Save results from service
        try:
            res = self.photo_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr('RascalControl: take_photo service call failed: {}'.format(e))
            raise
        else:
            self.bgnds[station] = res.image
            rospy.logwarn('RascalControl: Got background at {}'.format(station))


    # Function to check for and return sherd detections as list of lists: [x_center, y_center, rotation_angle]
    def detect_fun(self, color_mask, bgnd_img=None):
        found = False
        sherd_poses = [] # initialize empty
        # confirm that color mask exists
        if color_mask is None:
            rospy.logwarn('RascalControl: This color mask is missing.')
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
            rospy.logerr('RascalControl: Bounding box service call failed: {}'.format(e))
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
                    rospy.logerr('RascalControl: tf failure: {}'.format(e))
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
            self.publish_status("Grasping")
            self.move_fun_retry(pose)
        except:
            raise
        # Check for sherd detections and get list of locations / rotations
        try:
            return self.detect_fun(self.color_masks['mat'])
        except:
            raise


    # Function to retrieve an object
    def pick_place_fun(self, pose, working_z, pick=False, place=False):
        rospy.logdebug('RascalControl: pickFun triggered.')
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
                pose['position'][2] += self.orient_z
                self.move_fun_retry(pose) # move above sherd and orient
                pose['position'][2] -= self.orient_z + self.clearance
                self.move_fun_retry(pose, use_MoveIt=True) # move down to surface. Use MoveIt to avoid grasp plugin failure.
                self.gripper.close()
                pose['position'][2] = working_z # move back up to working height after grasping
                self.publish_status("Locomotion")
                self.move_fun_retry(pose, use_MoveIt=True) # Use MoveIt to prevent swinging at wrist
                self.grip_check_fun(pose)
            except:
                raise
        # PLACE MODE
        else:
            self.grip_check_fun(pose)
            try:
                pose['position'][2] += self.gripper_len + self.clearance + self.sherd_allowance
                self.publish_status("Locomotion")
                self.move_fun_retry(pose) # move down to surface
                self.gripper.open()
            except Exception as e:
                rospy.logwarn('RascalControl: failed to descend to placement height due to {}'.format(e))
                raise
