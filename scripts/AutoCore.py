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
        self.pickup_area_z = pickup_area['z']
        num_steps_x = np.ceil(pickup_area['length_x'] / pickup_area['step_size'])
        num_steps_y = np.ceil(pickup_area['length_y'] / pickup_area['step_size'])
        for i in range(0, int(num_steps_x)):
            x = pickup_area['offset_x'] + (i+0.5) * pickup_area['length_x'] / num_steps_x
            y_vals = range(0, int(num_steps_y))
            for j in (y_vals if i % 2 == 0 else reversed(y_vals)):
                y = pickup_area['offset_y'] + (j+0.5) * pickup_area['length_y'] / num_steps_y
                pickup_positions.append([x, y, self.pickup_area_z])
        self.pickup_positions = np.array(pickup_positions)
        #rospy.loginfo('Pickup locations:\n{}'.format(self.pickup_positions))
        
        # Initialize scale location
        scale_location = rospy.get_param('~scale_location')
        self.scale_position = np.array([scale_location['x'], scale_location['y'], self.working_z])
        self.scale_z = scale_location['z']
        
        # Initialize camera location
        cam_location = rospy.get_param('~cam_location')
        self.camera_position = np.array([cam_location['x'], cam_location['y'], self.working_z])
        self.camera_z = cam_location['z']

        # Initialize standby location (out of archival camera frame)
        standby_location = rospy.get_param('~standby_location')
        self.standby_position = np.array([standby_location['x'], standby_location['y'], self.working_z])
        self.standby_pose = {"position": self.standby_position, "pitch": self.working_p, "roll": self.working_r, "numerical": self.use_numerical_ik}
        
        # Initialize discard_area
        discard_area = rospy.get_param('~discard_area')
        self.discard_z = discard_area['z']
        self.discard_offset_x, self.discard_offset_y = discard_area['offset_x'], discard_area['offset_y']
        self.discard_length_x, self.discard_length_y = discard_area['length_x'], discard_area['length_y'] # dimensions of rectangular pickup area
        
        # Initialize other class members
        self.color_mask = None # color mask for sherd detection
        self.mat_z = None # average z value of mat in camera optical frame
        self.pose = None # placeholder for current location dictionary
        self.table_z = 0. # height of table
        
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


    ########## Sensor interface ##########
    # Function to check for object in gripper
    def grip_check_fun(self, pose):
        gripper_state = self.gripper.get_gripper_state()
        if not gripper_state == 2:
            raise GraspFailure(pose, 'Gripper_state = {}'.format(gripper_state))   

    # Function to check for and return sherd detections as list of lists: [x_center, y_center, rotation_angle]
    def detect_fun(self):
        found = False
        sherd_poses = [] # initialize empty
        # confirm that color mask exists
        if self.color_mask is None:
            rospy.logwarn('AutoCore: No color mask received.')
            return found, sherd_poses
      
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
                    'position': np.array([point_base.point.x, point_base.point.y, point_base.point.z + self.gripper_len + self.clearance]), 
                    'roll': item.bbox.center.roll, 
                    'pitch': self.working_p, 
                    'numerical': self.use_numerical_ik
                })
            rospy.logdebug('sherds list = {}'.format(sherd_poses))
        return found, sherd_poses
        
    
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
        try:
            self.move_fun(search_pose)
        except:
            raise
        # Check for sherd detections and get list of locations / rotations
        try:
            return self.detect_fun()
        except:
            raise


    # Function to retrieve an object
    def pick_place_fun(self, pose, pick=False, place=False):
        rospy.logdebug('AutoCore: pickFun triggered.')
        if pick and place:
            rospy.logerr('Only one of pick or place can be selected')
        elif not (pick or place):
            rospy.logerr('Pick or place must be selected')
        # PICK MODE
        if pick:
            self.gripper.open() # ensure gripper open if picking up a sherd
        # Move gripper to station
        pose['position'][2] += self.gripper_len # add gripper offset
        try:
            pose['position'][2] += 0.05
            self.move_fun(pose) # move above sherd and orient
            pose['position'][2] -= 0.05 + self.clearance
            self.move_fun(pose) # move down to table
        except:
            raise
        # Toggle gripper state
        if pick:
            self.gripper.close()
            self.grip_check_fun(pose) # TODO make it so arm goes back up even if gripper failure occurs
        else:
            self.gripper.open()
        # Move back up
        try:
            pose['position'][2] = self.working_z
            self.move_fun(pose) # move back up to working height
        except:
            raise

    
    # Randomly draw dropoff location
    def dropoff_position(self):
        rospy.logdebug('AutoCore: Discard triggered.')
        # Generate random location in dropoff area
        discard_x = self.discard_offset_x + random.random() * self.discard_length_x
        discard_y = self.discard_offset_y + random.random() * self.discard_length_y
        dropoff_pos = np.array([discard_x, discard_y, self.working_z])
        rospy.logdebug('Dropff position: {}'.format(dropoff_pos))
        return dropoff_pos
