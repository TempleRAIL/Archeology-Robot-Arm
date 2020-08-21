#!/usr/bin/env python

#Import Python libraries
import time
import math
import numpy as np

# Import pyrobot
from pyrobot import Robot
from pyrobot.locobot import camera
from pyrobot.locobot.gripper import LoCoBotGripper

# Import ROS libraries and message types
import rospy
import smach
import smach_ros
from robot_arm.msg import SherdData

# Import autocore
from AutoCore import AutoCore, PlanningFailure, GraspFailure
from mat_configuration import MatConfiguration

# Publishers
sherd_data_pub = rospy.Publisher('Sherd_Data', SherdData, queue_size=1)

# ** First state of State Machine: Sends the arm to the home configuration **
class Home(smach.State):
    def __init__(self, core, mat):
        smach.State.__init__(self, outcomes = ['no_mask', 'ready'], input_keys = [], output_keys = [])
        self.core = core
        self.mat = mat
        
    def execute(self, userdata):
        self.core.go_home()
        self.core.gripper.open()
        rospy.loginfo("Arm sent home.")
        # SMACH Logic
        for k,v in self.core.color_masks.items():
            if self.core.color_masks[k] is None:
                return 'no_mask'
            else:
                return 'ready'


# ** Second state of State Machine: Sends the arm to capture image of empty background mat **
class Calibrate(smach.State):
    def __init__(self, core, mat):
        smach.State.__init__(self, outcomes = ['not_ready', 'ready', 'replan'], input_keys = [], output_keys = ['station'])
        self.core = core
        self.mat = mat
        
    def execute(self, userdata):
        userdata.station = self.mat.stations['pickup']
        try:
            self.core.calibrate_fun(self.mat.mat_color_pose, location='mat')
            self.core.calibrate_fun(self.mat.scale_color_pose, location='scale')
        except PlanningFailure:
            return 'replan'
        except:
            return 'not_ready'
        else:
            return 'ready'


# ** Third state of the State Machine: Moves the arm to a desired configuration **
class Translate(smach.State):
    def __init__(self, core, mat):
        smach.State.__init__(self, outcomes = ['replan', 'search', 'put_down'], input_keys = ['station', 'attempts'], output_keys = ['station', 'attempts', 'goal'])
        self.core = core
        self.mat = mat
        
    def execute(self, userdata):
        # Print current station
        self.mat.print_station(userdata.station)
        pose = self.mat.get_goal_pose(userdata.station) # get goal pose
        rospy.loginfo('Moving to: {}'.format(pose))
        try:
            self.core.move_fun(pose)
        except PlanningFailure:
            return 'replan'
        else:
            if userdata.station == self.mat.stations['pickup'] or userdata.station == self.mat.stations['scale_pick'] or userdata.station == self.mat.stations['camera_pick']:
                return 'search'
            else:
                userdata.goal = pose
                return 'put_down'


# ** Fourth state of the State Machine: Examines the area below the camera for sherds **
class Examine(smach.State):
    def __init__(self, core, mat):
        smach.State.__init__(self, outcomes = ['not_ready', 'replan', 'none_found', 'sherd_found', 'next_location'], input_keys = ['station', 'attempts'], output_keys = ['station', 'attempts', 'goal'])
        self.core = core
        self.mat = mat
        
    def execute(self, userdata):
        # TODO set up to return to same pickup location for next sherd to avoid researching empty areas
        try:
            if userdata.station == self.mat.stations['pickup']:
                (found, sherd_poses) = self.core.shard_fun(self.mat.pickup_pose(userdata.attempts))
            #elif userdata.station == self.mat.stations['scale_pick']:
             #   (found, sherd_poses) = self.core.detect_fun(self.core.color_masks['scale'])
            elif userdata.station == self.mat.stations['camera_pick']:
                (found, sherd_poses) = self.core.detect_fun(self.core.color_masks['mat'])
        except PlanningFailure:
            return 'replan'
        except Exception as e:
            rospy.logwarn('Exception raised: {}'.format(e))
            return 'not_ready'
        else:
            if found:
                userdata.attempts = 0
                goal = sherd_poses[0]
                goal['pitch'] = self.mat.working_p
                goal['numerical'] = self.mat.use_numerical_ik
                userdata.goal = goal
                return 'sherd_found'
            else:
                userdata.attempts += 1
                if userdata.attempts == self.mat.num_pickup_positions:
                    userdata.attempts = 0
                    return 'none_found'
                return 'next_location'


# ** Fifth state of the State Machine: Lowers the gripper to acquire the sherd and returns to working height **
class Acquire(smach.State):
    def __init__(self, core, mat):
        smach.State.__init__(self, outcomes = ['replan', 'failed', 'acquired', 'regrasp'], input_keys = ['station', 'attempts', 'goal'], output_keys = ['station', 'attempts', 'sherd_msg'])
        self.core = core
        self.mat = mat
    
    def execute(self, userdata):
        pose = userdata.goal # get goal pose
        pose = self.mat.select_goal_z(pose, userdata.station) # get goal z
        # If failed to go to goal pose first time around (i.e. made more than one attempt already), get pose from sensor
        if userdata.attempts != 0:
            try:
                self.core.gripper.open()
                pose['position'][2] = self.mat.survey_z  # move up to survey height
                self.core.move_fun_retry(pose)
                if userdata.station == self.mat.stations['scale_pick']:
                    pose['position'][0] -= 0.075
                    self.core.move_fun_retry(pose)
                    (found, sherd_poses) = self.core.detect_fun(self.core.color_masks['scale'])
                else:
                    (found, sherd_poses) = self.core.detect_fun(self.core.color_masks['mat'])                    
            except PlanningFailure:
                return 'replan'
            except Exception as e:
                rospy.logwarn('Could not pick up sherd because of Exception: {}'.format(e))
                userdata.station = self.mat.stations['pickup']
                return 'failed'
            else:
                if found:
                    pose = sherd_poses[0]
                else:
                    rospy.logwarn('No sherd seen, trying last known sherd pose')
                    pose = userdata.goal
        # Try to acquire sherd
        try:
            #self.core.pick_place_fun(pose, self.mat.working_z, pick=True)
            if userdata.station == self.mat.stations['scale_pick']: # skip AutoCore pick_place_fun
                self.core.gripper.open()
                pose['position'][2] += self.core.gripper_len + self.core.clearance
                self.core.move_fun_retry(pose)
                self.core.gripper.close()
                self.core.grip_check_fun(pose)
                pose['position'][2] = self.mat.working_z # move back up to working height after grasping
                self.core.move_fun_retry(pose)
            else: # pick up with AutoCore's pick_place_fun, which takes care of gripper length and clearance
                self.core.pick_place_fun(pose, self.mat.working_z, pick=True)
        except GraspFailure:
            if userdata.attempts > 2:
                userdata.attempts = 0
                userdata.station = self.mat.stations['pickup']
                return 'failed'
            else:
                userdata.attempts += 1
                return 'regrasp'
        except PlanningFailure:
            return 'replan'
        else:
            userdata.station += 1
            userdata.attempts = 0
            userdata.sherd_msg = SherdData()
            return 'acquired'


# ** Sixth state of the State Machine: Lowers the gripper to discard the sherd and returns to working height **
class PlaceSherd(smach.State):
    def __init__(self, core, mat):
        smach.State.__init__(self, outcomes = ['failed', 'replan', 'retrieve_scale', 'retrieve_camera', 'next_sherd', 'recalibrate'], input_keys = ['station', 'goal', 'cal_counter', 'sherd_msg'], output_keys = ['station', 'cal_counter', 'goal', 'sherd_msg'])
        self.core = core
        self.mat = mat
        
    def execute(self, userdata):
        sherd_msg = userdata.sherd_msg
        pose = userdata.goal # get goal pose
        pose = self.mat.select_goal_z(pose, userdata.station) # get goal z
        # Try to place sherd
        try:
            self.core.pick_place_fun(pose, self.mat.working_z, place=True)
        except PlanningFailure:
            return 'replan'
        except Exception as e:
            rospy.logwarn('Could not execute placement because of Exception: {}'.format(e))
            userdata.station = self.mat.stations['pickup']
            return 'failed'
        else:
            if userdata.station == self.mat.stations['scale_place']:
                #pose['position'][2] += 0.01 # move gripper up a cm before recording sherd mass
                try:
                    self.core.move_fun_retry(pose)
                    sherd_msg = self.core.get_mass_fun(sherd_msg)
                    # TODO store mass in database
                except Exception as e:
                    rospy.logwarn('Could not get mass of sherd because of Exception: {}'.format(e))
                else:
                    userdata.station += 1
                    return 'retrieve_scale'
            elif userdata.station == self.mat.stations['camera_place']:
                # Move to standby position to get out of the way of the camera
                try:
                    pose['position'][2] = self.mat.working_z
                    self.core.move_fun_retry(pose) # move straight up to working height
                    self.core.move_fun_retry(self.mat.standby_pose)
                except Exception as e:
                    rospy.logwarn('Could not move to standby position because: {}'.format(e))
                    userdata.station = self.mat.stations['pickup']
                    return 'failed'
                # Take archival photo
                try:
                    sherd_msg = self.core.take_photo_fun(sherd_msg)
                    # TODO store image in database
                    if not (sherd_msg.mass or sherd_msg.archival_photo): # if either of these values is None
                        sherd_msg.incomplete = True
                        rospy.logwarn('/Sherd_Data msg seq# {} has missing fields.'.format(sherd_msg.header.seq))
                    else:
                        sherd_msg.incomplete = False
                    sherd_data_pub.publish(sherd_msg) # publish SherdData message to /Sherd_Data
                except Exception as e:
                    rospy.logwarn('Exception raised: {}'.format(e))
                else:
                    userdata.station += 1
                    return 'retrieve_camera'
            else:  # if placed in discard pile
                try:
                    pose['position'][2] = self.core.working_z
                    self.core.move_fun_retry(pose)
                except:
                    return 'failed'
                userdata.station = self.mat.stations['pickup']
                userdata.cal_counter += 1  # refresh color mask every 10 cycles
                if userdata.cal_counter > 9:
                    userdata.cal_counter = 0
                    return 'recalibrate'
                else:
                    return 'next_sherd'

def process_sherds():
    rospy.init_node('sherd_states')
    # Initialize pyrobot objects
    bot = Robot("locobot", use_base=False)
    configs = bot.configs
    gripper = LoCoBotGripper(configs, wait_time=3)
    # Initialize AutoCore
    core = AutoCore(bot, gripper)
    mat = MatConfiguration()
    # ** Creates the state machine **
    sm = smach.StateMachine(outcomes = ['NotReady'])
    sm.userdata.station = mat.stations['pickup']
    sm.userdata.attempts = 0
    sm.userdata.cal_counter = 0
    sm.userdata.goal = None
    sm.userdata.sherd_msg = None
    # ** Opens state machine container **
    with sm:
        # ** Adds the states to the container **
        smach.StateMachine.add('Home', Home(core, mat), transitions = {'no_mask': 'Calibrate', 'ready': 'Translate'})
        smach.StateMachine.add('Calibrate', Calibrate(core, mat), transitions = {'replan': 'Calibrate', 'not_ready': 'NotReady', 'ready': 'Examine'})
        smach.StateMachine.add('Translate', Translate(core, mat), transitions = {'replan': 'Translate', 'search': 'Examine', 'put_down': 'PlaceSherd'})
        smach.StateMachine.add('Examine', Examine(core, mat), transitions = {'not_ready': 'NotReady', 'replan': 'Examine', 'none_found': 'Home', 'sherd_found': 'Acquire', 'next_location': 'Examine'})
        smach.StateMachine.add('Acquire', Acquire(core, mat), transitions = {'replan': 'Acquire', 'failed': 'Home', 'acquired': 'Translate', 'regrasp': 'Acquire'})
        smach.StateMachine.add('PlaceSherd', PlaceSherd(core, mat), transitions = {'failed': 'Home', 'replan': 'PlaceSherd', 'next_sherd': 'Translate', 'retrieve_scale': 'Acquire', 'retrieve_camera': 'Translate', 'recalibrate': 'Calibrate'})

    # ** Execute the SMACH plan **
    sm.execute()

if __name__ == '__main__':
    process_sherds()

