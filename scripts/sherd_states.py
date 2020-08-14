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

# Stations
stations = {
    'pickup': 0,
    'scale': 1,
    'camera_place': 2,
    'camera_pick': 3,
    'dropoff': 4
}

# Publishers
sherd_data_pub = rospy.Publisher('Sherd_Data', SherdData, queue_size=1)

# ** First state of State Machine: Sends the arm to the home configuration **
class Home(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['no_mask', 'ready'], input_keys = [], output_keys = [])
        self.core = core
        
    def execute(self, userdata):
        self.core.go_home()
        self.core.gripper.open()
        rospy.loginfo("Arm sent home.")
        # SMACH Logic
        if self.core.color_mask is None:
            return 'no_mask'
        else:
            return 'ready'


# ** Second state of State Machine: Sends the arm to capture image of empty background mat **
class Calibrate(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['not_ready', 'ready', 'replan'], input_keys = [], output_keys = ['station'])
        self.core = core
        
    def execute(self, userdata):
        userdata.station = stations['pickup']
        try:
            self.core.calibrate_fun()
        except PlanningFailure:
            return 'replan'
        except:
            return 'not_ready'
        else:
            return 'ready'


# ** Third state of the State Machine: Moves the arm to a desired configuration **
class Translate(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['replan', 'search', 'put_down'], input_keys = ['station', 'attempts'], output_keys = ['station', 'attempts', 'goal'])
        self.core = core
        
    def execute(self, userdata):
        # Print current station
        station_string = [key for (key, value) in stations.items() if value == userdata.station]
        rospy.loginfo('Moving to station: {}'.format(*station_string))
        if userdata.station == stations['pickup']:
            pos = self.core.pickup_positions[0]
        elif userdata.station == stations['scale']:
            pos = self.core.scale_position
        elif userdata.station == stations['camera_place']:
            pos = self.core.camera_position
        elif userdata.station == stations['camera_pick']:
            pos = self.core.camera_survey_position
        elif userdata.station == stations['dropoff']:
            pos = self.core.dropoff_position()
        pose = {"position": pos, "pitch": self.core.working_p, "roll": self.core.working_r, "numerical": self.core.use_numerical_ik}
        rospy.loginfo('Moving to: {}'.format(pose))
        try:
            self.core.move_fun(pose)
        except PlanningFailure:
            return 'replan'
        else:
            if userdata.station == stations['pickup'] or userdata.station == stations['camera_pick']:
                return 'search'
            else:
                userdata.goal = pose
                return 'put_down'


# ** Fourth state of the State Machine: Examines the area below the camera for sherds **
class Examine(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['not_ready', 'replan', 'none_found', 'sherd_found', 'next_location'], input_keys = ['station', 'attempts'], output_keys = ['station', 'attempts', 'goal'])
        self.core = core
        
    def execute(self, userdata):
        # TODO set up to return to same pickup location for next sherd to avoid researching empty areas
        try:
            if userdata.station == stations['pickup']:
                (found, sherd_poses) = self.core.shard_fun(userdata.attempts)
            elif userdata.station == stations['camera_pick']:
                (found, sherd_poses) = self.core.detect_fun()
        except PlanningFailure:
            return 'replan'
        except Exception as e:
            rospy.logwarn('Exception raised: {}'.format(e))
            return 'not_ready'
        else:
            if found:
                userdata.attempts = 0
                userdata.goal = sherd_poses[0]
                return 'sherd_found'
            else:
                userdata.attempts += 1
                if userdata.attempts == np.size(self.core.pickup_positions, 0):
                    userdata.attempts = 0
                    return 'none_found'
                return 'next_location'


# ** Fifth state of the State Machine: Lowers the gripper to acquire the sherd and returns to working height **
class Acquire(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['replan', 'failed', 'acquired', 'regrasp'], input_keys = ['station', 'attempts', 'goal'], output_keys = ['station', 'attempts', 'sherd_msg'])
        self.core = core
    
    def execute(self, userdata):
        pose = userdata.goal
        if userdata.attempts != 0:  # If cannot go to goal pose first time around, get pose from sensor
            try:
                self.core.gripper.open()
                pose['position'][2] = self.core.survey_z  # move up to survey height
                self.core.move_fun(pose)
                (found, sherd_poses) = self.core.detect_fun()
            except PlanningFailure:
                return 'replan'
            except Exception as e:
                rospy.logwarn('Could not pick up sherd because of Exception: {}'.format(e))
                userdata.station = stations['pickup']
                return 'failed'
            else:
                if found:
                    pose = sherd_poses[0]
                else:
                    rospy.logwarn('No sherd seen, trying last known sherd pose')
                    pose = userdata.goal
        # Get goal pose
        if userdata.station == stations['pickup']:
            pose['position'][2] = self.core.table_z # overwrites z-value of top face of sherd (assigned in self.core.detect_fun)
        elif userdata.station == stations['scale']:
            pose['position'][2] = self.core.scale_z
        elif userdata.station == stations['camera_pick']:
            pose['position'][2] = self.core.camera_z
        elif userdata.station == stations['dropoff']:
            pose['position'][2] = self.core.discard_z
        # Try to acquire sherd
        try:
            if userdata.station == stations['scale']: # skip AutoCore pick_place_fun
                pose['position'][2] += (self.core.gripper_len + self.core.clearance) # move gripper back down around sherd
                self.core.move_fun(pose)
                self.core.gripper.close()
                self.core.grip_check_fun(pose)
                pose['position'][2] = self.core.working_z # move back up to working height after grasping
                self.core.move_fun(pose)
            else: # pick up with AutoCore's pick_place_fun, which takes care of gripper length and clearance
                self.core.pick_place_fun(pose, pick=True)
        except GraspFailure:
            if userdata.attempts > 2:
                userdata.attempts = 0
                userdata.station = stations['pickup']
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
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['failed', 'replan', 'regrasp_scale', 'regrasp_camera', 'next_sherd'], input_keys = ['station', 'goal', 'cal_counter', 'sherd_msg'], output_keys = ['station', 'cal_counter', 'goal', 'sherd_msg'])
        self.core = core
        
    def execute(self, userdata):
	sherd_msg = userdata.sherd_msg
        # Get goal pose
        pose = userdata.goal
        if userdata.station == stations['pickup']:
            pose['position'][2] = self.core.table_z
        elif userdata.station == stations['scale']:
            pose['position'][2] = self.core.scale_z + self.core.sherd_allowance  # allow for irregularly-shaped sherds
        elif userdata.station == stations['camera_place']:
            pose['position'][2] = self.core.camera_z + self.core.sherd_allowance
        elif userdata.station == stations['dropoff']:
            pose['position'][2] = self.core.table_z
        # Try to place sherd
        try:
            self.core.pick_place_fun(pose, place=True)
        except PlanningFailure:
            return 'replan'
        except Exception as e:
            rospy.logwarn('Could not execute placement because of Exception: {}'.format(e))
            userdata.station = stations['pickup']
            return 'failed'
        else: 
            if userdata.station == stations['scale']:
                success = False
                while not success:
                    try:
                        pose['position'][2] += 0.01 # move gripper up a cm before recording sherd mass
                        self.core.move_fun(pose)
                        sherd_msg = self.core.get_mass_fun(sherd_msg)
                        # TODO store mass in database
                        pose['position'][2] -= (0.01 + self.core.sherd_allowance)
                    except PlanningFailure:
                        continue
                    except Exception as e:
                        rospy.logwarn('Could not get mass of sherd because of Exception: {}'.format(e))
                    else:
                        success = True
                return 'regrasp_scale'
            elif userdata.station == stations['camera_place']:
                # Move to standby position to get out of the way of the camera
                success = False
                while not success:
                    try:
                        pose['position'][2] = self.core.working_z
                        self.core.move_fun(pose) # move straight up to working height
                        self.core.move_fun(self.core.standby_pose)
                    except PlanningFailure:
                        continue
                    except Exception as e:
                        rospy.logwarn('Could not move to standby position because: {}'.format(e))
                        userdata.station = stations['pickup']
                        return 'failed'
                    else:
                       success = True
                # Take archival photo
                success = False
                while not success:
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
                        success = True
                        userdata.station += 1
                return 'regrasp_camera'
            else:  # if placed in discard pile
                success = False
                while not success:
                    try:
                        pose['position'][2] = self.core.working_z
                        self.core.move_fun(pose)
                    except PlanningFailure:
                        continue
                    else:
                        success = True
                userdata.station = stations['pickup']
                userdata.cal_counter += 1  # refresh color mask every 10 cycles
                if userdata.cal_counter > 9:
                    self.core.color_mask = None
                    return 'failed'
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
    # ** Creates the state machine **
    sm = smach.StateMachine(outcomes = ['NotReady'])
    sm.userdata.station = stations['pickup']
    sm.userdata.attempts = 0
    sm.userdata.cal_counter = 0
    sm.userdata.goal = None
    sm.userdata.sherd_msg = None
    # ** Opens state machine container **
    with sm:
        # ** Adds the states to the container **
        smach.StateMachine.add('Home', Home(core), transitions = {'no_mask': 'Calibrate', 'ready': 'Translate'})
        smach.StateMachine.add('Calibrate', Calibrate(core), transitions = {'replan': 'Calibrate', 'not_ready': 'NotReady', 'ready': 'Examine'})
        smach.StateMachine.add('Translate', Translate(core), transitions = {'replan': 'Translate', 'search': 'Examine', 'put_down': 'PlaceSherd'})
        smach.StateMachine.add('Examine', Examine(core), transitions = {'not_ready': 'NotReady', 'replan': 'Examine', 'none_found': 'Home', 'sherd_found': 'Acquire', 'next_location': 'Examine'})
        smach.StateMachine.add('Acquire', Acquire(core), transitions = {'replan': 'Acquire', 'failed': 'Home', 'acquired': 'Translate', 'regrasp': 'Acquire'})
        smach.StateMachine.add('PlaceSherd', PlaceSherd(core), transitions = {'failed': 'Home', 'replan': 'PlaceSherd', 'next_sherd': 'Translate', 'regrasp_scale': 'Acquire', 'regrasp_camera': 'Translate'})

    # ** Execute the SMACH plan **
    sm.execute()
  
if __name__ == '__main__':
    process_sherds()

