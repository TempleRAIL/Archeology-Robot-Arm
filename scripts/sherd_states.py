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

# Import autocore
from AutoCore import AutoCore, PlanningFailure, GraspFailure


# Stations
stations = {
    'pickup': 0,
    'scale': 1,
    'camera': 2,
    'dropoff': 3
}


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
        smach.State.__init__(self, outcomes = ['not_ready', 'ready'], input_keys = [], output_keys = ['station'])
        self.core = core
        
    def execute(self, userdata):
        userdata.station = stations['pickup']
        try:
            self.core.calibrate_fun()
        except:
            return 'not_ready'
        else:
            return 'ready'


# ** Third state of the State Machine: Moves the arm to a desired configuration **
class Translate(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['replan', 'search', 'put_down'], input_keys = ['station', 'attempts', 'pose'], output_keys = ['station', 'attempts', 'pose'])
        self.core = core
        
    def execute(self, userdata):
        # Print current station
        station_string = [key for (key, value) in stations.items() if value == userdata.station]
        rospy.loginfo('Moving to station: {}'.format(*station_string))
        if userdata.station == stations['pickup']:
            pos = self.core.pickup_positions[0]
        elif userdata.station == stations['scale']:
            pos = self.core.scale_position
        elif userdata.station == stations['camera']:
            pos = self.core.camera_position
        elif userdata.station == stations['dropoff']:
            pos = self.core.dropoff_position()
        pose = {"position": pos, "pitch": self.core.pose["pitch"], "roll": 0, "numerical": self.core.use_numerical_ik}
        rospy.loginfo('Moving to: {}'.format(pose))
        try:
            self.core.move_fun(pose)
        except PlanningFailure:
            return 'replan'
        else:
            userdata.pose = pose
            if userdata.station == stations['pickup']:
                return 'search'
            else:
                return 'put_down'


# ** Fourth state of the State Machine: Examines the area below the camera for sherds **
class Examine(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['not_ready','none_found', 'sherd_found', 'next_location'], input_keys = ['station', 'attempts'], output_keys = ['station', 'attempts', 'pose'])
        self.core = core
        
    def execute(self, userdata):
        try:
            (found, sherd_pose) = self.core.shard_fun(userdata.attempts)
        except:
            return 'not_ready'
        else:
            if found:
                userdata.attempts = 0
                userdata.pose = sherd_pose
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
        smach.State.__init__(self, outcomes = ['replan', 'failed', 'acquired', 'regrasp'], input_keys = ['station', 'attempts', 'pose'], output_keys = ['station', 'attempts', 'pose'])
        self.core = core
    
    def execute(self, userdata):
        try:
            self.core.pick_place_fun(userdata.pose, userdata.station, pick=True)
        except GraspFailure:
            if userdata.attempts > 2:
                userdata.attempts = 0
                return 'failed'
            else:
                userdata.attempts += 1
                return 'regrasp'
        except PlanningFailure:
            return 'replan'
        else:
            userdata.station += 1
            userdata.attempts = 0
            return 'acquired'

# ** Sixth state of the State Machine: Lowers the gripper to discard the sherd and returns to working height **
class Discard(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['failed', 'regrasp', 'pickup', 'discard'], input_keys = ['station', 'pose', 'cal_counter'], output_keys = ['station', 'cal_counter'])
        self.core = core
        
    def execute(self, userdata):
        try:
            self.core.pick_place_fun(userdata.pose, userdata.station, place=True)
        except Exception as e:
            rospy.logwarn('Could not execute placement because of Exception: {}'.format(e))
            return 'failed'
        else:
            if userdata.station == stations['dropoff']:  # if placed in discard pile
                userdata.station = stations['pickup']
                userdata.cal_counter += 1  # refresh color mask every 10 cycles
                if userdata.cal_counter > 9:
                    self.core.color_mask = None
                    return 'failed'
                return 'discard'
            if userdata.station == stations['scale'] or userdata.station == stations['camera']:  # if placed on scale 
                return 'regrasp'
            else:
                return 'pickup'

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
    sm.userdata.pose = None
    # ** Opens state machine container **
    with sm:
        # ** Adds the states to the container **
        smach.StateMachine.add('Home', Home(core), transitions = {'no_mask': 'Calibrate', 'ready': 'Examine'})
        smach.StateMachine.add('Calibrate', Calibrate(core), transitions = {'not_ready': 'NotReady', 'ready': 'Examine'})
        smach.StateMachine.add('Translate', Translate(core), transitions = {'replan': 'Translate', 'search': 'Examine', 'put_down': 'Discard'})
        smach.StateMachine.add('Examine', Examine(core), transitions = {'not_ready': 'NotReady', 'none_found': 'Home', 'sherd_found': 'Acquire', 'next_location': 'Examine'})
        smach.StateMachine.add('Acquire', Acquire(core), transitions = {'replan': 'Acquire', 'failed': 'Home', 'acquired': 'Translate', 'regrasp': 'Acquire'})
        smach.StateMachine.add('Discard', Discard(core), transitions = {'failed': 'Home', 'pickup': 'Examine', 'regrasp': 'Acquire', 'discard': 'Translate'})
    # TODO add behavior for arm to move out of way of camera when taking picture / should be taken care of in core.pick_place_fun (final line).  Need to add sleep?
    # ** Execute the SMACH plan **
    sm.execute()
  
if __name__ == '__main__':
    process_sherds()

