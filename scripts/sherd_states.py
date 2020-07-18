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
# 0 = pickup
# 1 = scale
# 2 = camera
# 3 = dropoff


# ** First state of State Machine: Sends the arm to the home configuration **
class Zero(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['not_ready', 'no_mask', 'ready'], input_keys = [], output_keys = [])
        self.core = core
        
    def execute(self, userdata):
        self.core.go_home()
        self.core.gripper.open()
        print("Arm sent home.")
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
        userdata.station = 0
        try:
            self.core.calibrate_fun()
            return 'ready'
        except:
            return 'not_ready'


# ** Third state of the State Machine: Moves the arm to a desired configuration **
class Translate(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['not_ready', 'search', 'put_down'], input_keys = ['station', 'attempts', 'pose'], output_keys = ['station', 'attempts', 'pose'])
        self.core = core
        
    def execute(self, userdata):
        rospy.logwarn('Userdata.station is {}'.format(userdata.station))
        if userdata.station == 0:
            pos = self.core.pickup_positions[0]
        elif userdata.station == 1:
            pos = self.core.scale_position
        elif userdata.station == 2:
            pos = self.core.camera_position
        elif userdata.station == 3:
            pos = self.core.dropoff_position()
        userdata.pose = {"position": pos, "pitch": self.core.pose["pitch"], "roll": 0, "numerical": self.core.use_numerical_ik}
        try:
            rospy.logwarn('Location: {}'.format(userdata.pose))
            self.core.move_fun(userdata.pose)
            if userdata.station == 0:
                return 'search'
            else:
                return 'put_down'
        except PlanningFailure:
            return 'not_ready'


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
        # SMACH logic
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
        smach.State.__init__(self, outcomes = ['not_ready', 'failed', 'acquired', 'regrasp'], input_keys = ['station', 'attempts', 'pose'], output_keys = ['station', 'attempts', 'pose'])
        self.core = core
    
    def execute(self, userdata):
        try:
            self.core.pick_place_fun(userdata.pose, userdata.station, pick=True)
            userdata.station += 1
            userdata.attempts = 0
            return 'acquired'
        except GraspFailure:
            if userdata.attempts > 2:
                userdata.attempts = 0
                return 'failed'
            else:
                userdata.attempts += 1
                return 'regrasp'
        except PlanningFailure:
            return 'not_ready'

# ** Sixth state of the State Machine: Lowers the gripper to discard the sherd and returns to working height **
class Discard(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['failed', 'regrasp', 'pickup', 'discard'], input_keys = ['station', 'pose'], output_keys = ['station'])
        self.core = core
        
    def execute(self, userdata):
        try:
            self.core.pick_place_fun(userdata.pose, userdata.station, place=True)
            if userdata.station == 3:  # if placed in discard pile
                userdata.station = 0
                #cal_counter += 1  #TODO refresh color mask every 10 cycles
                    #if cal_counter > 9:
                        #self.core.color_mask = None
                        #return 'failed'
                return 'discard'
            elif userdata.station == 1:  # if placed on scale 
                return 'regrasp'
            else:
                return 'pickup'
        except:
            return 'failed'

def process_sherds():
    rospy.init_node('sherd_states')
    # Initialize pyrobot objects
    bot = Robot("locobot", use_base=False)
    configs = bot.configs
    gripper = LoCoBotGripper(configs, wait_time=3)
    # Initialize AutoCore
    core = AutoCore(bot, gripper)
    core.calibrate_fun()
    # ** Creates the state machine **
    sm = smach.StateMachine(outcomes = ['NotReady'])
    sm.userdata.station = 0
    sm.userdata.attempts = 0
    sm.userdata.pose = None
    # ** Opens state machine container **
    with sm:
        # ** Adds the states to the container **
        smach.StateMachine.add('Zero', Zero(core), transitions = {'not_ready': 'NotReady', 'no_mask': 'Calibrate', 'ready': 'Examine'})
        smach.StateMachine.add('Calibrate', Calibrate(core), transitions = {'not_ready': 'NotReady', 'ready': 'Examine'})
        smach.StateMachine.add('Translate', Translate(core), transitions = {'not_ready': 'NotReady', 'search': 'Examine', 'put_down': 'Discard'})
        smach.StateMachine.add('Examine', Examine(core), transitions = {'not_ready': 'NotReady', 'none_found': 'Zero', 'sherd_found': 'Acquire', 'next_location': 'Examine'})
        smach.StateMachine.add('Acquire', Acquire(core), transitions = {'not_ready': 'NotReady', 'failed': 'Zero', 'acquired': 'Translate', 'regrasp': 'Acquire'})
        smach.StateMachine.add('Discard', Discard(core), transitions = {'failed': 'Zero', 'pickup': 'Examine', 'regrasp': 'Acquire', 'discard': 'Translate'})
    # TODO add behavior for arm to move out of way of camera when taking picture / should be taken care of in core.pick_place_fun (final line).  Need to add sleep?
    # ** Execute the SMACH plan **
    sm.execute()
  
if __name__ == '__main__':
    process_sherds()

