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
from AutoCore import AutoCore


# Stations
# 0 = pickup
# 1 = scale
# 2 = camera
# 3 = dropoff


# ** First state of State Machine: Sends the arm to the home configuration **
class Zero(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['not_ready', 'no_mask', 'ready'], input_keys = ['station', 'attempts'], output_keys = ['station', 'attempts'])
        self.core = core
        
    def execute(self, userdata):
        self.core.go_home()
        print("Arm sent home.")
        # SMACH Logic
        if self.core.status == False:
            return 'not_ready'
        elif self.core.color_mask is None:
            return 'no_mask'
        elif self.core.status == True and not self.core.color_mask is None:
            return 'ready'


# ** Second state of State Machine: Sends the arm to capture image of empty background mat **
class Calibrate(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['not_ready', 'ready'], input_keys = ['station', 'attempts'], output_keys = ['station', 'attempts'])
        self.core = core
        
    def execute(self, userdata):
        self.core.calibrate_fun()
        # SMACH Logic
        if self.core.status == True:
            return 'ready'
        else:
            return 'not_ready'


# ** Third state of the State Machine: Moves the arm to a desired configuration **
class Translate(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['not_ready', 'search', 'put_down'], input_keys = ['station', 'attempts'], output_keys = ['station', 'attempts'])
        self.core = core
        
    def execute(self, userdata):
        if userdata.station == 0:
            pos = self.core.pickup_locations[0]
        elif userdata.station == 1:
            pos = self.core.scale_location
        elif userdata.station == 2:
            pos = self.core.camera_location
        elif userdata.station == 3:
            pos = self.core.dropoff_location()
        location = {"position": pos, "pitch": self.core.location["pitch"], "roll": 0, "numerical": self.core.use_numerical_ik}
        self.core.move_fun(location)
        # SMACH logic
        if self.core.status == True:
            if userdata.station == 0:
                return 'search'
            else:
                return 'put_down'
        else:
            return 'not_ready'


# ** Fourth state of the State Machine: Examines the area below the camera for sherds **
class Examine(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['none_found', 'sherd_found', 'next_location'], input_keys = ['station', 'attempts'], output_keys = ['station', 'attempts'])
        self.core = core
        
    def execute(self, userdata):
        found = self.core.shard_fun(userdata.attempts)
        # SMACH logic
        if found:
            userdata.attempts = 0
            return 'sherd_found'
        else:
            userdata.attempts += 1
            if userdata.attempts == np.size(self.core.pickup_locations, 0):
                userdata.attempts = 0
                return 'none_found'
            return 'next_location'


# ** Fifth state of the State Machine: Lowers the gripper to acquire the sherd and returns to working height **
class Acquire(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['failed', 'acquired', 'regrasp'], input_keys = ['station', 'attempts'], output_keys = ['station', 'attempts'])
        self.core = core
    
    def execute(self, userdata):
        self.core.pick_place_fun(self.core.location, self.core.working_z, pick=True)
        # SMACH logic
        # TODO need to update logic here to account for stations other than pick up
        if self.core.status == True:
            userdata.station += 1
            userdata.attempts = 0
            return 'acquired'
        else:
            if userdata.attempts > 2:
                userdata.attempts = 0
                return 'failed'
            else:
                userdata.attempts += 1
                return 'regrasp'

# ** Sixth state of the State Machine: Lowers the gripper to discard the sherd and returns to working height **
class Discard(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['failed', 'pickup', 'discard'], input_keys = ['station', 'attempts'], output_keys = ['station', 'attempts'])
        self.core = core
        
    def execute(self, userdata):
        self.core.pick_place_fun(self.core.location, self.core.working_z, place=True)
        # SMACH logic
        if self.core.status == True:
            if userdata.station == 3:
                userdata.station = 0
                #cal_counter += 1  #TODO refresh color mask every 10 cycles
                    #if cal_counter > 9:
                        #self.core.color_mask = None
                        #return 'failed'
                return 'discard'
            else:
                return 'pickup'
        else:
            return 'failed'

def process_sherds():
    rospy.init_node('sherd_states')
    # Initialize pyrobot objects
    bot = Robot("locobot")
    configs = bot.configs
    gripper = LoCoBotGripper(configs)
    # Initialize AutoCore
    core = AutoCore(bot, gripper)
    core.calibrate_fun()
    # ** Creates the state machine **
    sm = smach.StateMachine(outcomes = ['NotReady'])
    sm.userdata.station = 0
    sm.userdata.attempts = 0
    # ** Opens state machine container **
    with sm:
        # ** Adds the states to the container **
        smach.StateMachine.add('Zero', Zero(core), transitions = {'not_ready': 'NotReady', 'no_mask': 'Calibrate', 'ready': 'Examine'})
        smach.StateMachine.add('Calibrate', Calibrate(core), transitions = {'not_ready': 'NotReady', 'ready': 'Examine'})
        smach.StateMachine.add('Translate', Translate(core), transitions = {'not_ready': 'NotReady', 'search': 'Examine', 'put_down': 'Discard'})
        smach.StateMachine.add('Examine', Examine(core), transitions = {'none_found': 'Zero', 'sherd_found': 'Acquire', 'next_location': 'Examine'})
        smach.StateMachine.add('Acquire', Acquire(core), transitions = {'failed': 'Zero', 'acquired': 'Translate', 'regrasp': 'Acquire'})
        smach.StateMachine.add('Discard', Discard(core), transitions = {'failed': 'Zero', 'pickup': 'Examine', 'discard': 'Translate'})
    # TODO add behavior for arm to move out of way of camera when taking picture / should be taken care of in core.pick_place_fun (final line).  Need to add sleep?
    # ** Execute the SMACH plan **
    outcome = sm.execute()
  
if __name__ == '__main__':
    process_sherds()

