#!/usr/bin/env python

#Import Python libraries
import time
import math
import numpy as np
from pyrobot import Robot
from AutoCore import AutoCore
from pyrobot.locobot import camera
from pyrobot.locobot.gripper import LoCoBotGripper

# Import ROS libraries and message types
#import message_filters
import rospy
import ros_numpy
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray

# ** Import SMACH **
import smach
import smach_ros

bot = Robot("locobot")
configs = bot.configs
gripper = LoCoBotGripper(configs)
core = AutoCore(bot,gripper)

# ** First state of State Machine: Sends the arm to the home configuration **
class Zero(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['outcome1', 'outcome2'], input_keys = ['status_in'], output_keys = ['status_out'])
        
        def execute(self, userdata):
            userdata.status_out = userdata.status_in
            bot.arm.go_home()
            print("Arm sent home.")
            
            if core.DEF_STATUS == True:
                return 'outcome2'
            else:
                return 'outcome1'

# ** Second state of the State Machine: Moves the arm to a desired configuration **
class Translate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['outcome1', 'outcome2', 'outcome3'], input_keys = ['status_in'], output_keys = ['status_out'])
        
        def execute(self, userdata):
            userdata.status_out = userdata.status_in
            location = {"position": core.DEF_POS(userdata.status_in), "pitch": core.DEF_PITCH, "roll": 0, "numerical": core.DEF_NUMERICAL}
            core.moveFun(**location)
             
            if core.DEF_STATUS == True:
                if userdata.status_in == 0:
                    return 'outcome2'
                else:
                    return 'outcome3'
            else:
                return 'outcome1'


# ** Third state of the State Machine: Examines the area below the camera for sherds **
class Examine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['outcome1', 'outcome2', 'outcome3'], input_keys = ['status_in'], output_keys = ['status_out'])
        
        def execute(self, userdata):
            userdata.status_out = userdata.status_in
            if core.shardFun():
                return 'outcome2'
            else:
                core.DEF_LOOP += 1
                core.DEF_CURRENT += 1
                if core.DEF_LOOP > 11:
                    core.DEF_LOOP = 0
                    core.DEF_CURRENT = 0
                    return 'outcome1'
                return 'outcome3'
        
# ** Fourth state of the State Machine: Lowers the gripper to acquire the sherd and returns to working height **
class Acquire(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['outcome1', 'outcome2', 'outcome3'], input_keys = ['status_in'], output_keys = ['status_out'])
        
        def execute(self, userdata):
            location = {"position": core.DEF_POSITION, "pitch": core.DEF_PITCH, "roll": core.DEF_ORIENTATION, "numerical": core.DEF_NUMERICAL}
            core.pickFun(core.DEF_Z, **location)
            
            if core.DEF_STATUS == True:
                userdata.status_out = userdata.status_in + 1
                return 'outcome2'
            else:
                if core.DEF_TRY > 2:
                    core.DEF_TRY = 0
                    return 'outcome1'
                else:
                    core.DEF_TRY += 1
                    return 'outcome3'

# ** Fifth state of the State Machine: Lowers the gripper to discard the sherd and returns to working height **
class Discard(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['outcome1', 'outcome2', 'outcome3'], input_keys = ['status_in'], output_keys = ['status_out'])
        
        def execute(self, userdata):
            location = {"position": core.DEF_POSITION, "pitch": core.DEF_PITCH, "roll": core.DEF_ORIENTATION, "numerical": core.DEF_NUMERICAL}
            core.placeFun(core.DEF_Z, **location)
            
            if core.DEF_STATUS == True:
                if userdata.status_in == 3:
                    userdata.status_out = 0
                    return 'outcome3'
                else:
                    userdata.status_out = userdata.status_in
                    return 'outcome2'
            else:
                userdata.status_out = userdata.status_in
                return 'outcome1'

def process_sherds():
    
    bot.arm.go_home()
    print("Arm sent home.")
    core.calibrateFun()
    
    #rospy.init_node('process_sherds')
    #rospy.init_node('SherdStates')
    
    # ** Creates the state machine **
    sm = smach.StateMachine(outcomes = ['Zero'])
    sm.userdata.status = 0
    
    # ** Opens state machine container **
    with sm:
        # ** Adds the states to the container **
        smach.StateMachine.add('Zero', Zero(), transitions = {'outcome1': 'Zero', 'outcome2': 'Examine'}, remapping={'status_in': 'status', 'status_out': 'status'})
        smach.StateMachine.add('Translate', Translate(), transitions = {'outcome1': 'Zero', 'outcome2': 'Examine', 'outcome3': 'Discard'}, remapping={'status_in': 'status', 'status_out': 'status'})
        smach.StateMachine.add('Examine', Examine(), transitions = {'outcome1': 'Zero', 'outcome2': 'Acquire', 'outcome3': 'Examine'}, remapping={'status_in': 'status', 'status_out': 'status'})
        smach.StateMachine.add('Acquire', Acquire(), transitions = {'outcome1': 'Zero', 'outcome2': 'Translate', 'outcome3': 'Acquire'}, remapping={'status_in': 'status', 'status_out': 'status'})
        smach.StateMachine.add('Discard', Discard(), transitions = {'outcome1': 'Zero', 'outcome2': 'Examine', 'outcome3': 'Translate'}, remapping={'status_in': 'status', 'status_out': 'status'})

    # ** Execute the SMACH plan **
    outcome = sm.execute()
  
if __name__ == '__main__':
    process_sherds()
