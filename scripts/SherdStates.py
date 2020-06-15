#!/usr/bin/env python

#Import Python libraries
import time
import math
import numpy as np
from pyrobot import Robot
from AutoCore import AutoCore
from pyrobot.locobot import Camera
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
core = AutoCore(bot)

# ** First state of State Machine: Sends the arm to the home configuration **
class Zero(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['Zero', 'Translate'], input_keys = ['status_in'], output_keys = ['status_out'])
        
        def execute(self, userdata):
            userdata.status_out = userdata.status_in
            bot.arm.go_home()
            print("Arm sent home.")
            
            if core.DEF_STATUS == True:
                return 'Translate'
            else:
                return 'Zero'

# ** Second state of the State Machine: Moves the arm to a desired configuration **
class Translate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['Zero', 'Examine', 'Discard'], input_keys = ['status_in'], output_keys = ['status_out'])
        
        def execute(self, userdata):
            userdata.status_out = userdata.status_in
            location = {"position": core.DEF_POS(userdata.status_in), "pitch": core.DEF_PITCH, "roll": 0, "numerical": core.DEF_NUMERICAL}
            core.moveFun(**location)
             
            if core.DEF_STATUS == True:
                if userdata.status_in == 0:
                    return 'Examine'
                else
                    return 'Discard'
            else:
                return 'Zero'


# ** Third state of the State Machine: Examines the area below the camera for sherds **
class Examine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['Zero', 'Acquire', 'Examine'], input_keys = ['status_in'], output_keys = ['status_out'])
        
        def execute(self, userdata):
            userdata.status_out = userdata.status_in
            if core.shardFun():
                return 'Acquire'
            else
                core.DEF_LOOP += 1
                core.DEF_CURRENT += 1
                if core.DEF_LOOP > 11:
                    core.DEF_LOOP = 0
                    core.DEF_CURRENT = 0
                    return 'Zero'
                return 'Examine'
        
# ** Fourth state of the State Machine: Lowers the gripper to acquire the sherd and returns to working height **
class Acquire(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['Zero', 'Translate', 'Acquire'], input_keys = ['status_in'], output_keys = ['status_out'])
        
        def execute(self, userdata):
            location = {"position": core.DEF_POSITION, "pitch": core.DEF_PITCH, "roll": core.DEF_ORIENTATION, "numerical": core.DEF_NUMERICAL}
            core.pickFun(core.DEF_Z, **location)
            
            if core.DEF_STATUS == True:
                userdata.status_out = userdata.status_in + 1
                return 'Traverse'
            else:
                if core.DEF_TRY > 2
                    core.DEF_TRY = 0
                    return 'Zero'
                else
                    core.DEF_TRY += 1
                    return 'Acquire'

# ** Fifth state of the State Machine: Lowers the gripper to discard the sherd and returns to working height **
class Discard(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['Zero', 'Examine', 'Translate'], input_keys = ['status_in'], output_keys = ['status_out'])
        
        def execute(self, userdata):
            location = {"position": core.DEF_POSITION, "pitch": core.DEF_PITCH, "roll": core.DEF_ORIENTATION, "numerical": core.DEF_NUMERICAL}
            core.placeFun(core.DEF_Z, **location)
            
            if core.DEF_STATUS == True:
                if userdata.status_in == 3:
                    userdata.status_out = 0
                    return 'Translate'
                else
                    userdata.status_out = userdata.status_in
                    return 'Examine'
            else:
                userdata.status_out = userdata.status_in
                return 'Zero'

def process_sherds():
    
    bot.arm.go_home()
    print("Arm sent home.")
    bot.calibrateFun()
    
    rospy.init_node('process_sherds')
    
    # ** Creates the state machine **
    stateMachine = smach.StateMachine(outcomes = ['Zero'])
    stateMachine.userdata.status = 0
    
    # ** Opens state machine container **
    with stateMachine:
        # ** Adds the states to the container **
        smach.StateMachine.add('Zero', Zero(), transitions = {'outcome1': 'Zero', 'outcome2': 'Examine'}, remapping {'status_in': 'status', 'status_out': 'status'})
        smach.StateMachine.add('Translate', Translate(), transitions = {'outcome1': 'Zero', 'outcome2': 'Examine', 'outcome3': 'Discard'}, remapping {'status_in': 'status', 'status_out': 'status'})
        smach.StateMachine.add('Examine', Examine(), transitions = {'outcome1': 'Zero', 'outcome2': 'Acquire', 'outcome3': 'Examine'}, remapping {'status_in': 'status', 'status_out': 'status'})
        smach.StateMachine.add('Acquire', Acquire(), transitions = {'outcome1': 'Zero', 'outcome2': 'Translate', 'outcome3': 'Acquire'}, remapping {'status_in': 'status', 'status_out': 'status'})
        smach.StateMachine.add('Discard', Discard(), transitions = {'outcome1': 'Zero', 'outcome2': 'Examine' 'outcome3': 'Translate'}, remapping {'status_in': 'status', 'status_out': 'status'})
        
    # ** Execute the SMACH plan **
    outcome = stateMachine.execute()
  
if __name__ == '__main__':
    process_sherds()