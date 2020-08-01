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
            if userdata.station == stations['pickup']:
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
            (found, sherd_poses) = self.core.shard_fun(userdata.attempts)
        except PlanningFailure:
            return 'replan'
        except:
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
        smach.State.__init__(self, outcomes = ['replan', 'failed', 'acquired', 'regrasp'], input_keys = ['station', 'attempts', 'goal'], output_keys = ['station', 'attempts'])
        self.core = core
    
    def execute(self, userdata):
        # Try to go to goal pose first time around
        if userdata.attempts == 0:
            pose = userdata.goal
        # Get pose from sensor
        else:
            try:
                pose['position'][2] = self.core.pickup_area_z  # move up to survey height
                self.core.move_fun(pose)
                (found, sherd_poses) = self.core.detect_fun()
            except:
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
        elif userdata.station == stations['camera']:
            pose['position'][2] = self.core.camera_z
        elif userdata.station == stations['dropoff']:
            pose['position'][2] = self.core.discard_z
        # Try to acquire sherd
        try:
            if userdata.station == stations['scale']:
                self.core.gripper.close()  # skip AutoCore's pick_place_fun and close in place around sherd on scale
            else:
                self.core.pick_place_fun(pose, pick=True)
            pose['position'][2] = self.core.working_z # move back up to working height after grasping
            self.core.move_fun(pose)
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
class PlaceSherd(smach.State):
    def __init__(self, core):
        smach.State.__init__(self, outcomes = ['failed', 'replan', 'regrasp', 'next_sherd', 'success'], input_keys = ['station', 'goal', 'cal_counter'], output_keys = ['station', 'cal_counter', 'goal'])
        self.core = core
        
    def execute(self, userdata):
        # Get goal pose
        pose = userdata.goal
        if userdata.station == stations['pickup']:
            pose['position'][2] = self.core.table_z
        elif userdata.station == stations['scale']:
            pose['position'][2] = self.core.scale_z
        elif userdata.station == stations['camera']:
            pose['position'][2] = self.core.camera_z
        elif userdata.station == stations['dropoff']:
            pose['position'][2] = self.core.table_z
        # Try to place sherd
        try:
            self.core.pick_place_fun(pose, place=True)
        except PlanningFailure:
            return 'replan'
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
                return 'success'
            elif userdata.station == stations['scale']:
                time.sleep(1)
                try:
                    mass = self.core.get_mass_fun()
                except Exception as e:
                    rospy.logwarn('Could not get mass of sherd because of Exception: {}'.format(e))
                # TODO store mass in database
                return 'regrasp'
            elif userdata.station == stations['camera']:  # if placed on scale
                # Move to standby position to get out of the way of the camera
                success = False
                while not success:
                    try:
                        pose['position'][2] = self.core.working_z
                        self.core.move_fun(pose) # move straight up to working height
                        self.core.move_fun(self.core.standby_pose)
                    except PlanningFailure:
                        continue
                    except:
                        return 'failed'
                    else:
                        success = True
                try:
                    archival_photo = self.core.take_photo_fun()
                    # TODO store image in database
                except Exception as e:
                    rospy.logwarn('Could not take photo of sherd because of Exception: {}'.format(e))
                return 'regrasp'
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
    # ** Opens state machine container **
    with sm:
        # ** Adds the states to the container **
        smach.StateMachine.add('Home', Home(core), transitions = {'no_mask': 'Calibrate', 'ready': 'Examine'})
        smach.StateMachine.add('Calibrate', Calibrate(core), transitions = {'not_ready': 'NotReady', 'ready': 'Examine'})
        smach.StateMachine.add('Translate', Translate(core), transitions = {'replan': 'Translate', 'search': 'Examine', 'put_down': 'PlaceSherd'})
        smach.StateMachine.add('Examine', Examine(core), transitions = {'not_ready': 'NotReady', 'replan': 'Examine', 'none_found': 'Home', 'sherd_found': 'Acquire', 'next_location': 'Examine'})
        smach.StateMachine.add('Acquire', Acquire(core), transitions = {'replan': 'Acquire', 'failed': 'Home', 'acquired': 'Translate', 'regrasp': 'Acquire'})
        smach.StateMachine.add('PlaceSherd', PlaceSherd(core), transitions = {'failed': 'Home', 'replan': 'PlaceSherd', 'next_sherd': 'Examine', 'regrasp': 'Acquire', 'success': 'Translate'})

    # ** Execute the SMACH plan **
    sm.execute()
  
if __name__ == '__main__':
    process_sherds()

