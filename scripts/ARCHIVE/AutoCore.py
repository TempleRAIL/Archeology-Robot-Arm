import time
import numpy as np
from numpy.random import random_integers as rand

from pyrobot import Robot

DEF_STATUS = True
DEF_CURRENT = 0
DEF_HEIGHT = 0.35
DEF_POSITION = [0.04, .18, 0] # Placeholder
DEF_ZERO = [0, 0, 0, 0, 0]  # All joints = 0 rad
DEF_SCALE = [0, 0.175, 0]  # x,y,z meter coords
DEF_CAMERA = [0, -0.175, 0]  # x,y,z meter coords
DEF_SHARDS = [[0.0583,  0.2625, 0], [0.0583,  0.0875, 0], [0.0583, -0.0875, 0], 
              [0.0583, -0.2625, 0], [0.1750,  0.2625, 0], [0.1750,  0.0875, 0], 
              [0.1750, -0.0875, 0], [0.1750, -0.2625, 0], [0.2917,  0.2625, 0], 
              [0.2917,  0.0875, 0], [0.2917, -0.0875, 0], [0.2917, -0.2625, 0]]  # x,y,z meter coords
DEF_ORTHOG_EE = [0, 1.5, 0] 	# Euler angles for ee pose (gripper orthogonal to ground plane)
DEF_ORIENTATION = [0]


class autoCore:
    
    def _init_(self):
        self.status = True
        
def main():
    bot = Robot('locobot')
    bot.arm.go_home()

    while DEF_STATUS:
        shardFun()
        if not DEF_STATUS:
            break
        retrieveFun()
        if not DEF_STATUS:
            break
        moveFun()
        if not DEF_STATUS:
            break
        placeFun()
    moveFun(DEF_ZERO)
    
def shardFun():
    loop = 0
    status = True
    while status:    	
        # define an end-effector pose (position + orientation)    
    	heightLoc = {'position': np.array( [DEF_SHARDS[DEF_CURRENT][0], DEF_SHARDS[DEF_CURRENT][1], DEF_HEIGHT] ),
    	    	    'orientation': np.array( DEF_ORTHOG_EE ) }
        moveFun(heightLoc)
        # Call camera to check if present and assign true false variable
        if(True): # Place returned variable here
            status = False
            DEF_ORIENTATION = [0] # Place returned orientation here
            DEF_POSITION = [0] # Place center location of object here
        ++loop
        ++DEF_CURRENT
        if(DEF_CURRENT > 11):
            DEF_CURRENT = 0;
        if(loop > 12):
            DEF_STATUS = False
    # Once the object is secured, move to same location but at working height
    retrieveFun(DEF_POSITION)

def moveFun(pose):
    # Send desired position and orientation to IK to plot trajectory and execute it
    bot.arm.set_ee_pose(**pose)
    # If position can not be executed, set to False
    DEF_STATUS = False

def retrieveFun(position):
    moveFun(position)
    # Send command to close gripper
    # If closed completely, set to False
    DEF_STATUS = False
    # Else, if it does not close completely (meaning it gripped the object), continue
    # Send the command to rise to the current location up set Z = DEF_HEIGHT
    moveFun(DEF_HEIGHT)
    
def placeFun(position):    
    moveFun(position)
    # Send command to open gripper
    # Send the command to rise to the current location up set Z = DEF_HEIGHT
    moveFun(DEF_HEIGHT)

if __name__ == "__main__":
    main()
