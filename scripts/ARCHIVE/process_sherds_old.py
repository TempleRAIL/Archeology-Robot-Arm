#!/usr/bin/env python

#Import Python libraries
import time
import numpy as np
from numpy.random import random_integers as rand

# Import ROS libraries and message types
from pyrobot import Robot
import message_filters
import rospy
import ros_numpy
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray

# Publishers
survey_stamp_pub = rospy.Publisher('Survey_Stamp', Bool, queue_size=1, latch=True)

# pick-up area geometry [meters]
X_MIN = 0.22
X_MAX = 0.49

Y_MIN = -0.35 
Y_MAX = 0.35 

##############################################################
# go_survey()
# This function moves the arm to the surveillance position and publishes a Bool message.
# inputs: none
# publications: robot_arm/msgs/Bool

def go_survey():
    bot = Robot('locobot')  # initialize Locobot
    bot.arm.go_home()

    # send ee to surveillance position
    survey_position = [0.25, 0, 0.37]
    survey_orientation = [0, np.pi/3, 0]  # Euler angles
    survey_pose = {'position': np.array( survey_position ), 'orientation': np.array( survey_orientation )}
    bot.arm.set_ee_pose(**survey_pose)
    survey_status = True

    # publish timestamp of surveillance position
    msg = Bool()
    msg.data = survey_status    
    survey_stamp_pub.publish(msg)
    print("/Survey_Stamp message published.")

    
##############################################################
# process_sherds()
# This function initiates the process_sherds ROS node.
# inputs: none

def process_sherds():
    rospy.init_node('process_sherds')  # initiate node
    go_survey()

    #boundbox_sub = rospy.Subscriber("/Bounding_Boxes", Detection2DArray, callback_pickup)
    rospy.spin() # keeps Python from exiting until this node is stopped

##############################################################
# main function
    
if __name__ == '__main__':
    process_sherds()
