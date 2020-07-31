#!/usr/bin/env python

#Import Python libraries
import time
import numpy as np
import random

# Import pyrobot libraries
from pyrobot import Robot
from pyrobot.locobot.gripper import LoCoBotGripper
from pyrobot.locobot import camera

# Import ROS libraries and message types
import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped
from robot_arm.msg import Detection3DRPY, Detection3DRPYArray
from robot_arm.srv import ColorMask, ColorMaskRequest, SherdDetections, SherdDetectionsRequest, ScaleReading, ScaleReadingRequest

rospy.wait_for_service('read_scale')
scale_srv = rospy.ServiceProxy('read_scale', ScaleReading)

def get_mass_fun():
    # run read_scale.py
    req = ScaleReadingRequest()
    try:
        res = scale_srv(req)
    except rospy.ServiceException as e:
        rospy.logerr('AutoCore: read_scale service call failed: {}'.format(e))
        raise
    else:
        mass = res.mass
        print('AutoCore: Got sherd mass: {} kg'.format(mass))
        return mass

##############################################################
# main function
    
if __name__ == '__main__':
    get_mass_fun()
