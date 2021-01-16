#!/usr/bin/env python

"""
This script tests archival camera functionality in simulation.
Run this node after launching archaeology.launch and arm.launch.
"""

# Import ROS libraries and message types
import rospy
from cv_bridge import CvBridge, CvBridgeError  # Convert between ROS image msgs and OpenCV images
from sensor_msgs.msg import Image
from robot_arm.srv import *

# ROS service clients
rospy.wait_for_service('take_photo_server')
photo_srv = rospy.ServiceProxy('take_photo_server', Photo)

##############################################################
# take_photo()
# This function initiates ROS node that subscribes to the Gazebo camera topic and returns an image of the sherd being processed
# inputs: none
 
def take_photo():
    rospy.init_node('take_photo')
    rospy.logwarn('Taking archival photo. If shown, close figure to continue. Toggle figure display in mat_layout.yaml.')
    # run take_photo_server.py
    req = PhotoRequest()
    try:
        res = photo_srv(req)
    except rospy.ServiceException as e:
        rospy.logerr('Take_photo service call failed: {}'.format(e))
        raise
    else:
        photo = res.image
        rospy.logwarn('Got archival photo of sherd.')
        return photo
    rospy.spin() # simply keeps python from exiting until this node is stopped

##############################################################
# main function
    
if __name__ == '__main__':
    take_photo()
