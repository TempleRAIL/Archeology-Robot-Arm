#!/usr/bin/env python

# Import Python libraries
import numpy as np

# Import ROS libraries and message types
import rospy
from std_msgs.msg import Float32

##############################################################
# dummy_camera_height()
# This function initiates ROS node that publishes a dummy height of camera lens from ground, assuming distance vector is orthogonal to ground. 
# inputs: none
# outputs: Float32 message
 
def dummy_camera_height():
    pub = rospy.Publisher('Dummy_Height', Float32, queue_size=1) 
    rospy.init_node('dummy_camera_height')
    rate = rospy.Rate(10)	# 10 Hz
    while not rospy.is_shutdown():
    	dummy_height = 279.4	# mm
    	rospy.loginfo(dummy_height)
    	pub.publish(dummy_height)
    	rate.sleep()
##############################################################
# main function
    
if __name__ == '__main__':
    dummy_camera_height()
