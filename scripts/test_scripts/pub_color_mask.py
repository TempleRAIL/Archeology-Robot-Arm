#!/usr/bin/env python

# Import Python libraries
import rospy

# Import package services
from robot_arm.srv import *
from std_msgs.msg import Int16MultiArray

"""
This script publishes calls the color_mask_server service once and and publishes the response without.
Allows for testing perception scrips without running the full simulation.
Use in conjunction with a rosbag playback.
"""

# Function to generate color mask from image of empty background
def color_mask_publisher():
    rospy.init_node('color_mask_publisher')

    # ROS publishers
    pub_color_mask = rospy.Publisher('Color_Mask', Int16MultiArray, queue_size=1, latch=True) # locobot status publisher

    # ROS service clients
    rospy.wait_for_service('color_mask_server')
    color_mask_srv = rospy.ServiceProxy('color_mask_server', ColorMask)

    # Request data from service
    req = ColorMaskRequest() # srv request
    req.num_colors = 1
    try:
        res = color_mask_srv(req) # srv response
    except rospy.ServiceException as e:
        rospy.logerr('Cluster_Test: Color mask service call failed: {}'.format(e))
        raise
    # Publish srv response
    else:
        rospy.logwarn('Cluster_Test: Got color mask.')
        print res.color_mask
        pub_color_mask.publish(res.color_mask)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

##############################################################
# main function
    
if __name__ == '__main__':
    color_mask_publisher()
