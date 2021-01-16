#!/usr/bin/env python

"""
This script calls a sherd detection service without running the full simulation.
Use in conjunction with a rosbag playback.
Update the 'service', 'handle_overlap', and 'threshold' variables as necessary.
"""

# Import Python libraries
import rospy
from std_msgs.msg import Int16MultiArray

# Import package services
from robot_arm.srv import *

##############################################################
# Callback to request sherd detections from service
def callback(msg):
    service = 'detect_sherds_watershed_server'
    # ROS service clients
    rospy.wait_for_service(service)
    detection_srv = rospy.ServiceProxy(service, SherdDetections)

    color_mask = msg
    bgnd_img=None
    handle_overlap=True
    """
    - If using the watershed approach, 'threshold' represents the low gradient threshold.
    - If using clusters approach, 'threshold' is the maximum spatial distance allowed between pixels while still assigning them 
    to the same cluster.    
    """
    threshold=17
    found = False
    sherd_poses = [] # initialize empty

    # confirm that color mask exists
    if color_mask is None:
        rospy.logwarn('AutoCore: This color mask is missing.')

    # create srv request
    req = SherdDetectionsRequest()
    req.color_mask = color_mask
    req.threshold = threshold
    req.handle_overlap = handle_overlap
    if bgnd_img:
        req.subtract_background = True
        req.background_image = bgnd_img
    else:
        req.subtract_background = False

    # request data from service
    try:
        res = detection_srv(req)
    except rospy.ServiceException as e:
        rospy.logerr('AutoCore: Bounding box service call failed: {}'.format(e))
        raise
    #else:
    #    detections = res.detections.detections
    #    rospy.logdebug('Bounding boxes message: {}'.format(detections))
    
    
##############################################################
# Function to check for and return sherd detections as list of lists: [x_center, y_center, rotation_angle]
def test_detect_sherds():
    rospy.init_node('test_detect_sherds')

    # Subscribe to color mask topic and pass to callback
    rospy.Subscriber('Color_Mask', Int16MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


##############################################################
# main function
    
if __name__ == '__main__':
    test_detect_sherds()
