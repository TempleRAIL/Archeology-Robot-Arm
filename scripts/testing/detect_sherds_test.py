#!/usr/bin/env python

# Import Python libraries
import rospy
from std_msgs.msg import Int16MultiArray

# Import package services
from robot_arm.srv import *

##############################################################
# Callback to request sherd detections from service
def callback(msg):
    # ROS service clients
    rospy.wait_for_service('detect_by_watershed_server')
    detection_srv = rospy.ServiceProxy('detect_by_watershed_server', SherdDetections)

    color_mask = msg
    bgnd_img=None
    handle_overlap=True
    cluster_threshold=0.01
    found = False
    sherd_poses = [] # initialize empty

    # confirm that color mask exists
    if color_mask is None:
        rospy.logwarn('AutoCore: This color mask is missing.')

    # create srv request
    req = SherdDetectionsRequest()
    req.color_mask = color_mask
    req.cluster_threshold = cluster_threshold
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
