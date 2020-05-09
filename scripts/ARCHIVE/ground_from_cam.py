#!/usr/bin/env python

#Import Python libraries
import cv2
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.image as mpimg
import pyrealsense2 as rs	# Intel RealSense cross-platform open-source API

# Import ROS libraries and message types
import rospy
from cv_bridge import CvBridge, CvBridgeError  # Convert between ROS image msgs and OpenCV images
#import message_filters
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32, Int16MultiArray, MultiArrayLayout, MultiArrayDimension

bridge = CvBridge()  # OpenCV converter

# Global variables
ground_from_cam_pub = rospy.Publisher('Ground_from_Camera', Float32, queue_size=1)	# publisher for ground distance from camera


##############################################################
# callback_unpack_mask(MultiArray_msg)
# This function unpacks the message from the /Color_Mask topic into the upper- and lower-bounds of an HSV color mask.
# inputs: std_msgs/Int16MultiArray (or similar)
# outputs: global variables 'floor' and 'ceiling'

def callback_unpack_mask(MultiArray_msg):
    print "Triggered callback_unpack_mask."

    # Unpack HSV mask into global variables
    global floor, ceiling, has_color_mask
    floor = [MultiArray_msg.data[0], MultiArray_msg.data[1], MultiArray_msg.data[2]]	
    ceiling = [MultiArray_msg.data[3], MultiArray_msg.data[4], MultiArray_msg.data[5]]
    has_color_mask = True


##############################################################
# callback_average_depth(depth_img)
# This function calculates the average vertical distance of the ground from the camera. It takes the image of segmented sherds produced in callback_boundbox and extracts the coordinates of every black pixel (the background). It then averages all the depths associated with those pixel coordinates from the corresponding aligned-to-color depth image.
# inputs: image of segmented objects, sensor_msgs/Image
# publications: vision_msgs/Detection2DArray ROS message

def callback_average_depth(depth_img):
    if not has_color_mask: 
    	return 'Failed to unpack color mask.'

    print "Triggered callback_ground_from_cam."
    
    depth_map = bridge.imgmsg_to_cv2(depth_img, desired_encoding='passthrough')  # OpenCV image

    bg_pixels_list = []

    rows,cols,channels = objects.shape
    for i in range(rows):
	for j in range(cols):
	    if all(objects[i,j]) == False:
	    	bg_pixels_list.append([i,j])
    	    else:
    	    	pass

    print('All background (black) pixels: ' + str(bg_pixels_list))

    bg_depths_list = []

    for coordinate in bg_pixels_list:
    	bg_depths_list.append(depth_map(coordinate))

    print ('Depths of all background pixels: ' + str(bg_depths_list))

    ground_from_cam = sum(bg_depths_list)/len(bg_depths_list)

    print ('Average distance of ground from camera: %f mm.' % ground_from_cam)

##############################################################
# ground_from_cam()
# This function initiates the detect_shard ROS node. It subscribes to 1) the Color_Mask topic and 2) the camera topic.  It invokes callbacks on messages from both topics.
# inputs: none

  
def ground_from_cam():
    rospy.init_node('ground_from_cam')  # initiate 'detect shard' node

    # Subscribe to Color_Mask topic
    #rospy.Subscriber("/Color_Mask", Int16MultiArray, callback_unpack_mask, queue_size = 1)
    Color_Mask_msg = rospy.wait_for_message("/Color_Mask", Int16MultiArray)
    print "Subscribed to Color_Mask."
    callback_unpack_mask( Color_Mask_msg )

    # Subscribe to Aligned Depth topic
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, callback_ground_from_cam, queue_size = 1)
    print "Subscribed to aligned depth map."

    #ts = message_filters.ApproximateTimeSynchronizer([color_mask_sub, image_sub], 10, 0.1, allow_headerless=True)
    #ts.registerCallback(callback_boundbox)
    #print "Registered callback on both messages."

    rospy.spin() # keeps Python from exiting until this node is stopped

##############################################################
# main function
    
if __name__ == '__main__':
    detect_shard()
