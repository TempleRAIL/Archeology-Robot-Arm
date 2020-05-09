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
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension

bridge = CvBridge()  # OpenCV converter

##############################################################
# callback_get_conversion(CameraInfo_msg)
# This function uses the camera's intrinsinc parameters to report the real-world rectified focal length and a conversion factor between length in pixels to length in mm.
# inputs: sensor_msgs/CameraInfo
# outputs: global variables 'f_rect_mm' and 'px_per_mm'

def callback_get_conversion(CameraInfo_msg):
    print "Triggered callback_get_conversion."

    global f_rect_mm, px_per_mm
    
    # get rectified focal lengths fx and fy [px] from P matrix of intrinsic parameters
    #dim = CameraInfo_msg.P.dim
    #fx_rect_px = CameraInfo_msg.P[0+dim[0].stride*1+dim[1].stride*1+dim[2].stride*1] 
    #fy_rect_px = CameraInfo_msg.P[0+dim[0].stride*2+dim[1].stride*2+dim[2].stride*1]
    fx_rect_px = CameraInfo_msg.P[0] 
    fy_rect_px = CameraInfo_msg.P[4]
    f_rect_px = (fx_rect_px + fy_rect_px)/2  

    # hardware pixel size [mm] for Astra Orbbec S RGB sensor (subbing for Orbbec Pro)
    pixel_height_mm = 0.0019
    pixel_width_mm = pixel_height_mm

    # conversion factor
    px_per_mm = 1/pixel_height_mm

    # Calculate real-world rectified focal length [mm] for Orbbec Pro
    f_rect_mm = f_rect_px / px_per_mm

    print 'Px_per_mm', px_per_mm
    print 'Rectified real-world focal length', f_rect_mm


