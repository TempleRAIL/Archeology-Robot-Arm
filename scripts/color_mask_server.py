#!/usr/bin/env python

#Import Python libraries
import cv2
from sklearn.cluster import KMeans
from collections import Counter
from skimage.color import rgb2lab, rgb2hsv, deltaE_cie76
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.image as mpimg
import sys
import threading
import math

# Import ROS libraries and message types
import rospy
import ros_numpy
from cv_bridge import CvBridge, CvBridgeError  # Convert between ROS image msgs and OpenCV images
import message_filters
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import PointCloud2, Image
from robot_arm.msg import Detection3DRPY, Detection3DRPYArray
from robot_arm.srv import *

bridge = CvBridge()  # OpenCV converter

global image_msg, point_cloud_msg, camera_data_lock
image_msg = None
point_cloud_msg = None
camera_data_lock = threading.Lock()

##############################################################
# camera_data_callback(msg)
# This function saves the time-synced image and point cloud
# inputs: sensor_msgs/Image, sensor_msgs/PointCloud2

def camera_data_callback(im_msg, pc_msg):
    global image_msg, point_cloud_msg, camera_data_lock
    camera_data_lock.acquire()
    try:
        image_msg = im_msg
        point_cloud_msg = pc_msg
    finally:
        camera_data_lock.release()

##############################################################
# RGB2HEX(color)
# This function converts RGB colors to hex labels for the pie chart.
# inputs: RGB value stored in 1 x 3 array
# outputs: hex string

def RGB2HEX(color):
    return "#{:02x}{:02x}{:02x}".format(int(color[0]), int(color[1]), int(color[2]))

##############################################################
# convert_hsv_to_OpenCV(list)
# This function converts HSV colors from the standard range into the OpenCV range.
# inputs: list of HSV channels in float format: [decimal of 360 for hue], [decimal % saturation], [decimal % value]
# returns: list of HSV channels in 0-180 (hue) and 0-255 (saturation, value) ranges

def convert_hsv_to_OpenCV(float_hsv_values):
    hsv_in_OpenCV = [int(float_hsv_values[0]*180), int(float_hsv_values[1]*255), int(float_hsv_values[2]*255)]
    
    return hsv_in_OpenCV
  
##############################################################
# image_callback(msg)
# This function detects and extracts a user-defined number of the most dominant colors in an image.
# inputs: sensor_msgs/Image

def image_callback(msg):
    global image_msg, image_msg_lock
    image_msg_lock.acquire()
    try:
        image_msg = msg
    finally:
        image_msg_lock.release()

      
##############################################################
# color_mask_callback(req)
# This function is the callback for the color mask service
# inputs: robot_arm/ColorMaskRequest
# returns: robot_arm/ColorMaskResponse 

def color_mask_callback(req):
    global image_msg, point_cloud_msg, camera_data_lock
    
    if not req.num_colors == 1:
        rospy.logerr('Only 1 color supported')
        return
    
    if image_msg is None:
        rospy.logerr('No image received yet')
        return

    #print ("Extracting top ", req.num_colors, " colors.")
    
    camera_data_lock.acquire()
    try:
        img = bridge.imgmsg_to_cv2(image_msg, "bgr8")  # BGR OpenCV image
        # convert PointCloud2 ROS msg to numpy array (cloud corresponds to segmented sherds image)
        point_cloud = ros_numpy.numpify( point_cloud_msg )  # pointcloud should be dense (2D data structure corresponding to image)
    finally:
        camera_data_lock.release()

    rgb = img[:, :, ::-1]  # flip to RGB
    resized_rgb = cv2.resize(rgb, (600,400), interpolation = cv2.INTER_AREA)
    reshaped_rgb = resized_rgb.reshape(resized_rgb.shape[0]*resized_rgb.shape[1], 3)  # KMeans needs input of 2 dimensions

    # Create clusters of colors, extract into labels, and order colors in RGB and hex
    clf = KMeans(n_clusters = req.num_colors) 
    labels = clf.fit_predict(reshaped_rgb)
    counts = Counter(labels)
    center_colors = clf.cluster_centers_
    ordered_colors = [center_colors[i] for i in counts.keys()] 
    #print ("Ordered colors: ", ordered_colors)
    hex_colors = [RGB2HEX(ordered_colors[i]) for i in counts.keys()]

    if (req.show_chart):
    	print ("Close pie chart to continue.")
    	plt.figure(figsize = (8, 6))
    	plt.pie(counts.values(), labels = hex_colors, colors = hex_colors)
    	plt.show()

    # Create image from ordered colors
    img_to_cvt = np.zeros((req.num_colors,1,3),np.uint8)
    
    for i in range( len(ordered_colors) ): 
    	for j in range( len(ordered_colors[i]) ):
    	    img_to_cvt[i][0][j] = ordered_colors[i][j]
    
    hsv = rgb2hsv(img_to_cvt)  # convert image to HSV
    
    #print ("HSV image of top colors: ", hsv)    

    # Set bounds of color mask
    huethresh = 0.05
    satthresh = 0.28
    
    minhue = min(hsv[:,:,0]) - huethresh
    maxhue = max(hsv[:,:,0]) + huethresh
    
    minsat = min(hsv[:,:,1]) - satthresh
    maxsat = max(hsv[:,:,1]) + satthresh

    minval = .07
    maxval = 1

    floor = convert_hsv_to_OpenCV([minhue, minsat, minval])	# lower bound of mask
    ceiling = convert_hsv_to_OpenCV([maxhue, maxsat, maxval])	# upper bound of mask

    #print ("Floor: ", floor)
    #print ("Ceiling: ", ceiling)

    # Combine floor and ceiling into flat list for ROS message
    flat_mask = floor + ceiling

    #print ('Flat mask: ', flat_mask)


    res = ColorMaskResponse()
    res.mat_z = np.average(point_cloud['z']) # get average z value of top face of mat

    # Construct message for color mask
    # TODO set this up for more than a single color
    res.color_mask.data = flat_mask
    res.color_mask.layout.dim = [MultiArrayDimension(), MultiArrayDimension(), MultiArrayDimension()]
    
    res.color_mask.layout.dim[0].label = "height"
    res.color_mask.layout.dim[0].size = 2
    res.color_mask.layout.dim[0].stride = 3*1*2
    res.color_mask.layout.dim[1].label = "width"
    res.color_mask.layout.dim[1].size = 1
    res.color_mask.layout.dim[1].stride = 3*1
    res.color_mask.layout.dim[2].label = "channels"
    res.color_mask.layout.dim[2].size = 3
    res.color_mask.layout.dim[2].stride = 3

    #print ("Color mask sent.")
    
    return res


##############################################################
# color_mask_server()
# This function initiates a ROS service node that subscribes to the camera topic and invokes callback_make_mask on the Image message.  Video feed should be the empty mat.
# inputs: none
 
def color_mask_server():
    rospy.init_node('color_mask_server')

    # Subscribe in sync to Color Image and Color-Aligned PointCloud
    color_img_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    print("Subscribed to color image.")

    # use rs_rgbd.launch with physical RealSense camera
    pointcloud_sub = message_filters.Subscriber("/camera/depth_registered/points", PointCloud2)  
    print("Subscribed to point cloud.")

    sync = message_filters.ApproximateTimeSynchronizer([color_img_sub, pointcloud_sub], 1, 0.1, allow_headerless = True)
    sync.registerCallback( camera_data_callback )
    
    #color_img_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    color_mask_server = rospy.Service('color_mask_server', ColorMask, color_mask_callback)
 
    rospy.spin() # simply keeps python from exiting until this node is stopped

##############################################################
# main function
    
if __name__ == '__main__':
    color_mask_server()
