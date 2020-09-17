#!/usr/bin/env python

#Import Python libraries
import cv2
from sklearn.cluster import KMeans
from collections import Counter
from skimage.color import rgb2lab, rgb2hsv, deltaE_cie76
import colorsys
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.image as mpimg
import sys
import threading
import math
import operator

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
# HSV2RGB(color)
# This function converts HSV colors to RGB colors for display in pie chart. (Compatability with matplotlib)
# inputs: HSV value (OpenCV format) stored in 1 x 3 array
# outputs: RGB value

def HSV2RGB(color):
    return colorsys.hsv_to_rgb(color[0]/180, color[1]/255, color[2]/255)

##############################################################
# convert_hsv_to_OpenCV(list)
# This function converts HSV colors from the standard range into the OpenCV range.
# inputs: list of HSV channels in integer format: [degree out of 360 for hue], [0-100 saturation], [0-100 value]
# returns: list of HSV channels in 0-180 (hue) and 0-255 (saturation, value) ranges

def convert_hsv_to_OpenCV(hsv_values):
    hsv_in_OpenCV = [int(hsv_values[0]*0.01*180), int(float_hsv_values[1]*0.01*255), int(float_hsv_values[2]*0.01*255)]
    
    return hsv_in_OpenCV

##############################################################
# get_color_as_string(color)
# This function returns string labels for the pie chart.
# inputs: color value as 1x3 array
# outputs: color value as string

def get_color_as_string(color):
    return "({}, {}, {})".format(int(color[0]), int(color[1]), int(color[2]))

##############################################################
# get_mask_as_ROS_msg(num_colors, sim, cluster_labels, twoD_image)
# This function generates a ROS-msg friendly list of floors and ceilings defining a color mask for each color extracted by KMeans
# inputs: number of colors, cluster_labels = output of KMeans fit_predict method, 2D_image = OpenCV HSV image used to generate color mask, reshaped as a 2D array
# returns: flat list of floors and ceilings for each color mask

def get_mask_as_ROS_msg(num_colors, sim, cluster_labels, twoD_image):
    """
    cluster_labels and image are arrays with identical number of rows.
    
    2D_image has 3 columns for H, S, and V:
    array([ [ H S V]
            [ H S V]
            .
            .
            .
            [ H S V] ], dtype=uint8)

    cluster_labels:
    array([ 1
            0
            .
            .
            .
            2 ], dtype=uint8)

    An HSV color in the 2D_image is linked to its color cluster (cluster 0, cluster 1, etc.) by cross-referencing row numbers
    across these two arrays.
    """
    flat_mask = []
    minval = 17.85  # OpenCV value format
    maxval = 255

    for i in range(num_colors):
        cluster_indices = np.where(cluster_labels == i)	# in array of cluster labels, find every position belonging to ith cluster
        colors_in_cluster = twoD_image[cluster_indices]  # cross-reference: get array of all HSV colors belonging to cluster i

        if not sim:
            huethresh = np.std(colors_in_cluster[:,0])*0.01*180 # OpenCV hue format
            satthresh = np.std(colors_in_cluster[:,1])*0.01*255 # OpenCV saturation
        else:
            huethresh = 9  # OpenCV hue format
            satthresh = 71.4 #OpenCV saturation format

        print("Hue standard deviation is {} and sat std dev is {}".format(huethresh, satthresh))

        minhue = colors_in_cluster[i][0] - huethresh
        maxhue = colors_in_cluster[i][0] + huethresh

        minsat = colors_in_cluster[i][1] - satthresh
        maxsat = colors_in_cluster[i][1] + satthresh

        floor = [minhue, minsat, minval]	# lower bound of mask for ith color
        ceiling = [maxhue, maxsat, maxval]	# upper bound of mask ith color
        flat_mask += (floor+ceiling)  # combine floors and ceilings of all colors into flat list for ROS message

    return flat_mask

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
    
    rospy.loginfo("Extracting {} colors for this station's color mask".format(req.num_colors))
    
    if image_msg is None:
        rospy.logerr('No image received yet')
        return
  
    camera_data_lock.acquire()
    try:
        temp_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")  # BGR OpenCV image - no HSV encoding available for in cv_bridge
        hsv_img = cv2.cvtColor(temp_img, cv2.COLOR_BGR2HSV)  # convert to HSV for KMeans clustering
        # convert PointCloud2 ROS msg to numpy array (cloud corresponds to segmented sherds image)
        # pointcloud should be dense (2D data structure corresponding to image)
        point_cloud = ros_numpy.numpify( point_cloud_msg )  
    finally:
        camera_data_lock.release()

    resized_hsv = cv2.resize(hsv_img, (600,400), interpolation = cv2.INTER_AREA)  # downsampling
    reshaped_hsv = resized_hsv.reshape(resized_hsv.shape[0]*resized_hsv.shape[1], 3)  # KMeans needs input of 2 dimensions

    # Create clusters of colors, extract into labels, and order colors in HSV
    clf = KMeans(n_clusters = req.num_colors) 
    labels = clf.fit_predict(reshaped_hsv) # numpy array of which cluster each sample belongs to (e.g. [0, 0, 2, 1, 2, 1, 0...])
    counts = Counter(labels)
    colors_in_cluster = clf.cluster_centers_ # HSV coordinates for each cluster center
    ordered_colors = [colors_in_cluster[i] for i in counts.keys()] 
    #print ("Ordered colors: ", ordered_colors)
    color_strings = [get_color_as_string(ordered_colors[i]) for i in counts.keys()]
    

    # Show pie chart of extracted colors
    if (req.show_chart):
        pie_chart_colors = [HSV2RGB(ordered_colors[i]) for i in counts.keys()]
        print ("Close pie chart to continue.")
        plt.figure(figsize = (8, 6))
        plt.pie(counts.values(), labels = color_strings, colors = pie_chart_colors)
        plt.show()

    flat_mask = get_mask_as_ROS_msg(req.num_colors, req.sim, labels, reshaped_hsv)

    res = ColorMaskResponse()
    #res.mat_z = np.average(point_cloud['z']) # get average z value of top face of mat
    """
    Actual msg data structure is a flat list, i.e., [H,S,V,H,S,V,...,H,S,V]
    Layout of color_mask ROS msg described by layout.

                Floor       Ceiling
    Color 1     [H,S,V]     [H,S,V]
    Color 2     [H,S,V]     [H,S,V]
    Color 3     [H,S,V]     [H,S,V]
    .
    .
    .
    """
    res.color_mask.data = flat_mask
    res.color_mask.layout.dim = [MultiArrayDimension(), MultiArrayDimension(), MultiArrayDimension()]
    
    res.color_mask.layout.dim[0].label = "color"
    res.color_mask.layout.dim[0].size = req.num_colors
    res.color_mask.layout.dim[0].stride = req.num_colors*2*3
    res.color_mask.layout.dim[1].label = "floor_ceiling"
    res.color_mask.layout.dim[1].size = 2
    res.color_mask.layout.dim[1].stride = 2*3
    res.color_mask.layout.dim[2].label = "h,s,v"
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
