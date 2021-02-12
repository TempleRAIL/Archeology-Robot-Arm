#!/usr/bin/env python

#Import Python libraries
import cv2
from sklearn.cluster import KMeans
from collections import Counter
from skimage.color import rgb2lab, rgb2hsv, deltaE_cie76
import colorsys
import numpy as np
from matplotlib import pyplot as plt
import sys
import threading

# Import ROS libraries and message types
import rospy
from cv_bridge import CvBridge, CvBridgeError  # Convert between ROS image msgs and OpenCV images
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import Image
from rascal_msgs.msg import Detection3DRPY, Detection3DRPYArray
from rascal_msgs.srv import *



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



class ColorMaskServer:

    def __init__(self):
        # Initialize data members
        self.bridge = CvBridge()  # OpenCV converter
        self.image_msg = None
        self.camera_data_lock = threading.Lock()
        # Get ROS parameters
        self.sim = rospy.get_param('sherd_states/sim')
        self.show_chart = rospy.get_param('sherd_states/show_pie_chart', False)
        self.value_min = rospy.get_param('sherd_states/value_min', 17.85)
        self.value_max = rospy.get_param('sherd_states/value_max', 255)
        # Subscribe to color image
        self.color_img_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.logdebug("Subscribed to color image.")
        # Create service server
        color_mask_server = rospy.Service('color_mask', ColorMask, self.color_mask_callback)

    ##############################################################
    # camera_data_callback(msg)
    # This function saves the image
    # inputs: sensor_msgs/Image
    def image_callback(self, im_msg):
        self.camera_data_lock.acquire()
        try:
            self.image_msg = im_msg
        finally:
            self.camera_data_lock.release()

    ##############################################################
    # get_mask_as_ROS_msg(num_colors, cluster_labels, twoD_image)
    # This function generates a ROS-msg friendly list of floors and ceilings defining a color mask for each color extracted by KMeans
    # inputs: number of colors, cluster_labels = output of KMeans fit_predict method, 2D_image = OpenCV HSV image used to generate color mask, reshaped as a 2D array
    # returns: flat list of floors and ceilings for each color mask

    def get_mask_as_ROS_msg(self, num_colors, cluster_labels, twoD_image):
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
        
        for i in range(num_colors):
            cluster_indices = np.where(cluster_labels == i)	# in array of cluster labels, find every position belonging to ith cluster
            colors_in_cluster = twoD_image[cluster_indices]  # cross-reference: get array of all HSV colors belonging to cluster i

            if not self.sim:
                huethresh = np.std(colors_in_cluster[:,0])*0.01*180 # OpenCV hue format
                satthresh = np.std(colors_in_cluster[:,1])*0.01*255 # OpenCV saturation
            else:
                huethresh = 9  # OpenCV hue format
                satthresh = 71.4 #OpenCV saturation format

            minhue = colors_in_cluster[i][0] - huethresh
            maxhue = colors_in_cluster[i][0] + huethresh

            minsat = colors_in_cluster[i][1] - satthresh
            maxsat = colors_in_cluster[i][1] + satthresh

            floor = [minhue, minsat, self.value_min]	# lower bound of mask for ith color
            ceiling = [maxhue, maxsat, self.value_max]	# upper bound of mask ith color
            flat_mask += (floor+ceiling)  # combine floors and ceilings of all colors into flat list for ROS message

        return flat_mask
      
    ##############################################################
    # color_mask_callback(req)
    # This function is the callback for the color mask service
    # inputs: rascal_msgs/ColorMaskRequest
    # returns: rascal_msgs/ColorMaskResponse 

    def color_mask_callback(self, req):
        rospy.loginfo("Extracting {} colors for this station's color mask".format(req.num_colors))
        # Check that images have been received
        if self.image_msg is None:
            rospy.logerr('No sensor data received yet')
            return
        # Convert image data
        self.camera_data_lock.acquire()
        try:
            temp_img = self.bridge.imgmsg_to_cv2(self.image_msg, "bgr8")  # BGR OpenCV image - no HSV encoding available for in cv_bridge
            hsv_img = cv2.cvtColor(temp_img, cv2.COLOR_BGR2HSV)  # convert to HSV for KMeans clustering
        finally:
            self.camera_data_lock.release()
        
        # Reshape image
        resized_hsv = cv2.resize(hsv_img, (600,400), interpolation = cv2.INTER_AREA)  # downsampling
        reshaped_hsv = resized_hsv.reshape(resized_hsv.shape[0]*resized_hsv.shape[1], 3)  # KMeans needs input of 2 dimensions

        # Create clusters of colors, extract into labels, and order colors in HSV
        clf = KMeans(n_clusters = req.num_colors) 
        labels = clf.fit_predict(reshaped_hsv) # numpy array of which cluster each sample belongs to (e.g. [0, 0, 2, 1, 2, 1, 0...])
        
        # Show pie chart of extracted colors
        if self.show_chart:
            counts = Counter(labels)
            colors_in_cluster = clf.cluster_centers_ # HSV coordinates for each cluster center
            ordered_colors = [colors_in_cluster[i] for i in counts.keys()] 
            rospy.logdebug("Ordered colors: ", ordered_colors)
            color_strings = [get_color_as_string(ordered_colors[i]) for i in counts.keys()]
            
            pie_chart_colors = [HSV2RGB(ordered_colors[i]) for i in counts.keys()]
            print ("Close pie chart to continue.")
            plt.figure(figsize = (8, 6))
            plt.pie(counts.values(), labels = color_strings, colors = pie_chart_colors)
            plt.show()

        # Create response
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
        res = ColorMaskResponse()
        
        res.color_mask.data = self.get_mask_as_ROS_msg(req.num_colors, labels, reshaped_hsv)
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

        return res


##############################################################
# main function
    
if __name__ == '__main__':
    rospy.init_node('color_mask_server')
    cms = ColorMaskServer()
    rospy.spin() # simply keeps python from exiting until this node is stopped