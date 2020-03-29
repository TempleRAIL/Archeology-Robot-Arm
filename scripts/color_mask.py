#!/usr/bin/env python

# Import Python libraries
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import numpy as np
import cv2
from collections import Counter
from skimage.color import rgb2lab, rgb2hsv, deltaE_cie76
import os
import sys

# Import ROS libraries and message types
import rospy
from cv_bridge import CvBridge, CvBridgeError  # Convert between ROS image msgs and OpenCV images
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension

bridge = CvBridge()  # OpenCV converter

# Create a global PUBLISHER to 'Color_Mask' topic
color_mask_pub = rospy.Publisher('Color_Mask', Int16MultiArray, latch=True, queue_size=1) 


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
# callback_make_mask(img_msg)
# This function detects and extracts a user-defined number of the most dominant colors in an image.
# inputs: sensor_msgs/Image ROS message
# publications: Float32MultiArray containg colors

def callback_make_mask(img_msg):
    number_colors = 5 # total number colors to extract, starting with most dominant
    show_chart = 1 # 1 to show chart, else 0

    print 'Extracting top ', number_colors, ' colors.'
    
    img = bridge.imgmsg_to_cv2(img_msg, "bgr8")  # BGR OpenCV image
    rgb = img[:, :, ::-1]  # flip to RGB
    resized_rgb = cv2.resize(rgb, (600,400), interpolation = cv2.INTER_AREA)
    reshaped_rgb = resized_rgb.reshape(resized_rgb.shape[0]*resized_rgb.shape[1], 3)  # KMeans needs input of 2 dimensions


    # Create clusters of colors, extract into labels, and order colors in RGB and hex
    clf = KMeans(n_clusters = number_colors) 
    labels = clf.fit_predict(reshaped_rgb)
    counts = Counter(labels)
    center_colors = clf.cluster_centers_
    ordered_colors = [center_colors[i] for i in counts.keys()] 
    print ("Ordered colors: ", ordered_colors)
    hex_colors = [RGB2HEX(ordered_colors[i]) for i in counts.keys()]

    if (show_chart):
    	print 'Close pie chart to continue.'    
    	plt.figure(figsize = (8, 6))
        plt.pie(counts.values(), labels = hex_colors, colors = hex_colors)
    	plt.show()

    # Create image from ordered colors
    img_to_cvt = np.zeros((number_colors,1,3),np.uint8)
    
    for i in range( len(ordered_colors) ): 
    	for j in range( len(ordered_colors[i]) ):
    	    img_to_cvt[i][0][j] = ordered_colors[i][j]
    
    hsv = rgb2hsv(img_to_cvt)  # convert image to HSV
    
    print ("HSV image of top colors:", hsv)    

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

    print "Floor", floor
    print "Ceiling", ceiling

    # Combine floor and ceiling into flat list for ROS message
    flat_mask = floor + ceiling

    print 'Flat mask: ', flat_mask

    # Construct message for color mask
    Color_Mask = Int16MultiArray()
    Color_Mask.data = flat_mask
    Color_Mask.layout.dim = [MultiArrayDimension(), MultiArrayDimension(), MultiArrayDimension()]
    
    Color_Mask.layout.dim[0].label = "height"
    Color_Mask.layout.dim[0].size = 2
    Color_Mask.layout.dim[0].stride = 3*1*2
    Color_Mask.layout.dim[1].label = "width"
    Color_Mask.layout.dim[1].size = 1
    Color_Mask.layout.dim[1].stride = 3*1
    Color_Mask.layout.dim[2].label = "channels"
    Color_Mask.layout.dim[2].size = 3
    Color_Mask.layout.dim[2].stride = 3

    print "Message constructed."

    # Publish color mask
    color_mask_pub.publish(Color_Mask)


##############################################################
# color_mask()
# This function initiates ROS node that subscribes to the camera topic and invokes callback_make_mask on the Image message.  Video feed should be the empty mat.
# inputs: none
 
def color_mask():
    rospy.init_node('color_mask')
    #Image_msg = rospy.wait_for_message("/image_publisher_1584754889388915488/image_raw", Image)
    Image_msg = rospy.wait_for_message("/camera/color/image_raw", Image)
    callback_make_mask( Image_msg )
    rospy.spin() # simply keeps python from exiting until this node is stopped

##############################################################
# main function
    
if __name__ == '__main__':
    color_mask()
