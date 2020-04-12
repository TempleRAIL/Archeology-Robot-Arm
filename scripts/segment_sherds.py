#!/usr/bin/env python

#Import Python libraries
import cv2
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.image as mpimg
import sys

# Import ROS libraries and message types
import rospy
from cv_bridge import CvBridge, CvBridgeError  # Convert between ROS image msgs and OpenCV images
import message_filters
from sensor_msgs.msg import PointCloud2, Image
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32, Int16MultiArray, MultiArrayLayout, MultiArrayDimension

bridge = CvBridge()  # OpenCV converter

# Publishers
segmented_sherds_pub = rospy.Publisher('Segmented_Sherds', Image, latch=True, queue_size=1)
pointcloud_pub = rospy.Publisher('Sherds_PointCloud', PointCloud2, latch=True, queue_size=1)


##############################################################
# callback_unpack_mask(MultiArray_msg)
# This function unpacks the message from the /Color_Mask topic into the upper- and lower-bounds of an HSV color mask.
# inputs: std_msgs/Int16MultiArray (or similar)

def callback_unpack_mask(MultiArray_msg):
    print "Triggered callback_unpack_mask."

    # Unpack HSV mask into global variables
    global floor, ceiling, has_color_mask
    floor = [MultiArray_msg.data[0], MultiArray_msg.data[1], MultiArray_msg.data[2]]	
    ceiling = [MultiArray_msg.data[3], MultiArray_msg.data[4], MultiArray_msg.data[5]]
    has_color_mask = True


##############################################################
# callback_apply_mask(color_img, pointcloud)
# This function segments sherds from the color image by applying the previously-unpacked color mask to  The pointcloud taken simultaneously with the color_img is simply published
# inputs: color mask floor, color mask ceiling, sensor_msgs/Image
# publications: vision_msgs/Detection2DArray, sensor_msgs/PointCloud2

def callback_apply_mask(color_img, pointcloud):

    if not has_color_mask:
    	return 'Failed to unpack color mask.'
    	sys.exit()

    print "Triggered callback_apply_mask."
    
    img = bridge.imgmsg_to_cv2(color_img, "bgr8")  # BGR OpenCV image
    rgb = img[:, :, ::-1]  # flip to RGB for display
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # convert to HSV for color masking
   
    bg_mask = cv2.inRange( hsv, np.array(floor), np.array(ceiling) )
    fg_mask = cv2.bitwise_not(bg_mask)

    fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8)) 

    # Apply final mask to show sherds and black out background
    segmented_sherds = cv2.bitwise_and(rgb, rgb, mask=fg_mask)

    # Display original image and segmented objects
    plt.subplot(121),plt.imshow(rgb)
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(segmented_sherds)
    plt.title('Sherds'), plt.xticks([]), plt.yticks([])

    plt.show()

    # Convert segmented_sherds from OpenCV to ROS Image message
    segmented_sherds = bridge.cv2_to_imgmsg(segmented_sherds, encoding="passthrough")

    segmented_sherds_pub.publish(segmented_sherds)
    print "Segmented sherds image published to Segmented_Sherds."
    pointcloud_pub.publish(pointcloud)
    print "Pointcloud of sherds published to Sherds_PointCloud." 


##############################################################
# segment_sherds()
# This function initiates the segment_sherds ROS node. It subscribes to  & invokes callbacks on1) the Color_Mask topic and 2) the depth camera's color image and pointcloud in sync.
# inputs: none

  
def segment_sherds():
    rospy.init_node('detect_sherd')  # initiate 'detect shard' node

    # Subscribe to Color_Mask topic
    #rospy.Subscriber("/Color_Mask", Int16MultiArray, callback_unpack_mask, queue_size = 1)
    Color_Mask_msg = rospy.wait_for_message("/Color_Mask", Int16MultiArray)
    print "Subscribed to Color_Mask."
    callback_unpack_mask( Color_Mask_msg )

    # Subscribe in sync to Color Image and Color-Aligned PointCloud topics
    color_img_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    pointcloud_sub = message_filters.Subscriber("/camera/depth_registered/points", PointCloud2)  # this topic from rs_rgbd.launch
    sync = message_filters.TimeSynchronizer([color_img_sub, pointcloud_sub], 1)
    sync.registerCallback( callback_apply_mask )
    #color_img_sub.sub.unregister()
    #pointcloud_sub.sub.unregister()
    
    # Subscribe to Color Image topic
    #Color_Image = rospy.wait_for_message("/camera/color/image_raw", Image)
    #print "Subscribed to image."
    #callback_apply_mask( Color_Image )
    #rospy.Subscriber("/image_publisher_1584141470620349444/image_raw", Image, callback_segment, queue_size = 1)

    rospy.spin() # keeps Python from exiting until this node is stopped
##############################################################
# main function
    
if __name__ == '__main__':
    segment_sherds()
