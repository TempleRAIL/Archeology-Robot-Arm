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
from robot_arm.msg import BoolStamped
from sensor_msgs.msg import PointCloud2, Image
from vision_msgs.msg import Detection2D, Detection2DArray
from std_msgs.msg import Float32, Int16MultiArray, MultiArrayLayout, MultiArrayDimension

bridge = CvBridge()  # OpenCV converter

# Publishers
survey_stamp_pub = rospy.Publisher('Survey_Stamp', BoolStamped, queue_size=1)
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
# go_survey()
# This function moves the arm to the surveillance position and publishes a stamped Boolean ROS message.
# inputs: none
# publications: std_msgs/BoolStamped

def go_survey():
    bot = Robot('locobot')  # initialize Locobot

    # send ee to surveillance position
    survey_position = [0.25, 0, 0.37]
    survey_orientation = [0, np.pi/3, 0]  # Euler angles
    survey_pose = {'position': np.array( survey_position ), 'orientation': np.array( survey_orientation )}
    bot.arm.set_ee_pose(**survey_pose)
    survey_status = True

    # publish timestamp of surveillance position
    msg = BoolStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'ground_plane::link'
    msg.data = survey_status    
    survey_stamp_pub.publish(msg)

##############################################################
# segment_sherds()
# This function initiates the segment_sherds ROS node. It subscribes to the /Color_Mask topic.  It also sends the arm to the surveillance position.  Finally, it subscribes to the /Color_Image (camera), /Color-Aligned_PointCloud (camera), and the /Survey_Stamped topics in sync.
# inputs: none
  
def segment_sherds():
    rospy.init_node('detect_sherd')  # initiate 'detect shard' node

    # Subscribe to Color_Mask topic
    #rospy.Subscriber("/Color_Mask", Int16MultiArray, callback_unpack_mask, queue_size = 1)
    Color_Mask_msg = rospy.wait_for_message("/Color_Mask", Int16MultiArray)
    print "Subscribed to Color_Mask."
    callback_unpack_mask( Color_Mask_msg )

    # Subscribe in sync to Color Image, Color-Aligned PointCloud, and Survey_Stamped topics	
    color_img_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    pointcloud_sub = message_filters.Subscriber("/camera/depth_registered/points", PointCloud2)  # use rs_rgbd.launch
    
    # Send arm to surveillance position, then register callback only on images received once arm has reached that position.
    go_survey()
    survey_stamp_sub = message_filters.Subscriber("/Survey_Stamp", BoolStamped)
    sync = message_filters.TimeSynchronizer([color_img_sub, pointcloud_sub, survey_stamp_sub], 1)  # may need to use ApproxTimeSync
    sync.registerCallback( callback_apply_mask )
    
    rospy.spin() # keeps Python from exiting until this node is stopped
##############################################################
# main function
    
if __name__ == '__main__':
    segment_sherds()
