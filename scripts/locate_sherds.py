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

# Publishers
boundbox_pub = rospy.Publisher('Bounding_Boxes', Detection2DArray, queue_size=1)

##############################################################
# callback_mm_per_pixel(tf_msg)
# This function calculates a mm/pixel conversion factor using similar triangles. W/D = P/F, where W = true width of object; D = distance from camera lens to object; P = apparent width of object in pixels; F = focal length of camera 
# inputs: Float ROS msg (distance from camera lens to ground)
# outputs: mm_per_pixel conversion factor

def callback_get_scale(tf_msg):

    global scale_factor

    # use camera lens distance from ground instead of distance from sherd center

    D_mm = tf_msg.data	# magnitude of vector pointing from camera lens to ground
    F_mm = 1.93	# focal length from Intel RealSense D435 color camera (D400 series datasheet)

    scale_factor = D_mm/F_mm
    print ("Scale factor: %f" % scale_factor)

##############################################################
# callback_boundbox(segmented_img)
# This function extracts the contours of segmented sherds and publishes size, orientation, and location of bounding boxes in a ROS 2DDetectionArray message.
# inputs: sensor_msgs/Image
# publications: vision_msgs/Detection2DArray ROS message

def callback_boundbox(segmented_img):

    print "Triggered callback_boundbox."
    
    segmented_sherds = bridge.imgmsg_to_cv2(segmented_img, desired_encoding="passthrough")  # BGR OpenCV image
    #segmented_sherds = bgr[:, :, ::-1]  # flip to RGB for display
    

    # Display segmented objects
    plt.imshow(segmented_sherds)
    plt.title('Segmented Sherds'), plt.xticks([]), plt.yticks([])

    plt.show()  

    # Find contours in gray 'segmented_sherds' image.
    gray_sherds = cv2.cvtColor(np.array(segmented_sherds), cv2.COLOR_BGR2GRAY)
    _, contours, _ = cv2.findContours( gray_sherds, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )

 
    # Set minimum rectangle area to be recognized as sherd
    #min_height_px = 30
    #min_width_px = 30

    #print("Found %d objects in this frame - may or may not all be sherds." % (len(contours)))
    #print ("Recognizing only rectangles larger than %d by %d pixels as sherds." % (min_height_px, min_width_px))

    # Construct Detection2DArray ROS message to contain all valid bounding boxes
    BoundingBoxes_Array = Detection2DArray()
    BoundingBoxes_Array.header = segmented_img.header	# meta-data
        
    # For each detected contour, find bounding box
    for cnt in contours: 
        rect = cv2.minAreaRect(cnt)
    
	# real-world dimensions of boxes in mm
	pixel_size = 0.0014	# [mm] 1.4 micrometers per pixel on OV2740 sensor
    	width = scale_factor*pixel_size*rect[1][0]
    	height = scale_factor*pixel_size*rect[1][1]
    	#height_px = rect[1][1]
    	#width_px = rect[1][0]

        # If rectangle >= specified area in mm2
    	if width*height >= 645:
    	# if height_px >= min_height_px and width_px >= min_width_px:
	    
	    print("This sherd's bounding box: " + str(rect))
            
            # Extract (x,y) coordinates of box corners in order to draw rectangles, starting at "lowest" corner (largest y-coordinate) and moving CW
	    # Note: height is distance between 0th and 1st corner.  Width is distance between 1st and 2nd corner.
	    box = cv2.boxPoints(rect)
            box = np.int0(box)
            print("Box corner coordinates: " + str(box))

            # Debugging: draw bounding boxes around sherds
            # Convert original RGB image to np.array to draw contours as boxes
	    sherd_contours = cv2.drawContours( np.array(segmented_sherds), [box], 0, (255,0,0), 3 )

	    # x,y of bounding box centers
	    x_center_pos = rect[0][0]
	    y_center_pos = rect[0][1]

	    # transform rotation angle to read between long side and y-axis
	    angle = rect[2]
	    if width < height:
    	    #if width_px < height_px:
	    	angle += 180
	    else:
	    	angle += 90

	    # transform rotation angle so that gripper never rotates more than 90deg CW or CCW
	    # gripper positioned to only need to open along the smallest side of the bounding box
	    if angle > 90:
	    	grip_angle = angle-180
	    else:
		grip_angle = angle
	    	
	    # Debugging: print 
	    print("x center is " + str(x_center_pos))
	    print("y center is " + str(y_center_pos))
	    print ("%f wide and %f high." % (width, height))
	    print("Gripper rotation angle is %f degrees." % grip_angle)

	    plt.figure("Figure 2")
	    plt.imshow(sherd_contours) 
	    # plt.imshow(img[:, :, ::-1]) 
	    plt.title("Bounding Box around Sherd")
	    plt.show()
  
	    # Construct a Detection2D() msg to add this bounding box to BoundingBoxes_Array
	    BoundingBoxes_Element = Detection2D()
            BoundingBoxes_Element.header = BoundingBoxes_Array.header
            BoundingBoxes_Element.bbox.center.x = x_center_pos
            BoundingBoxes_Element.bbox.center.y = y_center_pos
            BoundingBoxes_Element.bbox.center.theta = np.radians(grip_angle)	# angle converted to radians
            BoundingBoxes_Element.bbox.size_x = width
            BoundingBoxes_Element.bbox.size_y = height
            BoundingBoxes_Array.detections.append(BoundingBoxes_Element)
       
	else:
            pass
  
    # Publish the BoundingBoxes_Array message to the 'Bounding_Boxes' topic
    boundbox_pub.publish(BoundingBoxes_Array)
    print "BoundingBoxes_Array msg published to Bounding_Boxes."


##############################################################
# locate_sherds()
# This function initiates the locate_sherds ROS node. It subscribes to an image of segmented sherds from the Segmented_Sherds topic.  It invokes a callback on that image to determine the location, size, and orientation of sherds via bounding boxes.
# inputs: none
  
def locate_sherds():
    rospy.init_node('locate_sherds')  # initiate node

    # Subscribe to topic publishing Camera from Ground
    camera_from_ground = rospy.wait_for_message("/Dummy_Height", Float32)
    print "Subscribed to Dummy Height topic."
    callback_get_scale(camera_from_ground)

    # Subscribe to Segmented_Sherds topic
    segmented_sherds = rospy.wait_for_message("/Segmented_Sherds", Image)
    print "Subscribed to segmented sherds image."
    callback_boundbox( segmented_sherds )
 
    rospy.spin() # keeps Python from exiting until this node is stopped

##############################################################
# main function
    
if __name__ == '__main__':
    locate_sherds()
