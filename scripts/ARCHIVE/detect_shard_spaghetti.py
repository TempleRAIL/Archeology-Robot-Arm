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

# Global publishers to Bounding_Boxes and Segmented_Sherds_Img
boundbox_pub = rospy.Publisher('Bounding_Boxes', Detection2DArray, queue_size=1)
segmented_sherds_pub = rospy.Publisher('Segmented_Sherds_Img', Image, queue_size=1)


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
# callback_boundbox(color_img)
# This function segments sherds from the image by masking the background color.  It also extracts their contours and publishes bounding boxes in a ROS 2DDetectionArray message.
# inputs: color mask floor, color mask ceiling, sensor_msgs/Image
# publications: vision_msgs/Detection2DArray ROS message

def callback_boundbox(color_img):
    if not has_color_mask:
    	return 'Failed to unpack color mask.'

    print "Triggered callback_boundbox."
    
    img = bridge.imgmsg_to_cv2(color_img, "bgr8")  # BGR OpenCV image
    rgb = img[:, :, ::-1]  # flip to RGB for display
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # convert to HSV for color masking
   
    bg_mask = cv2.inRange( hsv, np.array(floor), np.array(ceiling) )
    fg_mask = cv2.bitwise_not(bg_mask)

    fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8)) 

    # Apply final mask to show sherds and black out background
    objects = cv2.bitwise_and(rgb, rgb, mask=fg_mask)

    # Display original image and segmented objects
    plt.subplot(121),plt.imshow(rgb)
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(objects)
    plt.title('Sherds'), plt.xticks([]), plt.yticks([])

    plt.show()  

    # Find contours in gray 'objects' image. Finding contours in 'color_mask' image returns false positives.
    gray_objects = cv2.cvtColor(np.array(objects), cv2.COLOR_BGR2GRAY)
    _, contours, _ = cv2.findContours( gray_objects, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )

    # Set minimum rectangle area to be recognized as sherd
    min_height_px = 30
    min_width_px = 30

    print("Found %d objects in this frame - may or may not all be sherds." % (len(contours)))
    print ("Recognizing only rectangles larger than %d by %d pixels as sherds." % (min_height_px, min_width_px))

    # Construct Detection2DArray ROS message to contain all valid bounding boxes
    BoundingBoxes_Array = Detection2DArray()
    BoundingBoxes_Array.header = color_img.header	# meta-data
        
    # For each detected contour, find bounding box
    for cnt in contours: 
        rect = cv2.minAreaRect(cnt)
    
	# box dimensions in pixels
        height_px = rect[1][1]
	width_px = rect[1][0]

        # If rectangle >= specified area...
	if height_px >= min_height_px and width_px >= min_width_px:
	    
	    print("This sherd's bounding box: " + str(rect))
            
            # Extract (x,y) coordinates of box corners in order to draw rectangles, starting at "lowest" corner (largest y-coordinate) and moving CW
	    # Note: height is distance between 0th and 1st corner.  Width is distance between 1st and 2nd corner.
	    box = cv2.boxPoints(rect)
            box = np.int0(box)
            print("Box corner coordinates: " + str(box))

            # Debugging: draw bounding boxes around sherds
            # Convert original RGB image to np.array to draw contours as boxes
	    rgb_contours = cv2.drawContours( np.array(rgb), [box], 0, (255,0,0), 3 )

	    # x,y of bounding box centers
	    x_center_pos = rect[0][0]
	    y_center_pos = rect[0][1]

	    # transform rotation angle to read between long side and y-axis
	    angle = rect[2]
	    if width_px < height_px:
	    	angle += 180
	    else:
	    	angle += 90

	    # transform rotation angle so that gripper never rotates more than 90deg CW or CCW
	    if angle > 90:
	    	grip_angle = angle-180
	    else:
		grip_angle = angle
	    	
	    # Debugging: print 
	    print("x center is " + str(x_center_pos))
	    print("y center is " + str(y_center_pos))
	    print("Box rotation angle is %f." % angle)
	    print("Gripper rotates %f degrees." % grip_angle)

	    plt.figure("Figure 2")
	    plt.imshow(rgb_contours) 
	    # plt.imshow(img[:, :, ::-1]) 
	    plt.title("Bounding Box around Sherd")
	    plt.show()
  
	    # Construct a Detection2D() msg to add this bounding box to BoundingBoxes_Array
	    BoundingBoxes_Element = Detection2D()
            BoundingBoxes_Element.header = BoundingBoxes_Array.header
            BoundingBoxes_Element.bbox.center.x = x_center_pos
            BoundingBoxes_Element.bbox.center.y = y_center_pos
            BoundingBoxes_Element.bbox.center.theta = np.radians(grip_angle)	# angle converted to radians
            BoundingBoxes_Element.bbox.size_x = width_px
            BoundingBoxes_Element.bbox.size_y = height_px
            BoundingBoxes_Array.detections.append(BoundingBoxes_Element)
       
	else:
            pass
  
    # Publish the BoundingBoxes_Array message to the 'Bounding_Boxes' topic
    boundbox_pub.publish(BoundingBoxes_Array)
    print "BoundingBoxes_Array msg published to Bounding_Boxes."


##############################################################
# callback_ground_from_cam(depth_img)
# This function calculates the vertical distance of the ground from the camera. It takes the image of segmented sherds produced in callback_boundbox and extracts the coordinates of every black pixel (the background). It then averages all the depths associated with those pixel coordinates from the corresponding aligned-to-color depth image.
# inputs: image of segmented objects, sensor_msgs/Image
# publications: vision_msgs/Detection2DArray ROS message

def callback_ground_from_cam(depth_img):
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

    bg_depths_list = []

    for coordinate in bg_pixels_list:
    	row = coordinate[0]
	col = coordinate[1]
    	bg_depths_list.append(depth_map[row,col])

    ground_from_cam = sum(bg_depths_list)/len(bg_depths_list)

    print ('Average distance of ground from camera: %f mm.' % ground_from_cam)

##############################################################
# detect_shard()
# This function initiates the detect_shard ROS node. It subscribes to 1) the Color_Mask topic and 2) the camera topic.  It invokes callbacks on messages from both topics.
# inputs: none

  
def detect_shard():
    rospy.init_node('detect_shard')  # initiate 'detect shard' node

    # Subscribe to Color_Mask topic
    #rospy.Subscriber("/Color_Mask", Int16MultiArray, callback_unpack_mask, queue_size = 1)
    Color_Mask_msg = rospy.wait_for_message("/Color_Mask", Int16MultiArray)
    print "Subscribed to Color_Mask."
    callback_unpack_mask( Color_Mask_msg )

    # Subscribe to Camera_Info topic
    #rospy.Subscriber("/camera/camera_info", CameraInfo, callback_get_conversion, queue_size = 1)
    #Camera_Info_msg = rospy.wait_for_message("/camera/camera_info", CameraInfo)
    #callback_get_conversion( Camera_Info_msg )

    # Subscribe to Color Image topic
    rospy.Subscriber("/camera/color/image_raw", Image, callback_boundbox, queue_size = 1)
    #rospy.Subscriber("/image_publisher_1584141470620349444/image_raw", Image, callback_boundbox, queue_size = 1)
    print "Subscribed to image."

    # Subscribe to Aligned Depth topic
    depth_map = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image)
    callback_ground_from_cam( depth_map )
    print "Subscribed to aligned depth map."

    #ts = message_filters.ApproximateTimeSynchronizer([color_mask_sub, image_sub], 10, 0.1, allow_headerless=True)
    #ts.registerCallback(callback_boundbox)
    #print "Registered callback on both messages."

    rospy.spin() # keeps Python from exiting until this node is stopped

##############################################################
# main function
    
if __name__ == '__main__':
    detect_shard()
