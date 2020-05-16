#!/usr/bin/env python

#Import Python libraries
import cv2
import math
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.image as mpimg
#import pyrealsense2 as rs	# Intel RealSense cross-platform open-source API

# Import ROS libraries and message types
import message_filters
import rospy
import ros_numpy
from cv_bridge import CvBridge, CvBridgeError  # Convert between ROS image msgs and OpenCV images
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection2D, Detection2DArray
from std_msgs.msg import Float32, Int16MultiArray, MultiArrayLayout, MultiArrayDimension

bridge = CvBridge()  # OpenCV converter

# Publishers
boundbox_pub = rospy.Publisher('Bounding_Boxes', Detection2DArray, queue_size=1)

##############################################################
# callback_boundbox( segmented_img, pointcloud )
# This function draws bounding boxes around segmented sherds and publishes -in meters- their x,y center coordinates, widths, and heights.  It also publishes rotation angles in radians, optimized for the robot end effector.
# inputs: sensor_msgs/Image, sensor_msgs/PointCloud2
# publications: vision_msgs/Detection2DArray ROS message

def callback_boundbox(segmented_img, pointcloud):

    print "Triggered callback_boundbox."

    # convert PointCloud2 ROS msg to numpy array (cloud corresponds to segmented sherds image)
    points_numpify = ros_numpy.numpify( pointcloud )  # pointcloud should be dense (2D data structure corresponding to image)
    
    point_map = np.zeros( (points_numpify.shape[0], points_numpify.shape[1], 3) )
    point_map[:,:,0] = points_numpify['x']
    point_map[:,:,1] = points_numpify['y']
    point_map[:,:,2] = points_numpify['z']
   
    # convert image to OpenCV format
    segmented_sherds = bridge.imgmsg_to_cv2(segmented_img, desired_encoding="passthrough")
    
    # Display segmented objects
    plt.imshow(segmented_sherds)
    plt.title('Segmented Sherds'), plt.xticks([]), plt.yticks([])

    plt.show()  

    # Find contours in gray 'segmented_sherds' image.
    gray_sherds = cv2.cvtColor(np.array(segmented_sherds), cv2.COLOR_BGR2GRAY)
    _, contours, _ = cv2.findContours( gray_sherds, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )
 
    print("Found %d objects in this frame - may or may not all be sherds." % (len(contours)))

    # exclude boxes smaller than a minimum area
    #min_area = 0.0006  # sq. meters (roughly 1 sq. inch)
    min_area = False
    #print ("Recognizing only rectangles larger than %f sq. meters as sherds" % (min_area) )

    # Construct Detection2DArray ROS message to contain all valid bounding boxes
    BoundingBoxes_Array = Detection2DArray()
    BoundingBoxes_Array.header = segmented_img.header	# meta-data
        
    # For each detected contour, find bounding box
    for cnt in contours: 
        rect = cv2.minAreaRect(cnt)
    
	# col (x), row (y) of bounding box centers [pixel coordinates]
	col_center_pos = rect[0][0]
	row_center_pos = rect[0][1]

    	# width and height [pixels] and rotation angle [deg] of bounding boxes
    	width_px = rect[1][0]
    	height_px = rect[1][1]
	angle = rect[2]

    	# transform box rotation angle to read between long side and y-axis
    	if width_px < height_px:
	    angle += 180
	else:
	    angle += 90

    	# transform rotation angle so that gripper never rotates more than 90deg CW (-) or CCW (+)
	# gripper positioned to only need to open along the smallest side of the bounding box
	if angle > 90:
    	    grip_angle = angle-180
	else:
    	    grip_angle = -angle

	# get real-world dimensions of boxes using point map
    	# x,y coordinates of bounding box center in meters
    	x_center = point_map[int(row_center_pos)][int(col_center_pos)][0]
    	y_center = point_map[int(row_center_pos)][int(col_center_pos)][1]
    	print("x_center as found in pointcloud: ", x_center, "meters.")
    	print("y_center as found in pointcloud: ", y_center, "meters.")
    	    
	# endpoints of bounding box minor and major axes in pixel coordinates
    	    
    	# pixel coordinates of width endpoints
	col_width_end1 = col_center_pos + math.trunc( 0.5*width_px*np.cos( np.radians(grip_angle) ) )
    	col_width_end2 = col_center_pos - math.trunc( 0.5*width_px*np.cos( np.radians(grip_angle) ) )
    	row_width_end1 = row_center_pos + math.trunc( 0.5*width_px*np.sin( np.radians(grip_angle) ) )
    	row_width_end2 = row_center_pos - math.trunc( 0.5*width_px*np.sin( np.radians(grip_angle) ) )

    	# pixel coordinates of height endpoints
	col_height_end1 = col_center_pos + math.trunc( 0.5*height_px*np.cos( np.radians(grip_angle) ) )
    	col_height_end2 = col_center_pos - math.trunc( 0.5*height_px*np.cos( np.radians(grip_angle) ) )
	row_height_end1 = row_center_pos + math.trunc( 0.5*height_px*np.sin( np.radians(grip_angle) ) )
    	row_height_end2 = row_center_pos - math.trunc( 0.5*height_px*np.sin( np.radians(grip_angle) ) )

    	try:
    	    # meter coordinates of width endpoints
    	    x_width_end1 = point_map[int(row_width_end1)][int(col_width_end1)][0]
    	    y_width_end1 = point_map[int(row_width_end1)][int(col_width_end1)][1]
    	    x_width_end2 = point_map[int(row_width_end2)][int(col_width_end2)][0]
    	    y_width_end2 = point_map[int(row_width_end2)][int(col_width_end2)][1]

  	    # meter coordinates of height endpoints
    	    x_height_end1 = point_map[int(row_height_end1)][int(col_height_end1)][0]
    	    y_height_end1 = point_map[int(row_height_end1)][int(col_height_end1)][1]
    	    x_height_end2 = point_map[int(row_height_end2)][int(col_height_end2)][0]
    	    y_height_end2 = point_map[int(row_height_end2)][int(col_height_end2)][1]
    	except IndexError:
    	    continue

    	# bounding box width and height in meters
    	width_meter = math.sqrt( (x_width_end1-x_width_end2)**2+(y_width_end1-y_width_end2)**2 )
    	height_meter = math.sqrt( (x_height_end1-x_height_end2)**2+(y_height_end1-y_height_end2)**2 )

    	#if width_meter*height_meter >= min_area:
    	if not min_area:
	    
	    print("This sherd's bounding box: " + str(rect))
            
            # Extract (x,y) coordinates of box corners for drawing rectangles, starting at "lowest" corner (largest y-coordinate) and 	    	    moving CW. Height is distance between 0th and 1st corner. Width is distance between 1st and 2nd corner.
	    box = cv2.boxPoints(rect)
            box = np.int0(box)
            print("Box corner coordinates: " + str(box))

            # Debugging: draw bounding boxes around sherds
            # Convert original RGB image to np.array to draw contours as boxes
	    sherd_contours = cv2.drawContours( np.array(segmented_sherds), [box], 0, (255,0,0), 3 )
	    	
	    # Debugging
	    print("Row of center is " + str(row_center_pos))
	    print("Col of center is " + str(col_center_pos))
    	    print("Width endpoints in pixels: (%f, %f) and (%f, %f)." % (col_width_end1, row_width_end1, col_width_end2, row_width_end2) )
    	    print("Height endpoints in pixels: (%f, %f) and (%f, %f)." % (col_height_end1, row_height_end1, col_height_end2, row_height_end2) )
    	    print("Center coordinates in meters: (%f, %f)." % (x_center, y_center) )
    	    print("Width in meters: %f." % (width_meter) )
    	    print("Height in meters:  %f." % (height_meter) )
	    print("Gripper rotation angle is %f degrees." % grip_angle)

	    plt.figure("Figure 2")
	    plt.imshow(sherd_contours) 
	    # plt.imshow(img[:, :, ::-1]) 
	    plt.title("Bounding Box around Sherd")
	    plt.show()
  
	    # Construct a Detection2D() msg to add this bounding box to BoundingBoxes_Array
	    BoundingBoxes_Element = Detection2D()
            BoundingBoxes_Element.header = BoundingBoxes_Array.header
            BoundingBoxes_Element.bbox.center.x = x_center
            BoundingBoxes_Element.bbox.center.y = y_center
            BoundingBoxes_Element.bbox.center.theta = np.radians(grip_angle)	# angle converted to radians
            BoundingBoxes_Element.bbox.size_x = width_meter
            BoundingBoxes_Element.bbox.size_y = height_meter
            BoundingBoxes_Array.detections.append(BoundingBoxes_Element)
       
	else:
            pass
  
    # Publish the BoundingBoxes_Array message to the 'Bounding_Boxes' topic
    boundbox_pub.publish(BoundingBoxes_Array)
    print "BoundingBoxes_Array msg published to Bounding_Boxes."



##############################################################
# locate_sherds()
# This function initiates the locate_sherds ROS node. It subscribes to an image of segmented sherds and its corresponding 3D pointcloud.  It invokes a callback on that image to determine the location, size, and orientation of sherds via bounding boxes.
# inputs: none

  
def locate_sherds():
    rospy.init_node('locate_sherds')  # initiate node
    # Subscribe to Segmented_Sherds and Sherds_PointCloud topics
    segmented_sherds_sub = message_filters.Subscriber("/Segmented_Sherds", Image)
    print "Subscribed to segmented sherds image."
    sherds_pointcloud_sub = message_filters.Subscriber("/Sherds_PointCloud", PointCloud2)
    print "Subscribed to pointcloud of sherds."
    sync = message_filters.ApproximateTimeSynchronizer([segmented_sherds_sub, sherds_pointcloud_sub], 1, 1e10) # big slop time
    sync.registerCallback( callback_boundbox )
    
    rospy.spin() # keeps Python from exiting until this node is stopped

##############################################################
# main function
    
if __name__ == '__main__':
    locate_sherds()
