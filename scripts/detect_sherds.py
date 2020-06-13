#!/usr/bin/env python

#Import Python libraries
import cv2
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.image as mpimg
import sys
import threading

# Import ROS libraries and message types
import rospy
import ros_numpy
from cv_bridge import CvBridge, CvBridgeError  # Convert between ROS image msgs and OpenCV images
import message_filters
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
# segment_sherds(color_mask, img)
# This function segments sherds from the color image.
# inputs: robot_arm/SherdDetectionsRequest
# outputs: cv2 RGB image

def segment_sherds(color_mask, img):
    # Unpack HSV mask
    # TODO get working for more than 1 color
    floor = [color_mask.data[0], color_mask.data[1], color_mask.data[2]]    
    ceiling = [color_mask.data[3], color_mask.data[4], color_mask.data[5]]
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # convert to HSV for color masking
   
    bg_mask = cv2.inRange( hsv, np.array(floor), np.array(ceiling) )
    fg_mask = cv2.bitwise_not(bg_mask)

    fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8)) 

    # Apply final mask to show sherds and black out background
    rgb = img[:, :, ::-1]  # flip to RGB for display
    sherds_image = cv2.bitwise_and(rgb, rgb, mask=fg_mask)

    # Display original image and segmented objects
    plt.subplot(121),plt.imshow(rgb)
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(sherds_image)
    plt.title('Sherds'), plt.xticks([]), plt.yticks([])
    plt.show()
    
    return sherds_image


##############################################################
# locate_sherds(sherds_image, points )
# This function draws bounding boxes around segmented sherds and publishes -in meters- their x,y,z center coordinates, widths, and heights.  It also publishes rotation angles in radians, optimized for the robot end effector.
# inputs: sensor_msgs/Image, sensor_msgs/PointCloud2
# publications: robot_arm.msg/Detection3DArrayRPY custom ROS message

def locate_sherds(sherds_image, points, header):
    # Find contours in gray 'sherds_image' image.
    gray_image = cv2.cvtColor(np.array(sherds_image), cv2.COLOR_BGR2GRAY)
    _, contours, _ = cv2.findContours( gray_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )

    print("Found %d objects in this frame - may or may not all be sherds." % (len(contours)))
    # exclude boxes smaller than a minimum area
    use_min_area = False
    min_area = 0.0006  # sq. meters (roughly 1 sq. inch)
    #print ("Recognizing only rectangles larger than %f sq. meters as sherds" % (min_area) )

    # Construct Detection3DRPYArray custom ROS message to contain all valid bounding boxes
    detections = Detection3DRPYArray()
    detections.header = header    # meta-data

    # For each detected contour, find bounding box
    for cnt in contours:
        rect = cv2.minAreaRect(cnt)
        # col (x), row (y) of bounding box centers [pixel coordinates]
        col_center_pos = int(rect[0][0])
        row_center_pos = int(rect[0][1])
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
        x_center = points[row_center_pos][col_center_pos][0]
        y_center = points[row_center_pos][col_center_pos][1]
        z_center = points[row_center_pos][col_center_pos][2]
        print("x_center as found in pointcloud: ", x_center, "meters.")
        print("y_center as found in pointcloud: ", y_center, "meters.")
        print("z_center as found in pointcloud: ", z_center, "meters.")
           
        # endpoints of bounding box minor and major axes in pixel coordinates
        # pixel coordinates of width endpoints
        col_width_end1 = int( col_center_pos + math.trunc( 0.5*width_px*np.cos( np.radians(grip_angle) ) ) )
        col_width_end2 = int( col_center_pos - math.trunc( 0.5*width_px*np.cos( np.radians(grip_angle) ) ) )
        row_width_end1 = int( row_center_pos + math.trunc( 0.5*width_px*np.sin( np.radians(grip_angle) ) ) )
        row_width_end2 = int( row_center_pos - math.trunc( 0.5*width_px*np.sin( np.radians(grip_angle) ) ) )
        # pixel coordinates of height endpoints
        col_height_end1 = int( col_center_pos + math.trunc( 0.5*height_px*np.cos( np.radians(grip_angle) ) ) )
        col_height_end2 = int( col_center_pos - math.trunc( 0.5*height_px*np.cos( np.radians(grip_angle) ) ) )
        row_height_end1 = int( row_center_pos + math.trunc( 0.5*height_px*np.sin( np.radians(grip_angle) ) ) )
        row_height_end2 = int( row_center_pos - math.trunc( 0.5*height_px*np.sin( np.radians(grip_angle) ) ) )

        try:
            # meter coordinates of width endpoints
            x_width_end1 = point_map[row_width_end1][col_width_end1][0]
            y_width_end1 = point_map[row_width_end1][col_width_end1][1]
            x_width_end2 = point_map[row_width_end2][col_width_end2][0]
            y_width_end2 = point_map[row_width_end2][col_width_end2][1]
            # meter coordinates of height endpoints
            x_height_end1 = point_map[row_height_end1][col_height_end1][0]
            y_height_end1 = point_map[row_height_end1][col_height_end1][1]
            x_height_end2 = point_map[row_height_end2][col_height_end2][0]
            y_height_end2 = point_map[row_height_end2][col_height_end2][1]
        except IndexError:
            continue

        # bounding box width and height in meters
        width_meter = math.sqrt( (x_width_end1-x_width_end2)**2+(y_width_end1-y_width_end2)**2 )
        height_meter = math.sqrt( (x_height_end1-x_height_end2)**2+(y_height_end1-y_height_end2)**2 )
        if not use_min_area or (use_min_area and width_meter*height_meter >= min_area):
            print("This sherd's bounding box: " + str(rect))
            # Construct a Detection3DRPY() msg to add this bounding box to detections
            detection = Detection3DRPY()
            detection.header = detections.header
            detection.bbox.center.position.x = x_center
            detection.bbox.center.position.y = y_center
            detection.bbox.center.position.z = z_center
            detection.bbox.center.roll = np.radians(grip_angle)  # radians
            detection.bbox.size.x = width_meter
            detection.bbox.size.y = height_meter
            detections.detections.append(detection)

            # Debugging: draw bounding boxes around sherds
            # Convert original RGB image to np.array to draw contours as boxes
            # Extract (x,y) coordinates of box corners for drawing rectangles, starting at "lowest" corner (largest y-coordinate) and moving CW. Height is distance between 0th and 1st corner. Width is distance between 1st and 2nd corner.
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            sherd_contours = cv2.drawContours( np.array(sherds_image), [box], 0, (255,0,0), 3 )
            plt.figure("Figure 2")
            plt.imshow(sherd_contours)
            plt.title("Bounding Box around Sherd")
            plt.show()
            # Debugging
            print("Row of center is " + str(row_center_pos))
            print("Col of center is " + str(col_center_pos))
            print("Width endpoints in pixels: (%f, %f) and (%f, %f)." % (col_width_end1, row_width_end1, col_width_end2, row_width_end2) )
            print("Height endpoints in pixels: (%f, %f) and (%f, %f)." % (col_height_end1, row_height_end1, col_height_end2, row_height_end2) )
            print("Center coordinates in meters: (%f, %f)." % (x_center, y_center) )
            print("Width in meters: %f." % (width_meter) )
            print("Height in meters:  %f." % (height_meter) )
            print("Gripper rotation angle is %f degrees." % grip_angle)
    return detections


##############################################################
# detect_sherds_callback(req)
# This function segments sherds from the color image and returns their 3D coordinates
# inputs: robot_arm/SherdDetectionsRequest
# outputs: robot_arm/SherdDetectionsResponse

def detect_sherds_callback(req):
    global image_msg, point_cloud_msg, camera_data_lock
    # Check to make sure that data exists
    if image_msg is None or point_cloud_msg is None:
        rospy.logerr('No camera data yet.')
        sys.exit()
        return
    # Convert latest messages to format required for processing
    camera_data_lock.acquire()
    try:
        img = bridge.imgmsg_to_cv2(image_msg, "bgr8")  # BGR OpenCV image
        # convert PointCloud2 ROS msg to numpy array (cloud corresponds to segmented sherds image)
        point_cloud = ros_numpy.numpify( point_cloud_msg )  # pointcloud should be dense (2D data structure corresponding to image)
        header = image_msg.header
    finally:
        camera_data_lock.release()
    # Segment sherds using HSV color mask
    sherds_image = segment_sherds(req.color_mask, img)
    # Locate sherds in image
    points = np.zeros( (point_cloud.shape[0], point_cloud.shape[1], 3) )
    points[:,:,0] = point_cloud['x']
    points[:,:,1] = point_cloud['y']
    points[:,:,2] = point_cloud['z']
    res = SherdDetectionsResponse()
    res.detections = locate_sherds(sherds_image, points, header)
    return res

##############################################################
# detect_sherds()
# This function initiates the detect_sherds ROS node. It subscribes to the /Color_Mask topic.  It also sends the arm to the surveillance position.  Finally, it subscribes to the /Color_Image (camera), /Color-Aligned_PointCloud (camera), and the /Survey_Stamped topics in sync.
# inputs: none
  
def detect_sherds():
    rospy.init_node('detect_sherds')  # initiate 'detect sherds' node

    # Subscribe in sync to Color Image and Color-Aligned PointCloud
    color_img_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    print("Subscribed to color image.")

    pointcloud_sub = message_filters.Subscriber("/camera/depth_registered/points", PointCloud2)  # use rs_rgbd.launch
    print("Subscribed to point cloud.")

    sync = message_filters.ApproximateTimeSynchronizer([color_img_sub, pointcloud_sub], 1, 0.1, allow_headerless = True)
    sync.registerCallback( camera_data_callback )
    
    detect_sherds_server = rospy.Service('detect_sherds', SherdDetections, detect_sherds_callback)
 
    rospy.spin() # keeps Python from exiting until this node is stopped
    
##############################################################
# main function
if __name__ == '__main__':
    detect_sherds()
