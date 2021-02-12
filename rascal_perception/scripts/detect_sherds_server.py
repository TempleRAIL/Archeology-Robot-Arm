#!/usr/bin/env python

#Import Python libraries
import cv2
import numpy as np
from matplotlib import pyplot as plt
import sys
import threading

# Import ROS libraries and message types
import rospy
import ros_numpy
from cv_bridge import CvBridge, CvBridgeError  # Convert between ROS image msgs and OpenCV images
import message_filters
from sensor_msgs.msg import PointCloud2, Image
from rascal_msgs.msg import Detection3DRPY, Detection3DRPYArray
from rascal_msgs.srv import *

bridge = CvBridge()  # OpenCV converter

global image_msg, point_cloud_msg, camera_data_lock
image_msg = None
point_cloud_msg = None
camera_data_lock = threading.Lock()

def is_cv2():
    # if we are using OpenCV 2, then our cv2.__version__ will start
    # with '2.'
    return check_opencv_version("2.")
def is_cv3():
    # if we are using OpenCV 3.X, then our cv2.__version__ will start
    # with '3.'
    return check_opencv_version("3.")
def is_cv4():
    # if we are using OpenCV 3.X, then our cv2.__version__ will start
    # with '4.'
    return check_opencv_version("4.")
def check_opencv_version(major, lib=None):
    # if the supplied library is None, import OpenCV
    if lib is None:
        import cv2 as lib
        
    # return whether or not the current OpenCV version matches the
    # major version number
    return lib.__version__.startswith(major)

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
# segment_sherds(color_mask, sherds_img, non_sherds_img)
# This function segments sherds from the color image.
# inputs: rascal_msgs/SherdDetectionsRequest
# outputs: OpenCV RGB image

def segment_sherds(color_mask, sherds_img, non_sherds_img):
    """
    Structure of color_mask ROS msg:

                Floor       Ceiling
    Color 1     [H,S,V]     [H,S,V]
    Color 2     [H,S,V]     [H,S,V]
    Color 3     [H,S,V]     [H,S,V]
    .
    .
    .
    color_mask.layout.dim[0].size = num_colors
    color_mask.layout.dim[1].size = 2 (floor, ceiling)
    color_mask.layout.dim[2].size = 3 (H,S,V)
    
    """
    num_colors = color_mask.layout.dim[0].size
    stride = color_mask.layout.dim[1].stride
      
    # convert images to HSV for color-masking
    hsv_sherds = cv2.cvtColor(sherds_img, cv2.COLOR_BGR2HSV)
    if non_sherds_img is not None:
        hsv_nonsherd = cv2.cvtColor(non_sherds_img, cv2.COLOR_BGR2HSV)

    # initialize masks outside of loop as all zeros
    bg_mask_sherds = np.zeros((hsv_sherds.shape[0], hsv_sherds.shape[1]),np.uint8)
    bg_mask_nonsherd = np.copy(bg_mask_sherds)  # super important - assignment statements in Python do not copy objects, they create bindings between a target and an object.

    # build background mask(s) by applying every color range in succession 
    for i in range(0, num_colors):
        floor = [color_mask.data[0+(i*stride)], color_mask.data[1+(i*stride)], color_mask.data[2+(i*stride)]]
        ceiling = [color_mask.data[3+(i*stride)], color_mask.data[4+(i*stride)], color_mask.data[5+(i*stride)]]

        this_mask = cv2.inRange( hsv_sherds, np.array(floor), np.array(ceiling) )  # sherds blocked out
        bg_mask_sherds += this_mask

        # Debugging
        """   
        plt.imshow(bg_mask_sherds)
        plt.title("bg_mask_sherds iteration {}".format(i))
        plt.show()
        """

        if non_sherds_img is not None:
            that_mask = cv2.inRange( hsv_nonsherd, np.array(floor), np.array(ceiling) )  # non-sherds blocked out
            bg_mask_nonsherd += that_mask

    fg_mask_sherds = cv2.bitwise_not(bg_mask_sherds)  # invert so that sherds are allowed through

    sherds_mask = cv2.morphologyEx(fg_mask_sherds, cv2.MORPH_OPEN, kernel=np.ones((3,3),np.uint8))  # final sherds mask
    rgb = sherds_img[:, :, ::-1]  # flip to RGB for display
    segmented_sherds = cv2.bitwise_and(rgb, rgb, mask=sherds_mask) # apply mask that allows sherds through

    # Debugging        
    """
    plt.imshow(sherds_mask)
    plt.title("Final Sherds Mask")
    plt.show()
    """

    # Debugging: Display original image and segmented sherds
    """
    plt.subplot(121),plt.imshow(rgb)
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(segmented_sherds)
    plt.title('Segmented Objects'), plt.xticks([]), plt.yticks([])
    plt.show()
    """

    if non_sherds_img is not None:
        fg_mask_nonsherd = cv2.bitwise_not(bg_mask_nonsherd)  # invert so that non-sherds are allowed through
        fg_mask_nonsherd = cv2.dilate(fg_mask_nonsherd, kernel=np.ones((3,3),np.uint8), iterations=20) # dilate non-sherd areas
        nonsherd_mask = cv2.bitwise_not(fg_mask_nonsherd) # re-invert to block out non-sherds: final non-sherds mask

        # Debugging        
        """
        plt.subplot(121),plt.imshow(non_sherds_img[:, :, ::-1])
        plt.title('Non-sherds Image'), plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(nonsherd_mask)
        plt.title('Non-sherds Mask'), plt.xticks([]), plt.yticks([])
        plt.show()
        """
        segmented_sherds = cv2.bitwise_and(segmented_sherds, segmented_sherds, mask=nonsherd_mask) # apply mask to remove non-sherds
 
    # Debugging: Display original image and segmented sherds
    """
    if non_sherds_img is not None:
        plt.subplot(121),plt.imshow(rgb)
        plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(segmented_sherds)
        plt.title('Segmented Objects'), plt.xticks([]), plt.yticks([])
        plt.show()
    """
    return segmented_sherds

##############################################################
# locate_sherds(sherds_image, bgnd_image, points, header)
# This function draws bounding boxes around segmented sherds and publishes -in meters- their x,y,z center coordinates, widths, and heights.  It also publishes rotation angles in radians, optimized for the robot end effector.
# inputs: sensor_msgs/Image, sensor_msgs/PointCloud2
# publications: rascal_msgs.msg/Detection3DArrayRPY custom ROS message

def locate_sherds(sherds_image, points, header):
    point_map = points 

    img_height, img_width, _ = sherds_image.shape

    # Convert sherds_image to grayscale and find contours
    gray_image = cv2.cvtColor(np.array(sherds_image), cv2.COLOR_BGR2GRAY) # convert to grayscale
    if is_cv2() or is_cv4():
        contours, _ = cv2.findContours( gray_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE ) # find all contours
    elif is_cv3():
        _, contours, _ = cv2.findContours( gray_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE ) # find all contours

    #Debugging
    ROS_DEBUG("Found %d objects in this frame - may or may not all be sherds." % (len(contours)))
    # exclude boxes smaller than a minimum area
    use_min_area = False
    min_area = 0.0006  # sq. meters (roughly 1 sq. inch)
    #print ("Recognizing only rectangles larger than %f sq. meters as sherds" % (min_area) )

    # Construct Detection3DRPYArray custom ROS message to contain all valid bounding boxes
    detections = Detection3DRPYArray()
    detections.header = header    # meta-data

    # For each detected contour, find bounding box
    for cnt in contours:
        rect = cv2.minAreaRect(cnt) # [(x center, y center), (width, height), (rotation) in pixels]
        box = cv2.boxPoints(rect) # [4x1 array of tuples: coordinates of vertices in pixels]
        box = np.int0(box)

        # check that contour does not meet edges of image frame
        in_frame = True
        for corner in box:
            if (corner[0] <= 0) or (corner[0] >= img_width-1) or (corner[1] <= 0) or (corner[1] >= img_height-1):
                in_frame = False
            if not in_frame:
                break

        # Debugging
        """
        print box
        rospy.logwarn('Box in frame: {}'.format(in_frame))
        """

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
        elif angle < -90:
            grip_angle = angle+ 80
        else:
            grip_angle = angle
        # get real-world dimensions of boxes using point map
        # x,y coordinates of bounding box center in meters
        x_center = points[row_center_pos][col_center_pos][0]
        y_center = points[row_center_pos][col_center_pos][1]
        z_center = points[row_center_pos][col_center_pos][2]
        """
        print("x_center as found in pointcloud: ", x_center, "meters.")
        print("y_center as found in pointcloud: ", y_center, "meters.")
        print("z_center as found in pointcloud: ", z_center, "meters.")
        """
        # endpoints of bounding box minor and major axes in pixel coordinates
        # pixel coordinates of width endpoints
        col_width_end1 = int( col_center_pos + np.trunc( 0.5*width_px*np.cos( np.radians(grip_angle) ) ) )
        col_width_end2 = int( col_center_pos - np.trunc( 0.5*width_px*np.cos( np.radians(grip_angle) ) ) )
        row_width_end1 = int( row_center_pos + np.trunc( 0.5*width_px*np.sin( np.radians(grip_angle) ) ) )
        row_width_end2 = int( row_center_pos - np.trunc( 0.5*width_px*np.sin( np.radians(grip_angle) ) ) )
        # pixel coordinates of height endpoints
        col_height_end1 = int( col_center_pos + np.trunc( 0.5*height_px*np.cos( np.radians(grip_angle) ) ) )
        col_height_end2 = int( col_center_pos - np.trunc( 0.5*height_px*np.cos( np.radians(grip_angle) ) ) )
        row_height_end1 = int( row_center_pos + np.trunc( 0.5*height_px*np.sin( np.radians(grip_angle) ) ) )
        row_height_end2 = int( row_center_pos - np.trunc( 0.5*height_px*np.sin( np.radians(grip_angle) ) ) )

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
        width_meter = np.sqrt( (x_width_end1-x_width_end2)**2+(y_width_end1-y_width_end2)**2 )
        height_meter = np.sqrt( (x_height_end1-x_height_end2)**2+(y_height_end1-y_height_end2)**2 )
        
        if not (not in_frame) or use_min_area or (use_min_area and width_meter*height_meter >= min_area):
            # print("This sherd's bounding box: " + str(rect))
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

            # Debugging: draw, display, and print details of bounding boxes around sherds
            """
            #Convert original RGB image to np.array to draw contours as boxes
            # Extract (x,y) coordinates of box corners for drawing rectangles, starting at "lowest" corner (largest y-coordinate) and moving CW. Height is distance between 0th and 1st corner. Width is distance between 1st and 2nd corner.
            sherd_contours = cv2.drawContours( np.array(sherds_image), [box], 0, (255,0,0), 3 )

            plt.figure("Figure 2")
            plt.imshow(sherd_contours)
            plt.title("Bounding Box around Sherd")
            plt.show()

            print("Row of center is " + str(row_center_pos))
            print("Col of center is " + str(col_center_pos))
            print("Width endpoints in pixels: (%f, %f) and (%f, %f)." % (col_width_end1, row_width_end1, col_width_end2, row_width_end2) )
            print("Height endpoints in pixels: (%f, %f) and (%f, %f)." % (col_height_end1, row_height_end1, col_height_end2, row_height_end2) )
            print("Center coordinates in meters: (%f, %f)." % (x_center, y_center) )
            print("Width in meters: %f." % (width_meter) )
            print("Height in meters:  %f." % (height_meter) )
            print("Gripper rotation angle is %f degrees." % grip_angle)
            """
    return detections

##############################################################
# detect_sherds_callback(req)
# This function segments sherds from the color image and returns their 3D coordinates
# inputs: rascal_msgs/SherdDetectionsRequest
# outputs: rascal_msgs/SherdDetectionsResponse

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
    rospy.logwarn("req.subtract_background value is: {}".format(req.subtract_background))
    if req.subtract_background:
        non_sherds_img = bridge.imgmsg_to_cv2(req.background_image, "bgr8")  # BGR OpenCV image
    else:
        non_sherds_img = None
    # Segment sherds using HSV color mask and non-sherds mask
    sherds_image = segment_sherds(req.color_mask, img, non_sherds_img)
    # Locate sherds in image
    points = np.zeros( (point_cloud.shape[0], point_cloud.shape[1], 3) )
    points[:,:,0] = point_cloud['x']
    points[:,:,1] = point_cloud['y']
    points[:,:,2] = point_cloud['z']
    res = SherdDetectionsResponse()
    res.detections = locate_sherds(sherds_image, points, header)
    return res

##############################################################
# detect_sherds_server()
# This function initiates the detect_sherds ROS node. It subscribes to the /Color_Mask topic.  It also sends the arm to the surveillance position.  Finally, it subscribes to the /Color_Image (camera), /Color-Aligned_PointCloud (camera), and the /Survey_Stamped topics in sync.
# inputs: none
  
def detect_sherds_server():
    rospy.init_node('detect_sherds_server')  # initiate 'detect sherds' node

    # Subscribe in sync to Color Image and Color-Aligned PointCloud
    color_img_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    
    # use rs_rgbd.launch with physical RealSense camera
    pointcloud_sub = message_filters.Subscriber("/camera/depth_registered/points", PointCloud2)
    
    sync = message_filters.ApproximateTimeSynchronizer([color_img_sub, pointcloud_sub], 1, 0.1, allow_headerless = True)
    sync.registerCallback( camera_data_callback )
    
    detect_sherds_server = rospy.Service('detect_sherds_server', SherdDetections, detect_sherds_callback)
 
    rospy.spin() # keeps Python from exiting until this node is stopped
    
##############################################################
# main function
if __name__ == '__main__':
    detect_sherds_server()
