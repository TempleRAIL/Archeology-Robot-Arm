#!/usr/bin/env python

#Import Python libraries
import cv2
import os
import numpy as np
import math
from skimage.draw import polygon
import numpy as np
import networkx as nx
from scipy.spatial import distance_matrix
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components

from matplotlib import pyplot as plt
import matplotlib.image as mpimg
import sys
import threading
import math

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
# segment_sherds(color_mask, sherds_img, non_sherds_img)
# This function segments sherds from the color image.
# inputs: robot_arm/SherdDetectionsRequest
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
# get_contours(img)
# This function converts the image to grayscale and extracts contours.
# inputs: OpenCV image
# returns: contours (array of arrays)

def get_contours(img):
    gray = cv2.cvtColor(np.array(img), cv2.COLOR_BGR2GRAY) # convert to grayscale
    _, contours, _ = cv2.findContours( gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )

    return contours

##############################################################
# in_frame_only(contours)
# This function filters out contours which run out of the image frame.
# inputs: contours (array of arrays) from cv2.findContours
# returns: in-frame contours only (array of arrays)

def in_frame_only(img, contours):
    #Debugging
    """
    print("Found {} contours total - may or may not all be sherds.".format( len(contours) ))
    """
    img_height, img_width, _ = img.shape

    for cnt in contours:
        rect = cv2.minAreaRect(cnt) # [(x center, y center), (width, height), (rotation) in pixels]
        box = cv2.boxPoints(rect) # [4x1 array of tuples: coordinates of vertices in pixels]
        box = np.int0(box)

        # Debugging: draw and display bounding boxes around contours
        # Convert original RGB image to np.array to draw contours as boxes
        
        sherd_contours = cv2.drawContours( np.array(img), [box], 0, (255,0,0), 3 )

        plt.figure("Figure 2")
        plt.imshow(sherd_contours)
        plt.title("Bounding Box around Sherd")
        plt.show()
        
        # remove contour from array if its edges meet the image frame
        in_frame = True        
        for corner in box:
            if (corner[0] <= 0) or (corner[0] >= img_width-1) or (corner[1] <= 0) or (corner[1] >= img_height-1):
                in_frame = False
                contours.remove(cnt)
                break
    #Debugging
    """
    print("Found {} contours in frame.".format( len(contours) ))
    """
    
    return contours

##############################################################
# get_top_sherd()
# This function filters out contours which run out of the image frame.
# inputs: contours (array of arrays) from cv2.findContours
# returns: in-frame contours only (array of arrays)

def get_top_sherd(sherds_image, point_map, eps, if_contours):
    img_height, img_width, _ = sherds_image.shape
    print("Img_height and img_width are {} and {}.".format(img_height, img_width))

    # calculate difference matrices
    row_diff = np.diff(point_map, axis=0)
    col_diff = np.diff(point_map, axis=1)

    #print("col_diff has shape {} and looks like: {}".format(col_diff.shape,col_diff))

    # initialize distance matrices
    row_dist = np.zeros( (row_diff.shape[0],row_diff.shape[1]) ) 
    col_dist = np.zeros( (col_diff.shape[0],col_diff.shape[1]) )

    # build distance matrices
    for i in range(0,row_diff.shape[0]):
        for j in range(0,row_diff.shape[1]):
            row_dist[i,j] = math.sqrt(np.sum(np.square(row_diff[i,j][2])))

    for i in range(0,col_diff.shape[0]):
        for j in range(0,col_diff.shape[1]):
            col_dist[i,j] = math.sqrt(np.sum(np.square(col_diff[i,j][2])))

    #print("Distance matrix for columns: {}".format(col_dist))

    top_sherd_candidates = {}
    counter = 0

    for cnt in if_contours:
        rect = cv2.minAreaRect(cnt) # [(x center, y center), (width, height), (rotation) in pixels]
        box = cv2.boxPoints(rect) # [4x1 array of tuples: coordinates of vertices in pixels]
        box = np.int0(box)

        # assuming that each contour represents a pile of sherds, get coordinates of all pixels in sherd pile
        r = np.array( [vertex[1] for vertex in box] ) # row coordinates of box vertices
        c = np.array( [vertex[0] for vertex in box] ) # column coordinates of box vertices
        rr, cc = polygon(r,c) # row and column coordinates of all pixels inside bounding box
        temp_list = zip(rr,cc) # merge into one list of tuple coordinates

        sherd_pile = []
        for coord in temp_list:
            if not (coord[0] >= img_height or coord[1] >= img_width): # some box vertices can fall outside of image frame
                sherd_pile.append(coord)

        #print("First 4 pixels in sherd_pile: {}".format(sherd_pile[0:4]))

        # add pixels in sherd_pile as nodes to a networkx graph
        G = nx.Graph()
        G.add_nodes_from(sherd_pile) # create node for each pixel in sherd_pile

        #print("Done adding nodes from sherd_pile. Length of sherd_pile is {} and number of nodes in graph is {}.".format( len(sherd_pile), G.number_of_nodes() ))

        # add connections (edges) between nodes based on distance      
        # [i,j] of dist matrix in row_dist is distance between [i,j] and [i+1, j] in image
        # [i,j] of dist matrix in row_dist is distance between [i,j] and [i, j+1] in image

        for pixel in sherd_pile:
            i,j = pixel[0], pixel[1]
            try:
                if row_dist[i,j] <= eps: 
                    G.add_edge( (i,j), (i+1,j) )
                if col_dist[i,j] <= eps: 
                    G.add_edge( (i,j), (i,j+1) )
            except IndexError:
                pass

        # generate adjacency matrix from graph
        adj_matrix = nx.adjacency_matrix(G, nodelist=sherd_pile) # SciPy adjacency matrix, ordered using sherd_pile
        #print("SciPy adjacency matrix: {}".format(adj_matrix[0:20]))

        # get number of sherds (i.e., groups of connected pixels) and list of labels (sherd 0, sherd 1, etc.)
        # list of labels maps onto pixel coordinates in sherd_pile.
        num_sherds, sherd_labels = connected_components(adj_matrix, return_labels=True)

        # dictionary: {sherd 0: [connected pixels], sherd 1: [connected pixels] ...}
        sherds_and_pixels = {}
        for i in range(0,num_sherds):
            indices = [j for j in range(len(sherd_labels)) if sherd_labels[j] == i]  # indices of all pixels labeled as sherd i
            sherds_and_pixels[i] = [sherd_pile[index] for index in indices] #[np.array(sherd_pile[index]) for index in indices]

        print("Number of sherds: {}. {} items in sherds_and_pixels.".format( num_sherds, len(sherds_and_pixels.items()) ))
        
        # identify top sherd in this pile, (highest avg. z coordinate)
        # we index into point map using coordinates of connected pixels
        max_avg_z = None
        for sherd, pixel_list in sherds_and_pixels.items():
            xyz_coords = np.array( [point_map[pix[0]][pix[1]] for pix in pixel_list] )
            temp_z = np.average(xyz_coords[:,2])
            if not max_avg_z or temp_z > max_avg_z: # update top sherd candidate from this pile
                top_sherd = sherd
                max_avg_z = temp_z
            
        print("Top sherd for this pile is {} with average z of {}.".format(sherd, max_avg_z))

        # this dictionary contains the top sherd from each pile
        top_sherd_candidates[counter] = { 'max_avg_z': max_avg_z, 'pixels': sherds_and_pixels[top_sherd] }
        counter+=1
    
    print( "Keys in top_sherd_candidates: {}".format(top_sherd_candidates.keys()) )

    avg_z = None
    for sherd, data in top_sherd_candidates.items():
        temp_z = data['max_avg_z']
        if not avg_z or temp_z > avg_z: # update top sherd across all piles
            top_sherd = sherd
            avg_z = temp_z
  
    # create mask: top sherd pixels --> foreground; else --> background
    top_sherd_mask = np.zeros( (sherds_image.shape[0], sherds_image.shape[1]), np.uint8 )
    for pix in top_sherd_candidates[top_sherd]['pixels']:
        top_sherd_mask[pix[0]][pix[1]] = 255

    # mask out every pixel NOT in top sherd --> final image
    top_sherd_img = cv2.bitwise_and(sherds_image, sherds_image, mask=top_sherd_mask)

    # Debugging:
    plt.subplot(121),plt.imshow(top_sherd_mask)
    plt.title('Top Sherd Mask'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(top_sherd_img)
    plt.title('Top Sherd, Segmented'), plt.xticks([]), plt.yticks([])
    plt.show()
    
    return top_sherd_img, avg_z

##############################################################
# locate_sherds(color_seg_img, point_map, handle_overlap, eps, header)
# This function draws bounding boxes around segmented sherds and publishes in [meters] their x,y,z center coordinates, widths, and heights.  It also publishes rotation angles in radians, optimized for the robot end effector.
# inputs: sensor_msgs/Image, sensor_msgs/PointCloud2
# publications: robot_arm.msg/Detection3DArrayRPY custom ROS message

def locate_sherds(color_seg_img, point_map, handle_overlap, eps, header):

    # Construct Detection3DRPYArray custom ROS message to contain all valid bounding boxes around sherds
    detections = Detection3DRPYArray()
    detections.header = header    # meta-data

    contours = get_contours(color_seg_img)

    if not handle_overlap: # if not viewing overlapping sherds
        if_contours = in_frame_only(color_seg_img, contours) # list of in-frame contours only
    else:
        top_sherd_img, avg_z = get_top_sherd(color_seg_img, point_map, eps, contours) # segmented image of top sherd + z-height
        contours = get_contours(top_sherd_img)
        if_contours = in_frame_only(top_sherd_img, contours)

    # exclude boxes smaller than a minimum area
    use_min_area = True
    min_area = 0.0006  # [sq. meters]
    #print ("Recognizing only rectangles larger than %f sq. meters as sherds" % (min_area) )

    #TODO exclude outermost contour? (the sherd container)

    # For each detected contour, disaggregate sherd pile, then find bounding box around each sherd
    for cnt in if_contours:
        rect = cv2.minAreaRect(cnt) # [(x center, y center), (width, height), (rotation) in pixels]
        box = cv2.boxPoints(rect) # [4x1 array of tuples: coordinates of vertices in pixels]
        box = np.int0(box)       

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
        x_center = point_map[row_center_pos][col_center_pos][0] # x, y coordinates of bounding box center in [m]
        y_center = point_map[row_center_pos][col_center_pos][1]
        if handle_overlap: # if viewing overlapping sherds
            z_center = avg_z # end effector of robot will descend to this height for grasping
        else:
            z_center = point_map[row_center_pos][col_center_pos][2]

        """
        print("x_center as found in pointcloud: ", x_center, "meters.")
        print("y_center as found in pointcloud: ", y_center, "meters.")
        print("z_center as found in pointcloud: ", z_center, "meters.")
        """
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
        area = width_meter*height_meter
        
        if not (use_min_area and area <= min_area) or not use_min_area:
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
            
            #Convert original RGB image to np.array to draw contours as boxes
            # Extract (x,y) coordinates of box corners for drawing rectangles, starting at "lowest" corner (largest y-coordinate) and moving CW. Height is distance between 0th and 1st corner. Width is distance between 1st and 2nd corner.
            """            
            sherd_contours = cv2.drawContours( np.array(color_seg_img), [box], 0, (255,0,0), 3 )

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
    rospy.logwarn("req.subtract_background value is: {}".format(req.subtract_background))
    if req.subtract_background:
        non_sherds_img = bridge.imgmsg_to_cv2(req.background_image, "bgr8")  # BGR OpenCV image
    else:
        non_sherds_img = None
    # Segment sherd piles using HSV color mask and non-sherds mask
    color_seg_img = segment_sherds(req.color_mask, img, non_sherds_img)
    # Locate sherds in image
    point_map = np.zeros( (point_cloud.shape[0], point_cloud.shape[1], 3) )
    point_map[:,:,0] = point_cloud['x']
    point_map[:,:,1] = point_cloud['y']
    point_map[:,:,2] = point_cloud['z']
    res = SherdDetectionsResponse()
    res.detections = locate_sherds(color_seg_img, point_map, req.handle_overlap, req.cluster_threshold, header)

    return res

##############################################################
# detect_sherds_pile_server()
# This function initiates the detect_sherds_pile ROS node. It subscribes to the 
# subscriptions: /Color_Mask; synced [/Color_Image (camera), /Color-Aligned_PointCloud (camera), /Survey_Stamped]
  
def detect_sherds_pile_server():
    rospy.init_node('detect_sherds_pile_server')  # initiate 'detect sherds' node

    # Subscribe in sync to Color Image and Color-Aligned PointCloud
    color_img_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    print("Subscribed to color image.")

    # use rs_rgbd.launch with physical RealSense camera
    pointcloud_sub = message_filters.Subscriber("/camera/depth_registered/points", PointCloud2)
    print("Subscribed to point cloud.")

    sync = message_filters.ApproximateTimeSynchronizer([color_img_sub, pointcloud_sub], 1, 0.1, allow_headerless = True)
    sync.registerCallback( camera_data_callback )
    
    detect_sherds_pile_server = rospy.Service('detect_sherds_pile_server', SherdDetections, detect_sherds_callback)
 
    rospy.spin() # keeps Python from exiting until this node is stopped
    
##############################################################
# main function
if __name__ == '__main__':
    detect_sherds_pile_server()
