#!/usr/bin/env python

#Import Python libraries
import cv2
import math
import numpy as np
import os
from scipy import ndimage as ndi
from skimage.color import label2rgb
from skimage.feature import peak_local_max
from skimage.morphology import disk
from skimage.segmentation import watershed
from skimage import data
from skimage.filters import rank
from skimage.util import img_as_ubyte

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
# show_box(img, box)
# This function...
# inputs:
# returns:

def show_box(img, box):
    sherd_box = cv2.drawContours( img, [box], 0, (255,0,0), 3 )
    print("Press any key to continue.")
    plt.figure()
    plt.imshow(sherd_box)
    plt.title("Bounding Box around Sherd")
    plt.show()

    cv2.waitKey(0) # wait until any key is pressed

##############################################################
# def in_frame_only(img)
# This function filters out contours which run out of the image frame.
# inputs: contours (array of arrays) from cv2.findContours
# returns: in-frame contours only (array of arrays)

def in_frame_only(img):
    img_height, img_width, _ = img.shape
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) # convert to grayscale
    _, contours, _ = cv2.findContours( gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )

    #Debugging
    """
    print("Found {} contours total - may or may not all be sherds.".format( len(contours) ))
    """

    for cnt in contours:
        rect = cv2.minAreaRect(cnt) # [(x center, y center), (width, height), (rotation) in pixels]
        box = cv2.boxPoints(rect) # [4x1 array of tuples: coordinates of vertices in pixels]
        box = np.int0(box)
       
        # remove contour from array if its edges meet the image frame
        in_frame = True        
        for corner in box:
            print('Corner: {}'.format(corner))
            if (corner[0] <= 0) or (corner[0] >= img_width) or (corner[1] <= 0) or (corner[1] >= img_height):
                in_frame = False
                contours.remove(cnt)
                break
    #Debugging
    """
    print("Found {} contours in frame.".format( len(contours) ))
    """
    
    return np.array(contours)

##############################################################
# within(list_of_arrays, row_limits, col_limits)
# This function asseses whether a group of connected pixels form an internal contour relative to user-defined mins and maxes for row and column coordinates.
# inputs: list [numpy.array(all_row_coords), numpy.array(all_col_coords)], tuple (row_min, row_max), tuple (col_min, col_max)
# returns: bool

def within(list_of_arrays, row_limits, col_limits):
    #TODO write code to catch wrong argument types 
    row_min, row_max = row_limits[0], row_limits[1]
    col_min, col_max = col_limits[0], col_limits[1]
    if any(list_of_arrays[0] <= row_min) or any(list_of_arrays[0] >= row_max) or any(list_of_arrays[1] <= col_min) or any(list_of_arrays[1] >= col_max):
        return False
    else:
        return True

##############################################################
# gen_lut()
# This function...
# inputs:
# returns: 

def gen_lut():
  """
  Generate a label colormap compatible with opencv lookup table, based on
  Rick Szelski algorithm in `Computer Vision: Algorithms and Applications`,
  appendix C2 `Pseudocolor Generation`.
  :Returns:
    color_lut : opencv compatible color lookup table
  """
  tobits = lambda x, o: np.array(list(np.binary_repr(x, 24)[o::-3]), np.uint8)
  arr = np.arange(256)
  r = np.concatenate([np.packbits(tobits(x, -3)) for x in arr])
  g = np.concatenate([np.packbits(tobits(x, -2)) for x in arr])
  b = np.concatenate([np.packbits(tobits(x, -1)) for x in arr])
  return np.concatenate([[[b]], [[g]], [[r]]]).T

##############################################################
# labels2rgb(labels, lut)
# This function...
# inputs:
# returns:

def labels2rgb(labels, lut):
  """
  Convert a label image to an rgb image using a lookup table
  :Parameters:
    labels : an image of type np.uint8 2D array
    lut : a lookup table of shape (256, 3) and type np.uint8
  :Returns:
    colorized_labels : a colorized label image
  """
  return cv2.LUT(cv2.merge((labels, labels, labels)), lut)


##############################################################
# get_z(z_values, thresh)
# This function takes the average and standard of all z values, then returns the mean of those values that fall within a certain range of the overall average.
# inputs: flat array of z-values, 
# returns: average z-height for topography

def get_z(z_values,thresh=0.005):
    avg_z, std_dev = np.average(z_values), np.std(z_values)
    if std_dev < thresh:
        thresh = std_dev
    floor, ceil = avg_z-thresh, avg_z+thresh
    first_dev = np.array( [value for value in z_values if floor<=value<=ceil] )
    z=np.average(first_dev)
    
    return z


##############################################################
# get_top_sherd(labels)
# This function filters out contours which run out of the image frame.
# inputs: contours (array of arrays) from cv2.findContours
# returns: in-frame contours only (array of arrays)

def get_top_sherd(labels, color_seg_img, point_map):
    img_height, img_width = labels.shape
    frame_row_lims = (0,img_height-1)
    frame_col_lims = (0,img_width-1)
    unique_labels = np.unique(labels)
    unique_labels = np.delete(unique_labels, np.where(unique_labels==0)) # remove label for background pixels
    print('Unique labels: {}'.format(unique_labels))
    top_sherd = {}

    for label in unique_labels:
        pixel_coords = np.where(labels==label) # returns [np.array(row_coords), np.array(col_coords)]
        # exclude small clusters and out-of-frame clusters
        if (pixel_coords[0].size <= 2000): 
            continue
        pixel_list = list(zip(pixel_coords[0], pixel_coords[1])) # reorg as [ (r,c), (r,c), (r,c) ]
        xyz = np.array( [point_map[pix[0]][pix[1]] for pix in pixel_list] )
        current_z = get_z(xyz[:,2])
        #current_z = np.average(xyz[:,2])
        # ensure bounding box around pixels is fully within image frame before updating top sherd
        if (not top_sherd) or (current_z < top_sherd['z']):
            top_sherd = {'label': label, 'pixels': pixel_list, 'z': current_z}

    print("Top sherd label: {} and z: {}".format(top_sherd['label'], top_sherd['z']))
  
    # create mask: top sherd pixels --> foreground; else --> background
    top_sherd_mask = np.zeros( labels.shape, np.uint8 )
    for pix in top_sherd['pixels']:
        top_sherd_mask[(pix[0])][(pix[1])] = 255

    # mask out every pixel NOT in top sherd --> final image
    top_sherd_img = cv2.bitwise_and(color_seg_img, color_seg_img, mask=top_sherd_mask)

    # Debugging:
    plt.subplot(121),plt.imshow(top_sherd_mask)
    plt.title('Top Sherd Mask'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(top_sherd_img)
    plt.title('Top Sherd, Segmented'), plt.xticks([]), plt.yticks([])
    plt.show()
    
    return top_sherd_img, top_sherd
    

##############################################################
# apply_watershed(color_seg_img)
# This function applies OpenCV's watershed algorithm for instance segmentation (separating touching/overlapping sherds).
# inputs: color_seg_img (OpenCV RGB image) showing sherd pile only
# outputs: segmented_sherds (OpenCV RGB image)

def apply_watershed(color_seg_img):
    gray = cv2.cvtColor(color_seg_img, cv2.COLOR_RGB2GRAY) # convert to grayscale

    # WATERSHED ON GRADIENT IMAGE
    # denoise image
    denoised = rank.median(gray, disk(2))

    # find continuous region (low gradient -
    # where less than [low_grad_thresh] for this image) --> markers
    # disk(5) is used here to get a more smooth image
    low_grad_thresh = 17
    markers = rank.gradient(denoised, disk(5)) < low_grad_thresh
    markers, num_markers = ndi.label(markers)
    
    # local gradient (disk(2) is used to keep edges thin)
    gradient = rank.gradient(denoised, disk(2))
   
    # process the watershed
    labels = watershed(gradient, markers)
    labels=labels.astype(np.uint8)
    kernel = np.ones((3,3),np.uint8)
    labels = cv2.dilate(labels,kernel,iterations = 1)  # eliminate basins that follow sherd outlines
    
    lock=threading.Lock() # ensure that background pixels are set to 0
    lock.acquire
    labels[gray==0] = 0
    lock.release

    # convert watershed labels (single channel) to RGB image format (3-channel)
    # these colorized labels are not used elsewhere in this script; created only in case they are useful later
    lut=gen_lut()
    rgb_labels = labels2rgb(labels, lut)
    
    # display results
    
    fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(8,8), sharex=True, sharey=True)
    ax = axes.ravel()

    ax[0].imshow(gray, cmap=plt.cm.gray)
    ax[0].set_title("Original in Grayscale")

    ax[1].imshow(gradient, cmap=plt.cm.nipy_spectral)
    ax[1].set_title("Local Gradient")

    ax[2].imshow(markers, cmap=plt.cm.nipy_spectral)
    ax[2].set_title("Markers")

    ax[3].imshow(gray, cmap=plt.cm.gray)
    ax[3].imshow(labels, cmap=plt.cm.nipy_spectral, alpha=.5)
    ax[3].set_title("Segmented")

    for a in ax:
        a.axis('off')

    fig.tight_layout()
    plt.show()
    
    return labels

##############################################################
# locate_sherds(color_seg_img, point_map, header, handle_overlap)
# This function draws bounding boxes around segmented sherds and publishes in [meters] their x,y,z center coordinates, widths, and heights.  It also publishes rotation angles in radians, optimized for the robot end effector.
# inputs: sensor_msgs/Image, sensor_msgs/PointCloud2, std_msgs/Header
# outputs: robot_arm.msg/Detection3DArrayRPY custom ROS message

def locate_sherds(color_seg_img, point_map, header, handle_overlap):
    # Construct Detection3DRPYArray custom ROS message to contain all valid bounding boxes around sherds
    sherds = Detection3DRPYArray()
    sherds.header = header    # meta-data

    # exclude detections smaller than a minimum area?
    use_min_area = False
    min_area = 0.0006  # [sq. meters]
    #print ("Recognizing only rectangles larger than %f sq. meters as sherds" % (min_area) )

    if handle_overlap:
        labels = apply_watershed(color_seg_img)
        top_sherd_img, top_sherd = get_top_sherd(labels, color_seg_img, point_map)
        contours = in_frame_only(top_sherd_img)
    else:
        contours = in_frame_only(color_seg_img)

    if not contours.any():  # if no in-frame contours, return empty sherds message
        return sherds

    #TODO exclude outermost contour? (the sherd container)

    # For each detected contour, find bounding box around each sherd
    for cnt in contours:
        rect = cv2.minAreaRect(cnt) # [(x center, y center), (width, height), (rotation) in pixels]

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
        #z_center = point_map[row_center_pos][col_center_pos][2]
        
        if handle_overlap: # if viewing overlapping sherds
            z_center = top_sherd['z'] # end effector of robot will descend to this height for grasping
        else:
            z_center = None
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
            # Construct a Detection3DRPY() msg to add this bounding box to the list of sherd detections
            detection = Detection3DRPY()
            detection.header = sherds.header
            detection.bbox.center.position.x = x_center
            detection.bbox.center.position.y = y_center
            detection.bbox.center.position.z = z_center
            detection.bbox.center.roll = np.radians(grip_angle)  # radians
            detection.bbox.size.x = width_meter
            detection.bbox.size.y = height_meter
            sherds.detections.append(detection)

            # Debugging: draw, display, and print details of bounding box just added to sherd detections
                        
            box = cv2.boxPoints(rect) # [4x1 array of tuples: coordinates of vertices in pixels]
            box = np.int0(box)
            show_box(color_seg_img, box)
            
            # Extract (x,y) coordinates of box corners for drawing rectangles, starting at "lowest" corner (largest y-coordinate) and moving CW. Height is distance between 0th and 1st corner. Width is distance between 1st and 2nd corner.
            """
            print("Row of center is " + str(row_center_pos))
            print("Col of center is " + str(col_center_pos))
            print("Width endpoints in pixels: (%f, %f) and (%f, %f)." % (col_width_end1, row_width_end1, col_width_end2, row_width_end2) )
            print("Height endpoints in pixels: (%f, %f) and (%f, %f)." % (col_height_end1, row_height_end1, col_height_end2, row_height_end2) )
            print("Center coordinates in meters: (%f, %f)." % (x_center, y_center) )
            print("Width in meters: %f." % (width_meter) )
            print("Height in meters:  %f." % (height_meter) )
            print("Gripper rotation angle is %f degrees." % grip_angle)
            """
    return sherds


##############################################################
# segment_by_color(color_mask, sherds_img, non_sherds_img)
# This function applies a color mask to an image of sherds.  If necessary, it then subtracts a background image of non-sherds from the color-segmented image.  Overlapping and touching sherds are NOT segmented from each other.
# outputs: color_seg_img (OpenCV RGB image)

def segment_by_color(color_mask, sherds_img, non_sherds_img):
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
    color_seg_img = cv2.bitwise_and(rgb, rgb, mask=sherds_mask) # apply mask that allows sherds through

    # Debugging        
    """
    plt.imshow(sherds_mask)
    plt.title("Final Sherds Mask")
    plt.show()
    """

    # Debugging: Display original image and color-segmented image
    """
    plt.subplot(121),plt.imshow(rgb)
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(color_seg_img)
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
        color_seg_img = cv2.bitwise_and(color_seg_img, color_seg_img, mask=nonsherd_mask) # apply mask to remove non-sherds
 
    # Debugging: Display original image and segmented sherds
    """
    if non_sherds_img is not None:
        plt.subplot(121),plt.imshow(rgb)
        plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(segmented_sherds)
        plt.title('Segmented Objects'), plt.xticks([]), plt.yticks([])
        plt.show()
    """
    return color_seg_img


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
    # Segment sherds using HSV color mask and, if handling touching/overlapping sherds, watershed algorithm
    color_seg_img = segment_by_color(req.color_mask, img, non_sherds_img)
    # Locate sherds in image
    point_map = np.zeros( (point_cloud.shape[0], point_cloud.shape[1], 3) )
    point_map[:,:,0] = point_cloud['x']
    point_map[:,:,1] = point_cloud['y']
    point_map[:,:,2] = point_cloud['z']
    res = SherdDetectionsResponse()
    res.detections = locate_sherds(color_seg_img, point_map, header, req.handle_overlap)

    return res

##############################################################
# detect_by_watershed_server()
# This function initiates the detect_sherds_pile ROS node. It subscribes to the 
# subscriptions: /Color_Mask; synced [/Color_Image (camera), /Color-Aligned_PointCloud (camera), /Survey_Stamped]
  
def detect_by_watershed_server():
    rospy.init_node('detect_by_watershed_server')  # initiate 'detect sherds' node

    # Subscribe in sync to Color Image and Color-Aligned PointCloud
    color_img_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    print("Subscribed to color image.")

    # use rs_rgbd.launch with physical RealSense camera
    pointcloud_sub = message_filters.Subscriber("/camera/depth_registered/points", PointCloud2)
    print("Subscribed to point cloud.")

    sync = message_filters.ApproximateTimeSynchronizer([color_img_sub, pointcloud_sub], 1, 0.1, allow_headerless = True)
    sync.registerCallback( camera_data_callback )
    
    #detect_by_watershed_server = rospy.Service('detect_by_watershed_server', SherdDetections, detect_sherds_callback)
    rospy.Service('detect_by_watershed_server', SherdDetections, detect_sherds_callback)
 
    rospy.spin() # keeps Python from exiting until this node is stopped
    
##############################################################
# main function
if __name__ == '__main__':
    detect_by_watershed_server()
