import cv2
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.image as mpimg

"""
This script loads a static image from file and displays bounding boxes around contours.
Use to investigate how different findContours arguments behave.
"""

##############################################################
# get_contours(img)
# This function converts the image to grayscale and extracts contours.
# inputs: OpenCV image
# returns: contours (array of arrays)

def get_contours(img):
    gray = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2GRAY) # convert to grayscale
    _, contours, _ = cv2.findContours( gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )

    return contours

##############################################################
# show_contours(img)
# This function...
# inputs:
# returns:

def show_contours(img):
    contours = get_contours(img)
    for cnt in contours:
        rect = cv2.minAreaRect(cnt) # [(x center, y center), (width, height), (rotation) in pixels]
        box = cv2.boxPoints(rect) # [4x1 array of tuples: coordinates of vertices in pixels]
        box = np.int0(box)
        img_contours = cv2.drawContours( np.array(img), [box], 0, (255,0,0), 3 )
        plt.figure()
        plt.imshow(img_contours)
        plt.title("Bounding Box around Sherd")
        plt.show()

##############################################################
# test()
# This function initiates the detect_sherds_pile ROS node. It subscribes to the 
# subscriptions: /Color_Mask; synced [/Color_Image (camera), /Color-Aligned_PointCloud (camera), /Survey_Stamped]
  
def test():
    img = cv2.imread('/home/deb/catkin_ws/src/robot_arm/images/color_bar.jpg', cv2.IMREAD_COLOR)
    show_contours(img)

##############################################################
# main function
if __name__ == '__main__':
    test()
