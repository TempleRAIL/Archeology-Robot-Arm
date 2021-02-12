#!/usr/bin/env python

# Import Python libraries
import cv2
from matplotlib import pyplot as plt

# Import ROS libraries and message types
import rospy
from cv_bridge import CvBridge, CvBridgeError  # Convert between ROS image msgs and OpenCV images
from sensor_msgs.msg import Image
from rascal_msgs.srv import Photo, PhotoRequest, PhotoResponse


class TakePhotoServer():

    def __init__(self):
        self.server = rospy.Service('take_photo', Photo, self.take_photo_callback)
        self.bridge = CvBridge()  # OpenCV converter
        self.display = rospy.get_param('sherd_states/show_cam_photo', False)

    ##############################################################
    # take_photo_callback(req)
    # This function is the callback for the read_scale service
    # inputs: robot_arm/PhotoRequest
    # returns: robot_arm/PhotoResponse 

    def take_photo_callback(self, req):
        # Subscribe to archival camera image ROS topic (echoed from gazebo topic)
        if req.camera_type == PhotoRequest.ARCHIVAL:
            msg = rospy.wait_for_message('/archival_camera/image_raw', Image)
            #rospy.logdebug("Got message from /archival_camera/image_raw topic.")
        elif req.camera_type == PhotoRequest.WRIST:
            msg = rospy.wait_for_message('/camera/color/image_raw', Image)
            rospy.logdebug("Got message from /camera/color/image_raw topic.")
        else:
            rospy.logerr("Camera type not defined")
        # Display results if desired
        if self.display:
            cv_photo = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # BGR OpenCV image
            cv_photo = cv_photo[:, :, ::-1]  # flip to RGB for display
            plt.figure()
            plt.imshow(cv_photo)
            if req.which_camera == 'archival':
                plt.title("Archival Photo")
            else:
                plt.title("Photo from Robot Camera")
            plt.show()
        # Return result
        res = PhotoResponse()
        res.image = msg    
        return res

##############################################################
# main function
    
if __name__ == '__main__':
    rospy.init_node('take_photo_server')
    tps = TakePhotoServer()
    rospy.spin() # simply keeps python from exiting until this node is stopped
