#!/usr/bin/env python

#Import Python libraries
import time
import math
import numpy as np
import random

# Import ROS libraries and message types
#import message_filters
import rospy
import ros_numpy
import tf2_ros
from tf2_geometry_msgs import PointStamped
#from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool, Int16MultiArray
#from vision_msgs.msg import Detection3D, Detection3DArray
from robot_arm.msg import Detection3DRPY, Detection3DRPYArray

class AutoCore():
    
    def __init__(self, bot, gripper):
        
        # Robot passed from state machine
        self.bot = bot
    
        # pick-up area geometry [meters]
        x_offset = 0.22
        x_length = 0.25
        x_rects = 3
        x_sublength = x_length/x_rects
        y_length = 0.5
        y_rects = 4
        y_sublength = y_length/y_rects
        x_centers = [x_offset+0.5*x_sublength, x_offset+1.5*x_sublength, x_offset+2.5*x_sublength]
        y_centers = [-1.5*y_sublength, -.5*y_sublength, .5*y_sublength, 1.5*y_sublength]
       
        
        # class variables
        self.DEF_TRY = 0 # variable used when acquiring sherds to track attempts
        self.DEF_LOOP = 0 # variable used when examining sherd boxes to track unoccupied boxes
        self.DEF_CURRENT = 0 # variable used when examining sherd boxes to track current box 
        self.DEF_STATUS = True # variable used to track when errors occur
        self.DEF_HEIGHT = 0.25  # working height
        self.DEF_MAT = np.array([0.14, -0.24, self.DEF_HEIGHT]) # x,y,z meters
        self.DEF_SCALE = np.array([0.14, 0.3, self.DEF_HEIGHT])  # x,y,z meters
        self.DEF_CAMERA = np.array([0, -0.175, 0])  # x,y,z meters
        self.SCALE_Z = 0.085
        self.CAMERA_Z = self.SCALE_Z

        # construct array of sub-rectangle centers
        DEF_SHARDS = []
        for x in x_centers:
            for y in y_centers:

                center = [x,y, self.DEF_HEIGHT]
                DEF_SHARDS.append(center)
        
        self.DEF_SHARDS = np.array(DEF_SHARDS)
                
        # Array the State Machine uses to know which location to translate to during each transition
        self.DEF_POS = np.array([[0.250, 0, self.DEF_HEIGHT], [0, -0.175, self.DEF_HEIGHT], [0, 0.175, self.DEF_HEIGHT], [-0.250, 0, self.DEF_HEIGHT]])
        
        random.seed()
        self.DEF_DISCARD = np.array([random.uniform(-x_offset-x_length, -x_offset), random.uniform(-y_length/2, y_length/2), self.DEF_HEIGHT])  # random points over discard area 
        self.DEF_ORIENTATION = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]) # placeholder for gripper orientation
        self.DEF_POSITION = np.array([0, 0, 0]) # placeholder for gripper position
        self.DEF_Z = 0 # placeholder for sherd height
        self.DEF_PITCH = np.pi/2  # gripper orthogonal to ground; roll will be defined by sherd rotation angles
        self.DEF_NUMERICAL = False # required argument for bot.arm.set_ee_pose_pitch_roll

    # Function to trigger generation of color mask
    # Captures image of empty background mat
    def calibrateFun(self):
        print("AutoCore.calibrateFun(self) triggered.")
        # ROS publishers
        calibrate_trigger_pub = rospy.Publisher('Calibrate_Trigger', Bool, queue_size=1)
        #calibrate_xyz = np.array( [0.14, -0.24, DEF_HEIGHT] )
        calibrateLoc = {"position": self.DEF_MAT, "pitch": self.DEF_PITCH, "numerical": self.DEF_NUMERICAL} 
        self.moveFun(**calibrateLoc)

        generate_mask = True
        msg = Bool()
        msg.data = generate_mask

        calibrate_trigger_pub.publish(msg)
        print("/Calibrate_Trigger message published.")
        
  
    # Function to look for object in box
    # Searches until found and then retrieves
    def shardFun(self):
        print("Examining surface for sherds")
        print("Loop = ", self.DEF_LOOP)
        heightLoc = {'position': self.DEF_SHARDS[self.DEF_CURRENT], "pitch": self.DEF_PITCH, "roll": 0, "numerical": self.DEF_NUMERICAL} 
        print('position: ', self.DEF_SHARDS[self.DEF_CURRENT])
        try:
            self.bot.arm.set_ee_pose_pitch_roll(**heightLoc)
            time.sleep(1)
        except Exception as e:
            print("Exception to requested pose thrown.")
            print(e)
            self.DEF_STATUS = False
        time.sleep(1)
        sherds = []
        msg = rospy.wait_for_message("/Bounding_Boxes", Detection3DRPYArray)
        detections = msg.detections
        if not detections:
            report = False
        else:
            report = True
            for i in len(detections):
                sherds[i] = [detections[i].bbox.center.x, detections[i].bbox.center.y, detections[i].bbox.center.z, detections[i].bbox.center.theta]
            sherds = np.array(sherds)
        if report: # Place returned variable here
            self.DEF_Z = sherds[0][2] # Place returned height here (from DetectionArray msg)
            self.DEF_ORIENTATION = sherds[0][3] # Place returned angle here (from DetectionArray msg)
            self.DEF_POSITION = np.array([sherds[0][0], sherds[0][1], 0]) # Place returned center position (from DetectionArray msg)
        return report


    # Function to call IK to plot and execute trajectory
    def moveFun(self, **pose):
        print("moveFun triggered.")
        try:
            self.bot.arm.set_ee_pose_pitch_roll(**pose)
            #bot.arm.set_ee_pose(**pose)
            time.sleep(1)
        except:
            print("Exception to moveFun() thrown.")
            self.DEF_STATUS = False


    # Function to check for and return sherd detections as list of lists: [x_center, y_center, rotation_angle]
    def detectFun(self):

        # confirm that color mask exists
        Color_Mask_msg = rospy.wait_for_message("/Color_Mask", Int16MultiArray)
        if Color_Mask_msg.data:
            print("Color mask exists.  Proceed.")
        else:
            print("Color mask does not exist.")
            return False
      
        # run segment_sherds.py on what robot sees in this position
        detect_trigger_pub = rospy.Publisher('Detect_Trigger', Bool, queue_size=1)
        detect_sherd = True
        msg = Bool()
        msg.data = detect_sherd
        detect_trigger_pub.publish(msg)
        print("/Detect_Trigger message published.")
        
        msg = rospy.wait_for_message("/Bounding_Boxes", Detection3DArrayRPY)
        detections = msg.detections
        print("/Bounding_Boxes detections message: ", detections)

        sherds = [] # initialize empty list of lists: [sherd_x, sherd_y, sherd_angle]

        if not detections:
            report = False
            return report, sherds
        else:
            report = True
            tfBuffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tfBuffer)
            rate = rospy.Rate(5.0)
            print("tf_listener created.")

            rospy.sleep(1.0)

            for item in detections:
                point_cam = PointStamped()  # build ROS message for conversion
                point_cam.header.frame_id = "camera_link"

                # get center x,y,z from /Bounding_Boxes detections
                point_cam.point.x, point_cam.point.y, point_cam.point.z = item.bbox.center.position.x, item.bbox.center.position.y, 0

                sherd_angle = item.bbox.center.roll  # get sherd rotation angle

                try:
                    point_base = tfBuffer.transform(point_cam, "arm_base_link")
                except:  # tf2_ros.buffer_interface.TypeException as e:
                    e = sys.exc_info()[0]
                    rospy.logerr(e)
                    sys.exit(1)
                print("Obtained transform between camera_link and arm_base_link.")                
                print("Sherd center point (x,y,z) [m] in arm_base_link frame: ", point_base)
                sherds.append( [point_base.point.x, point_base.point.y, point_base.point.z, sherd_angle] )
            sherds = np.array(sherds)
            print("sherds list = ", sherds)
            return report, sherds


    # Function to retrieve an object
    def pickFun(self, z, **pose):
        print("pickFun triggered.")
        #self.bot.gripper.open()
        self.gripper.open()
        self.moveFun(**pose)  # position gripper at working height
        descend_z = np.array( [0, 0, -(self.DEF_HEIGHT-z)] )  # z-displacement downwards to 3 cm below top face of sherd
        self.bot.arm.move_ee_xyz(descend_z, plan = True)  # move gripper down to sherd
        time.sleep(1)
        #self.bot.gripper.close()
        self.gripper.close()  # close around sherd
        #gripper_state = gripper.get_gripper_state()
        #print("gripper_state = ", gripper_state)
        #if gripper_state == 3:  # '3' is even when gripper has closed around sherd, so this check does not work
            #report = False
            #return report
            #time.sleep(1)

        # Move gripper back up
        self.bot.arm.move_ee_xyz(-descend_z, plan=True)


    # Function to place an object
    def placeFun(self, z, **pose):
        print("placeFun triggered.")

        self.moveFun(**pose)  # position gripper at working height
        descend_z = np.array( [0, 0, -(self.DEF_HEIGHT-z)] )    # z-displacement downwards to z
        self.bot.arm.move_ee_xyz(descend_z, plan=True)  # move gripper down near surface
        time.sleep(2)
        #self.bot.gripper.open()
    	self.gripper.open()
        time.sleep(1)
       #gripper_state = gripper.get_gripper_state()
        #print("gripper_state = ", gripper_state)
 
        # Move gripper back up
        self.bot.arm.move_ee_xyz(-descend_z, plan=True)

        
    def discardFun(self):
        print("Moving over discard area.")
        discardLoc = {"position": self.DEF_DISCARD, "pitch": self.DEF_PITCH, "roll": 0, "numerical": self.DEF_NUMERICAL}    
        self.moveFun(**discardLoc)
        
        
