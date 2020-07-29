#!/usr/bin/env python

# Import ROS libraries and message types
import rospy
from geometry_msgs.msg import WrenchStamped
from robot_arm.srv import *

##############################################################
# weight_callback(req)
# This function is the callback for the weight service
# inputs: robot_arm/WeightRequest
# returns: robot_arm/WeightResponse 

def weight_callback(req):
    # Subscribe to force_torque sensor ROS topic (echoed from gazebo topic)
    msg = rospy.wait_for_message("/ft_sensor_topic", WrenchStamped)
    print("Got message from /ft_sensor_topic.")

    res = WeightResponse()
    res.weight = msg.wrench.force.z # [N]

    return res
    
##############################################################
# weigh_sherd()
# This function initiates ROS node that subscribes to the ft_sensor_topic and returns the weight of the sherd
# inputs: none
 
def weigh_sherd():
    rospy.init_node('weigh_sherd')
    weigh_sherd_server = rospy.Service('weigh_sherd', Weight, weight_callback)
    rospy.spin() # simply keeps python from exiting until this node is stopped

##############################################################
# main function
    
if __name__ == '__main__':
    weigh_sherd()
