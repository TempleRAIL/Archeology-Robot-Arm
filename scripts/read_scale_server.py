#!/usr/bin/env python

# Import ROS libraries and message types
import rospy
from geometry_msgs.msg import WrenchStamped
from robot_arm.srv import ScaleReading, ScaleReadingResponse

##############################################################
# read_scale_callback(req)
# This function is the callback for the read_scale service
# inputs: robot_arm/ScaleReadingtRequest
# returns: robot_arm/ScaleReadingResponse 

def read_scale_callback(req):
    # Subscribe to force_torque sensor ROS topic (echoed from gazebo topic)
    tare = rospy.get_param('sherd_states/scale_tare')
    msg = rospy.wait_for_message("/ft_sensor_topic", WrenchStamped)
    print("Got message from /ft_sensor_topic.")
    weight = msg.wrench.force.z # [N]
    mass = -(weight/9.8)-tare # [kg]

    res = ScaleReadingResponse()
    res.mass = mass
    return res
    
##############################################################
# read_scale_server()
# This function initiates ROS service node that subscribes to the ft_sensor_topic and returns the mass of the object on the scale.
# inputs: none
 
def read_scale_server():
    rospy.init_node('read_scale_server')
    read_scale_server = rospy.Service('read_scale_server', ScaleReading, read_scale_callback)
    rospy.spin() # simply keeps python from exiting until this node is stopped

##############################################################
# main function
    
if __name__ == '__main__':
    read_scale_server()
