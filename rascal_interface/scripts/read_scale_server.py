#!/usr/bin/env python

# Import ROS libraries and message types
import rospy
from geometry_msgs.msg import WrenchStamped
from rascal_msgs.srv import ScaleReading, ScaleReadingResponse

##############################################################
# read_scale_callback(req)
# This function is the callback for the read_scale service
# inputs: rascal_msgs/ScaleReadingRequest
# returns: rascal_msgs/ScaleReadingResponse 

def read_scale_callback(req):
    tare = rospy.get_param('sherd_states/scale_tare')
    # Subscribe to scale_ft_sensor ROS topic (echoed from Gazebo topic)
    msg = rospy.wait_for_message("/scale_ft_sensor", WrenchStamped)
    print("Got message from /scale_ft_sensor topic.")
    weight = msg.wrench.force.z # [N]
    mass = -(weight/9.8)-tare # [kg]
    res = ScaleReadingResponse()
    res.mass = mass
    return res
    
##############################################################
# read_scale_server()
# This function initiates ROS service node that subscribes to the /scale_ft_sensor (force_torque) topic, then returns the mass of the object on the scale.  ROS topic is echoed from Gazebo topic.
# inputs: none
 
def read_scale_server():
    rospy.init_node('read_scale_server')
    read_scale_server = rospy.Service('read_scale_server', ScaleReading, read_scale_callback)
    rospy.spin() # simply keeps python from exiting until this node is stopped

##############################################################
# main function
    
if __name__ == '__main__':
    read_scale_server()
