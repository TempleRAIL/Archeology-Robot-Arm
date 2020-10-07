#!/usr/bin/env python

# Import ROS libraries and message types
import rospy
from gazebo_msgs.msg import ContactsState
from robot_arm.srv import SherdID, SherdIDResponse

##############################################################
# id_sherd_callback(req)
# This function is the callback for the id_sherd_to_weigh service
# inputs: robot_arm/SherdIDRequest
# returns: robot_arm/SherdIDResponse 

# gazebo_msgs/GetLinkProperties Service: https://docs.ros.org/kinetic/api/gazebo_msgs/html/srv/GetLinkProperties.html

def id_sherd_callback(req):
    # Subscribe to contact sensor ROS topic (echoed from Gazebo topic)
    msg = rospy.wait_for_message("/scale_contact_sensor", ContactsState)
    print("Got message from /scale_contact_sensor topic.")
    body_name = msg.states[0].collision1_name # e.g., "sherd2::sherd_b_link::collision" string
    string_list = body_name.split('::')
    del string_list[2] # delete 'collision' from string list
    link_name = '::'.join(string_list) # e.g., "sherd2::sherd_b_link::

    res = SherdIDResponse()
    res.link_name = link_name
    return res
    
##############################################################
# id_sherd_to_weigh_server()
# This function initiates a ROS service node that subscribes to the scale_contact_sensor ROS topic (echoed from Gazebo topic) and returns the link_name of the body in contact with the scale.
# inputs: none
 
def id_sherd_to_weigh_server():
    rospy.init_node('id_sherd_to_weigh_server')
    id_sherd_to_weigh_server = rospy.Service('id_sherd_to_weigh_server', SherdID, id_sherd_callback)
    rospy.spin() # simply keeps python from exiting until this node is stopped

##############################################################
# main function
    
if __name__ == '__main__':
    id_sherd_to_weigh_server()
