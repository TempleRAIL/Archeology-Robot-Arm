#!/usr/bin/env python

# Import ROS libraries and message types
import rospy
from gazebo_msgs.msg import ContactsState
from rascal_msgs.srv import SherdID, SherdIDResponse


class SherdMassServer:

    def __init__(self):
        self.server = rospy.Service('sherd_mass', SherdID, self.sherd_id_callback)
    
    ##############################################################
    # id_sherd_callback(req)
    # This function is the callback for the id_sherd_to_weigh service
    # inputs: rascal_msgs/SherdIDRequest
    # returns: rascal_msgs/SherdIDResponse 

    # gazebo_msgs/GetLinkProperties Service: https://docs.ros.org/kinetic/api/gazebo_msgs/html/srv/GetLinkProperties.html
    def sherd_id_callback(self, req):
        attempts = 0
        success = False
        while not success:
            if attempts >= 25:
                break
            msg = rospy.wait_for_message("/scale_contact_sensor", ContactsState)
            if msg.states:
                rospy.logdebug("Got non-empty ContactsState msg from /scale_contact_sensor topic.")
                success = True
            else:
                attempts += 1

        body_name = msg.states[0].collision1_name # e.g., "sherd2::sherd_b_link::collision" string
        string_list = body_name.split('::')
        del string_list[2] # delete 'collision' from string list
        link_name = '::'.join(string_list) # e.g., "sherd2::sherd_b_link::

        res = SherdIDResponse()
        res.link_name = link_name
        return res


##############################################################
# main function
    
if __name__ == '__main__':
    rospy.init_node('sherd_mass')
    sws = SherdMassServer()
    rospy.spin()
