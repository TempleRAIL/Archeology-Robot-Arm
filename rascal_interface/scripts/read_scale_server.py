#!/usr/bin/env python

# Import ROS libraries and message types
import rospy
from geometry_msgs.msg import WrenchStamped
from rascal_msgs.srv import ScaleReading, ScaleReadingResponse


class ReadScaleServer:
    
    def __init__(self):
        self.server = rospy.Service('read_scale', ScaleReading, self.read_scale_callback)
        self.tare = rospy.get_param('sherd_states/scale_tare')

    ##############################################################
    # read_scale_callback(req)
    # This function is the callback for the read_scale service
    # inputs: rascal_msgs/ScaleReadingRequest
    # returns: rascal_msgs/ScaleReadingResponse 
    def read_scale_callback(self, req):
        # Subscribe to scale_ft_sensor ROS topic (echoed from Gazebo topic)
        msg = rospy.wait_for_message("/scale_ft_sensor", WrenchStamped)
        rospy.logdebug("Got message from /scale_ft_sensor topic.")
        res = ScaleReadingResponse()
        res.mass = -(msg.wrench.force.z / 9.81) - self.tare # [kg]
        return res
    

##############################################################
# main function
    
if __name__ == '__main__':
    rospy.init_node('read_scale_server')
    rss = ReadScaleServer()
    rospy.spin() # simply keeps python from exiting until this node is stopped
