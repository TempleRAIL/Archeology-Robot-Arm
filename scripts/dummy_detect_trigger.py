#!/usr/bin/env python


# Import ROS libraries and message types
import rospy
from std_msgs.msg import Bool

# ROS publisher
detect_trigger_pub = rospy.Publisher('Detect_Trigger', Bool, queue_size=1)

        
def dummy_detect_trigger():
    rospy.init_node('dummy_detect_trigger')

    detect_sherds = True
    msg = Bool()
    msg.data = detect_sherds

    detect_trigger_pub.publish(msg)
    print("/Detect_Trigger message published.")
  
if __name__ == '__main__':
    dummy_detect_trigger()

