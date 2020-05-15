#!/usr/bin/env python


# Import ROS libraries and message types
import rospy
from std_msgs.msg import Bool

# ROS publisher
calibrate_trigger_pub = rospy.Publisher('Calibrate_Trigger', Bool, queue_size=1)

        
def dummy_calibrate_trigger():
    rospy.init_node('dummy_calibrate_trigger')

    generate_mask = True
    msg = Bool()
    msg.data = generate_mask

    calibrate_trigger_pub.publish(msg)
    print("/Calibrate_Trigger message published.")
  
if __name__ == '__main__':
    dummy_calibrate_trigger()

