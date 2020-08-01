#!/usr/bin/env python

# RUN THIS NODE AFTER ARCHAEOLOGY.LAUNCH AND ARM.LAUNCH IN THE ROBOT_ARM PACKAGE TO TEST THE SCALE FUNCTIONALITY

# Import ROS libraries and message types
import rospy
from robot_arm.srv import ColorMask, ColorMaskRequest, SherdDetections, SherdDetectionsRequest, ScaleReading, ScaleReadingRequest

# ROS service clients
rospy.wait_for_service('read_scale_erver')
scale_srv = rospy.ServiceProxy('read_scale_server', ScaleReading)

def get_mass_fun():
    # run read_scale.py
    req = ScaleReadingRequest()
    try:
        res = scale_srv(req)
    except rospy.ServiceException as e:
        rospy.logerr('read_scale service call failed: {}'.format(e))
        raise
    else:
        mass = res.mass
        print('Got sherd mass: {} kg'.format(mass))
        return mass

##############################################################
# main function
    
if __name__ == '__main__':
    get_mass_fun()
