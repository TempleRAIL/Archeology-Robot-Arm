#!/usr/bin/env python

# RUN THIS NODE AFTER ARCHAEOLOGY.LAUNCH AND ARM.LAUNCH IN THE ROBOT_ARM PACKAGE TO TEST THE SCALE FUNCTIONALITY

# Import ROS libraries and message types
import rospy
from rascal_msgs.srv import ScaleReading, ScaleReadingRequest, SherdID, SherdIDRequest
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import GetLinkProperties, GetLinkPropertiesRequest



def get_mass_fun():
    scale_ft_mode = rospy.get_param('~scale_ft_mode') # Boolean

    if scale_ft_mode:
        # ROS service clients for FORCE_TORQUE mode
        rospy.wait_for_service('read_scale_server')
        read_scale_srv = rospy.ServiceProxy('read_scale_server', ScaleReading)
        # run read_scale_srv
        req = ScaleReadingRequest()
        try:
            res = read_scale_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr('read_scale service call failed: {}'.format(e))
            raise
     else:
        # ROS service clients for CONTACT mode
        rospy.wait_for_service('id_sherd_to_weigh_server')
        id_sherd_srv = rospy.ServiceProxy('id_sherd_to_weigh_server', SherdID)
        rospy.wait_for_service('gazebo/get_link_properties')
        link_props_srv = rospy.ServiceProxy('gazebo/get_link_properties', GetLinkProperties)
        # run id_sherd_srv to get link name of sherd on scale
        req = SherdIDRequest()
        try:
            res = id_sherd_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr('AutoCore: id_sherd service call failed: {}'.format(e))
            raise
        link_name = res.link_name
        rospy.logwarn('AutoCore: got link name of sherd on scale: {}'.format(link_name))
        # call link_props_srv to get mass of sherd
        req = GetLinkPropertiesRequest()
        req.link_name = link_name
        try:
            res = link_props_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr('AutoCore: id_sherd service call failed: {}'.format(e))
            raise

    mass = res.mass
    print('Got sherd mass: {} kg'.format(mass))
    return mass

##############################################################
# main function
    
if __name__ == '__main__':
    get_mass_fun()
