#!/usr/bin/env python

import rospy
import rosbag

if __name__ == "__main__":
    filename = '/mnt/c/Users/pdame/Downloads/sherd_data_2020-10-15-14-56-02.bag'
    bag = rosbag.Bag(filename)

    timing = {
      'Initialization' : rospy.Duration(0.),
      'Grasping' : rospy.Duration(0.),
      'Locomotion' : rospy.Duration(0.),
      'Data Collection' : rospy.Duration(0.),
      'Planning' : rospy.Duration(0.)
    }
    current_status = ''
    start_time = None
    num_sherds = 0
    try:
        for topic, msg, t in bag.read_messages():
            if topic == '/clock':
                time = msg.clock
            elif topic == '/status':
                if not start_time is None:
                    timing[current_status] += time - start_time
                current_status = msg.data
                start_time = time
            elif topic == '/sherd_data' and not msg.incomplete:
                num_sherds += 1
    finally:
        bag.close()

    for key in timing:
        print('{} took {} secs'.format(key, timing[key].to_sec()))
